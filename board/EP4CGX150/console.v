module console #(
	parameter NI = 8,
	parameter NO = 3
) (
	input							clkcons,
	input				[15:0]	pc,
	input				[15:0]	datapath,
	input				[NI:0]	sigi, // 0 - CPU_RUN, 1 - CPU_WAIT, 2 - MMU18, 3 - MMU22, 4 - CPUSLOW, 5,6 - CPU_MODE, 7 - MASTER, 8 - DATA
	output			[NO:0]	sigo, // 0 - RESET, 1 - CONT, 2 - CPUSLOW, 3 - HALT
	
	input							clk_p,
	input							sys_init,
	input				[21:0]	wb_adr_i,
	input				[15:0]	wb_dat_i,
	output	reg	[15:0]	wb_dat_o,
	input							bus_stb,
	input							wb_we_i,
	output	reg				cons_ack,
	
	output	reg				dma_req,
	input							dma_gnt,
   output	reg	[17:0]	dma_adr_o,  // выходной адрес при DMA-обмене
   input				[15:0]	dma_dat_i,  // входная шина данных DMA
   output	reg	[15:0]	dma_dat_o,  // выходная шина данных DMA
   output	reg				dma_stb_o,  // строб цикла шины DMA
   output	reg				dma_we_o,   // направление передачи DMA (0 - dev->cons, 1 - cons->dev) 
   input							dma_ack_i,  // Ответ от устройства, с которым идет DMA-обмен
	
	output	reg	[15:0]	startup_adr,
	output	reg				cons_start,
	
	
	output 	reg	[2:0]		cons_row,
	inout		reg	[11:0]	cons_col,
	output	reg	[5:0]		cons_ledrow
) ;

`ifdef bootrom_module
	initial startup_adr	= 16'o165020 ;
`else
	initial startup_adr	= 16'o140000 ;
`endif

	initial cons_start	= 1'b0 ;

	reg [15:0] display_reg_r, display_reg_ff ;
	reg [15:0] pc_r, pc_ff ;
	reg [15:0] datapath_r, datapath_ff ;
	
	reg [NI:0] sigi_r, sigi_ff ;

	wire			cpu_run	= sigi_r[0] ;
	wire			cpu_wait	= sigi_r[1] ;
	wire			mmu18		= sigi_r[2] ;
	wire			mmu22		= sigi_r[3] ;
	wire			mmu16		= ~(mmu18 | mmu22) ;
	wire			cpu_slowi = sigi_r[4] ;
	wire	[2:0] cpu_mode	= sigi_r[6:5] == 2'b00 ? 3'b001 : sigi_r[6:5] == 2'b01 ? 3'b010 : sigi_r[6:5] == 2'b11 ? 3'b100 : 3'b111 ;
	wire			bus_master		= sigi_r[7] ;
	wire			idd 				= sigi_r[8] ;
	
	reg clk = 0 ;
	reg [12:0] counter = 13'd00 ;

	always @(posedge clkcons) begin
		counter <= counter + 1'd1 ;
		if (counter == 13'd499) begin
			counter <= 13'd0 ;
			clk = ~clk ;
		end
	end

	reg [15:0] display_reg ;
	reg [15:0] data_reg ;

	always @(posedge clk) begin
		pc_r <= pc_ff ;
		pc_ff <= pc ;
		
		datapath_r <= datapath_ff ;
		datapath_ff <= datapath ;
		
		sigi_r <= sigi_ff ;
		sigi_ff <= sigi ;
	end

	 /*
	 * PANEL INTERFACE
	 */

	reg [11:0] led_bits[5:0] ;
	reg [11:0] switch_bits[2:0] ;
	
	reg [15:0]	data_leds 		= 16'd0 ;
	reg [21:0]	address_leds	= 22'd0 ;
	reg			addr_err			= 1'b0 ;
	reg			parity_err		= 1'b0 ;
	wire [1:0]	parity 					 ;
	reg [7:0]	cons_mode		= 8'o10 ;
	reg [3:0]	disp_mode		= 4'b0001 ;
	
	wire [21:0]	switch_sr = {switch_bits[1][9:0], switch_bits[0]} ;
	wire test_mode = ~switch_bits[2][0] ;
	wire load_addr = switch_bits[2][1] ;
	wire exam		= switch_bits[2][2] ;
	wire dep			= switch_bits[2][3] ;
	wire reset		= switch_bits[1][11] ;
	wire cont		= switch_bits[2][4] ;
	wire cpu_slowo	= switch_bits[2][6] ;
	wire cpu_halt	= switch_bits[2][5] ;
	wire cons_rst	= switch_bits[1][10] ;
	wire start		= switch_bits[2][7] ;
	
	assign sigo = {cpu_halt, cpu_slowo, cont, reset} ;
	assign parity = cpu_slowi ? 2'b01 : 2'b10 ;
	
	reg [21:0] address_reg = 22'b0 ;
	reg address_reg_act = 1'b0 ;

	initial begin
		led_bits[0] = 12'd0 ;
		led_bits[1] = 12'd0 ;
		led_bits[2] = 12'd0 ;
		led_bits[3] = 12'd0 ;
		led_bits[4] = 12'd0 ;
		led_bits[5] = 12'd0 ;
		
		cons_row = 3'bZZZ ;
		cons_ledrow = 6'bZZZZZZ ;
		cons_col = 12'bZZZZZZZZZZZZ ;
	end

	always @(posedge clk) begin
		if (test_mode | cons_rst) begin
			led_bits[0] = 12'o7777 ;
			led_bits[1] = 12'o7777 ;
			led_bits[2] = 12'o7777 ;
			led_bits[3] = 12'o7777 ;
			led_bits[4] = 12'o7777 ;
			led_bits[5] = 12'o7777 ;
		end else begin
			address_leds <= address_reg_act ? address_reg : {6'b0, pc_r} ;
			data_leds <= ~cpu_run ? data_reg : disp_mode == 4'b0001 ? datapath_r : disp_mode == 4'b1000 ? display_reg : 8'b0 ;
		
			{led_bits[1], led_bits[0]} <= {2'b0, address_leds} ;
			led_bits[2] <= {parity_err, addr_err, cpu_run, cpu_wait, bus_master, cpu_mode, idd, mmu16, mmu18, mmu22} ;
			{led_bits[4], led_bits[3]} <= {disp_mode[1:0], cons_mode[3:0], parity, data_leds} ;
			led_bits[5][11:6] <= {disp_mode[3:2], cons_mode[7:4]} ;
		end
	end

	reg [5:0]	current_row = 6'd0 ;
	
	reg rot_first_time = 1'b1 ;
	reg rot_last_clk_a, rot_last_clk_d, rot_clk, rot_dat ;

	// 1000 Hz, 1ms
	always @(posedge clk) begin
		current_row <= current_row + 1'd1 ;
		if (current_row == 6'd16)
			current_row <= 6'd0 ;
			
		case (current_row)
			6'd0	: begin
				cons_row <= 3'bZZZ ;
				cons_col <= ~led_bits[0] ;
				cons_ledrow <= 6'b000001 ;
			end

			6'd2	: begin
				cons_col <= ~led_bits[1] ;
				cons_ledrow <= 6'b000010 ;
			end

			6'd4	: begin
				cons_col <= ~led_bits[2] ;
				cons_ledrow <= 6'b000100 ;
			end

			6'd6	: begin
				cons_col <= ~led_bits[3] ;
				cons_ledrow <= 6'b001000 ;
			end

			6'd8	: begin
				cons_col <= ~led_bits[4] ;
				cons_ledrow <= 6'b010000 ;
			end

			6'd10	: begin
				cons_col <= ~led_bits[5] ;
				cons_ledrow <= 6'b100000 ;
			end
			
			6'd11	: begin
				cons_ledrow <= 6'bZZZZZZ ;
				cons_col <= 12'bZZZZZZZZZZZZ ;
				cons_row <= 3'b110 ;
			end

			6'd12	: switch_bits[0]	<= ~cons_col ;
			6'd13	: cons_row			<= 3'b101 ;
			6'd14	: switch_bits[1]	<= ~cons_col ;
			6'd15	: cons_row			<= 3'b011 ;
			6'd16	: begin
				switch_bits[2]	<= ~cons_col ;
				
				if (rot_first_time) begin
					rot_first_time <= 1'b0 ;
					rot_last_clk_a <= ~cons_col[8] ;
					rot_last_clk_d <= ~cons_col[10] ;
				end else begin
					rot_clk = ~cons_col[8] ;
					rot_dat = ~cons_col[9] ;
					if (rot_clk != rot_last_clk_a && rot_clk == 1'b1) begin
						if (rot_dat == 1'b0)
							cons_mode <= {cons_mode[0], cons_mode[7:1]} ;
						else
							cons_mode <= {cons_mode[6:0], cons_mode[7]} ;
					end
					rot_last_clk_a <= ~cons_col[8] ;
					
					rot_clk = ~cons_col[10] ;
					rot_dat = ~cons_col[11] ;
					if (rot_clk != rot_last_clk_d && rot_clk == 1'b1) begin
						if (rot_dat == 1'b0)
							disp_mode <= {disp_mode[0], disp_mode[3:1]} ;
						else
							disp_mode <= {disp_mode[2:0], disp_mode[3]} ;
					end
					rot_last_clk_d <= ~cons_col[10] ;
				end
			end
		endcase
		
		if (cpu_run) begin
			address_reg_act <= 1'b0 ;
		end else if (load_addr) begin
			address_reg_act <= 1'b1 ;
		end
	end
	
	wire cons_stb = bus_stb & (wb_adr_i[15:1] == 15'o77674) ; // 177570 >> 1
	
	always @(posedge clk_p) begin
		if (sys_init | cons_rst) begin
			cons_ack <= 1'b0 ;
			display_reg <= 16'b0 ;
		end else if (cons_stb) begin
			if (wb_we_i)
				display_reg <= wb_dat_i ;
			else
				wb_dat_o <= switch_sr[15:0] ;
			cons_ack <= 1'b1 ;
		end else begin
			cons_ack <= 1'b0 ;
		end
	end
	
	wire	load_req ;
	reg 	load_ack = 1'b0 ;

	key_handler load_handler(
		.clk(clk_p),
		.ena(~cpu_run),
		.key(load_addr),
		.ack(load_ack),
		.signal(load_req)
	) ;

	wire	exam_req ;
	reg 	exam_ack = 1'b0 ;
	
	key_handler exam_handler(
		.clk(clk_p),
		.ena(~cpu_run),
		.key(exam),
		.ack(exam_ack),
		.signal(exam_req)
	) ;

	wire	dep_req ;
	reg	dep_ack	= 1'b0 ;

	key_handler dep_handler(
		.clk(clk_p),
		.ena(~cpu_run),
		.key(dep),
		.ack(dep_ack),
		.signal(dep_req)
	) ;
	
	wire	start_req ;
	reg	start_ack	= 1'b0 ;

	key_handler start_handler(
		.clk(clk_p),
		.ena(~cpu_run),
		.key(start),
		.ack(start_ack),
		.signal(start_req)
	) ;

	localparam CS_IDLE		= 4'd0 ;
	localparam CS_DMA_REQ	= 4'd1 ;
	localparam CS_EXAM_1		= 4'd2 ;
	localparam CS_EXAM_2		= 4'd3 ;
	localparam CS_DEP_1		= 4'd4 ;
	localparam CS_DEP_2		= 4'd5 ;
	localparam CS_DONE		= 4'd6 ;
	localparam CS_LOAD		= 4'd7 ;
	localparam CS_START		= 4'd8 ;
	
	reg [3:0] cons_state = CS_IDLE ;
	reg exam_last = 1'b0, dep_last = 1'b0 ;
	
	always @(posedge clk_p) begin
		if (sys_init | cons_rst) begin
			dma_we_o		<= 1'b0 ;
			dma_req		<= 1'b0 ;
			dma_stb_o	<= 1'b0 ;
			data_reg		<= 16'b0 ;
			exam_ack		<= 1'b0 ;
			dep_ack		<= 1'b0 ;
			exam_last	<= 1'b0 ;
			dep_last		<= 1'b0 ;
			start_ack	<= 1'b0 ;
			cons_state <= CS_IDLE ;
		end else begin
			if (cpu_run) begin
				exam_last	<= 1'b0 ;
				dep_last		<= 1'b0 ;
				data_reg		<= 16'b0 ;
				address_reg	<= 22'b0 ;
			end
			
			case (cons_state)
				CS_IDLE : begin
					if (load_req == 1'b0)
						load_ack <= 1'b0 ;

					if (exam_req == 1'b0)
						exam_ack <= 1'b0 ;
					
					if (dep_req == 1'b0)
						dep_ack	<= 1'b0 ;
						
					if (exam_req) begin
						dep_last <= 1'b0 ;
					end
					
					if (cons_start)
						start_ack <= 1'b0 ;
					
					if (dep_req) begin
						exam_last <= 1'b0 ;
					end
					
					if (load_req) begin
						load_ack <= 1'b1 ;
						cons_state <= CS_LOAD ;
					end
					
					if (exam_req | dep_req) begin
						dma_req <= 1'b1 ;
						cons_state <= CS_DMA_REQ ;
					end
					
					if (start_req) begin
						startup_adr <= address_reg[15:0] ;
						cons_start <= 1'b1 ;
						start_ack <= 1'b1 ;
						cons_state <= CS_START ;
					end
				end
				
				CS_DMA_REQ : begin
					if (dma_gnt) begin
						if (exam_req) begin
							if (exam_last)
								address_reg <= address_reg + 2'd2 ;
							cons_state <= CS_EXAM_1 ;
						end else
							if (dep_req) begin
								if (dep_last)
									address_reg <= address_reg + 2'd2 ;
								cons_state <= CS_DEP_1 ;
							end else begin
								exam_ack <= 1'b0 ;
								dep_ack	<= 1'b0 ;
								dma_req <= 1'b0 ;
								dma_stb_o <= 1'b0 ;
								cons_state <= CS_IDLE ;
							end
					end
				end
				
				CS_EXAM_1 : begin
					exam_last <= 1'b1 ;
					exam_ack <= 1'b1 ;
					dma_adr_o <= {address_reg[17:1], 1'b0} ;
					dma_we_o <= 1'b0 ;
					dma_stb_o <= 1'b1 ;
					cons_state <= CS_EXAM_2 ;
				end
				
				CS_EXAM_2 : begin
					if (dma_ack_i) begin
						data_reg <= dma_dat_i ;
						dma_stb_o <= 1'b0 ;
						dma_req <= 1'b0 ;
						cons_state <= CS_DONE ;
					end
				end
				
				CS_DEP_1 : begin
					dep_last <= 1'b1 ;
					dep_ack <= 1'b1 ;
					dma_adr_o <= {address_reg[17:1], 1'b0} ;
					dma_dat_o <= switch_sr[15:0] ;
					dma_we_o <= 1'b1 ;
					dma_stb_o <= 1'b1 ;
					cons_state <= CS_DEP_2 ;
				end
				
				CS_DEP_2 : begin
					if (dma_ack_i) begin
						data_reg <= switch_sr[15:0] ;
						dma_we_o <= 1'b0 ;
						dma_stb_o <= 1'b0 ;
						dma_req <= 1'b0 ;
						cons_state <= CS_DONE ;
					end
				end
				
				CS_DONE : begin
					if (dma_gnt == 1'b0)
						cons_state <= CS_IDLE ;
				end
				
				CS_LOAD : begin
					if (load_req == 1'b0) begin
						address_reg <= switch_sr ;
						load_ack <= 1'b0 ;
						exam_last	<= 1'b0 ;
						dep_last		<= 1'b0 ;
						cons_state <= CS_IDLE ;
					end
				end
				
				CS_START : begin
					if (start_req == 1'b0) begin
						cons_start <= 1'b0 ;
						start_ack <= 1'b0 ;
						cons_state <= CS_IDLE ;
					end
				end
			endcase
		end
	end

endmodule

module key_handler (
	input			clk,
	input			ena,
	input			key,
	input			ack,
	output reg	signal
) ;
	initial signal = 1'b0 ;
	
	reg prev = 1'b0 ;
	reg val	= 1'b0 ;
	
	always @(posedge clk) begin
		if (ack) begin
			signal <= 1'b0 ;
		end else begin
			prev <= val ;
			val <= key ;
			
			if (ena && prev == 1'b1 && val == 1'b0) begin
				signal <= 1'b1 ;
			end
		end
	end
	
endmodule
