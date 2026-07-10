module toy11(
	input							clk_p,
	input							sys_init,
	input				[21:0]	wb_adr_i,
	input				[15:0]	wb_dat_i,
	output	reg	[15:0]	wb_dat_o,
	input							bus_stb,
	input							wb_we_i,
	input				[1:0]		wb_sel_i,
	output	reg				toy_ack,

	inout							i2c_SDA,		//SDA line
	inout							i2c_SCL		//SCL line
) ;
	wire [14:0] adr_i = wb_adr_i[15:1] ;
	wire toy_stb = bus_stb & ((adr_i == 15'o77653) || (adr_i == 15'o77654) || (adr_i == 15'o77655) || (adr_i == 15'o77656)) ; // 177526..177534

	//  [0] : go read
	//  [1] : go write
	//  [7] : i2c done
	// [15] : i2c err
	reg [15:0] csr = 16'o200 ; // command-status register 177526

	reg i2cdone = 1'b1 ; 
	wire i2cerr ;
	
	reg [7:0]	seconds_r, seconds_o,
					minutes_r, minutes_o,
					hours_r,   hours_o,
					days_r,    days_o,
					months_r,  months_o,
					years_r,   years_o ;
					
	always @(posedge clk_p) begin
			csr[1:0] <= 2'b0 ;

			if (sys_init) begin
				toy_ack <= 1'b0 ;
				csr <= 16'b0 ;
			end else if (toy_stb) begin
				{csr[15], csr[7]} <= {i2cerr, i2cdone} ;
				
				if (wb_we_i) begin
					if (wb_sel_i[0])
						case (adr_i)
							15'o77653 : csr[1:0] <= wb_dat_i[1:0] ;
							15'o77654 : seconds_o <= wb_dat_i[7:0] ;
							15'o77655 : hours_o <= wb_dat_i[7:0] ;
							15'o77656 : months_o <= wb_dat_i[7:0] ;
						endcase
					
					if (wb_sel_i[1])
						case (adr_i)
							15'o77654 : minutes_o <= wb_dat_i[15:8] ;
							15'o77655 : days_o <= wb_dat_i[15:8] ;
							15'o77656 : years_o <= wb_dat_i[15:8] ;
						endcase
				end else begin
					case (adr_i)
						15'o77653 : wb_dat_o <= csr ;
						15'o77654 : wb_dat_o <= {minutes_r, seconds_r} ;
						15'o77655 : wb_dat_o <= {days_r, hours_r} ;
						15'o77656 : wb_dat_o <= {years_r, months_r} ;
					endcase
				end
				
				toy_ack <= 1'b1 ;
			end else
				toy_ack <= 1'b0 ;
	end
	
	/*
	*		**** I2C ****
	*/
	
	reg start = 1'b0 ;
	reg read_write = 1'b0 ;
	reg stop = 1'b0 ;
	reg [7:0] data_o = 8'b0 ;
	wire [7:0] data_i ;
	
	wire data_ready, data_valid, data_o_last ;
	wire busy, cmd_ready ;
	reg cmd_valid = 1'b0 ;
	
	wire scl_t, sda_t ;
	wire scl_o, sda_o ;
	
	wire scl_i = i2c_SCL;
	assign i2c_SCL = scl_t ? 1'bz : scl_o;
	wire sda_i = i2c_SDA;
	assign i2c_SDA = sda_t ? 1'bz : sda_o;

	i2c_master i2cmaster(
		.clk(clk_p),
		.rst(sys_init),
		
		.s_axis_cmd_address(7'o150),
		.s_axis_cmd_start(start),
		.s_axis_cmd_read(read_write),
		.s_axis_cmd_write(~read_write),
		.s_axis_cmd_write_multiple(1'b0),
		.s_axis_cmd_stop(stop),
		.s_axis_cmd_valid(cmd_valid),
		.s_axis_cmd_ready(cmd_ready),
		
		.s_axis_data_tdata(data_o),
		.s_axis_data_tvalid(1'b1),
		.s_axis_data_tlast(1'b1),
		
		.s_axis_data_tready(data_ready),
		
		.m_axis_data_tdata(data_i),
		.m_axis_data_tvalid(data_valid),
		.m_axis_data_tready(1'b1),
		.m_axis_data_tlast(data_o_last),
		
		.busy(busy),
		.bus_control(),
		.bus_active(),
		.missed_ack(i2cerr),
		
		.prescale(16'd125),
		.stop_on_idle(1'b1),
		
		.scl_i(scl_i),
		.scl_o(scl_o),
		.scl_t(scl_t),
		.sda_i(sda_i),
		.sda_o(sda_o),
		.sda_t(sda_t)
	) ;

	localparam	TS_IDLE				= 4'd0 ;
	localparam	TS_SET_READ_REG	= 4'd1 ;
	localparam	TS_READ_CMD			= 4'd2 ;
	localparam	TS_READ_BYTE		= 4'd3 ;
	localparam	TS_READ_DONE		= 4'd4 ;
	localparam	TS_SET_WRITE_REG	= 4'd5 ;
	localparam	TS_PREP_WRDATA		= 4'd6 ;
	localparam	TS_WRITE_CMD		= 4'd11 ;
	localparam	TS_WRITE_BYTE		= 4'd12 ;

	reg [3:0]	ts = TS_IDLE ;
	reg [3:0]	byte_counter = 4'b0 ;
					
	always @(posedge clk_p) begin
		if (sys_init) begin
			ts <= TS_IDLE ;
			i2cdone <= 1'b1 ;
			start <= 1'b0 ;
			stop <= 1'b0 ;
			cmd_valid <= 1'b0 ;
		end else begin
			case (ts)
				TS_IDLE : begin
					if (csr[0])
						ts <= TS_SET_READ_REG ;
					else if (csr[1]) begin
						ts <= TS_PREP_WRDATA ;
					end ;
				end
				
				TS_SET_READ_REG : begin
					i2cdone <= 1'b0 ;
					read_write <= 1'b0 ;
					data_o <= 8'd0 ;
					cmd_valid <= 1'b1 ;
					start <= 1'b1 ;
					ts <= TS_READ_CMD ;
				end
				
				TS_READ_CMD : begin
					start <= 1'b0 ;
					stop <= 1'b1 ;
					
					if (cmd_ready) begin
						read_write <= 1'b1 ;
						start <= 1'b1 ;
						stop <= 1'b0 ;
						byte_counter <= 4'd7 ;
						ts <= TS_READ_BYTE ;
					end
				end
				
				TS_READ_BYTE : begin
					start <= 1'b0 ;
					
					if (data_valid) begin
						byte_counter <= byte_counter - 1'b1 ;
						start <= 1'b1 ;
						
						case (byte_counter)
							4'd7 : seconds_r	<= data_i ;
							4'd6 : minutes_r	<= data_i ;
							4'd5 : hours_r		<= data_i ;
							4'd3 : days_r		<= data_i ;
							4'd2 : months_r	<= data_i ;
							4'd1 : begin
								years_r <= data_i ;

								cmd_valid <= 1'b0 ;
								start <= 1'b0 ;
								stop <= 1'b1 ;
								ts <= TS_READ_DONE ;
							end
						endcase
					end
				end
				
				TS_READ_DONE : begin
					i2cdone <= 1'b1 ;
					ts <= TS_IDLE ;
				end
				
				TS_PREP_WRDATA : begin
					i2cdone <= 1'b0 ;
					ts <= TS_SET_WRITE_REG ;
				end
				
				TS_SET_WRITE_REG : begin
					i2cdone <= 1'b0 ;
					read_write <= 1'b0 ;
					data_o <= 8'd0 ;
					start <= 1'b1 ;
					stop <= 1'b0 ;
					byte_counter <= 4'd8 ;
					ts <= TS_WRITE_CMD ;
				end
				
				TS_WRITE_CMD : begin
					cmd_valid <= 1'b1 ;
					
					if (cmd_ready) begin
						ts <= TS_WRITE_BYTE ;
					end
				end
				
				TS_WRITE_BYTE : begin
					start <= 1'b0 ;
					
					if (cmd_ready) begin
						byte_counter <= byte_counter - 1'b1 ;
						start <= 1'b1 ;
						
						case (byte_counter)
							4'd8 : data_o  <= 8'd0 ; //register
							4'd7 : data_o  <= seconds_o ; // seconds
							4'd6 : data_o	<= minutes_o ; // minutes
							4'd5 : data_o	<= hours_o ; // hours
							4'd4 : data_o  <= 8'd1  ; // weekdays unused
							4'd3 : data_o	<= days_o ; // days
							4'd2 : data_o	<= months_o ; // months
							4'd1 : begin
								data_o	<= years_o ; // years
								cmd_valid <= 1'b0 ;
								start <= 1'b0 ;
								stop <= 1'b1 ;
								i2cdone <= 1'b1 ;
								ts <= TS_IDLE ;
							end
						endcase
					end
				end
			endcase
		end
	end
	
endmodule
