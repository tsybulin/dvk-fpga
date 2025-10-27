module lp11(
	input							clk_p,
	input							sys_init,

	// BUS
	input				[21:0]	wb_adr_i,
	input				[15:0]	wb_dat_i,
	output	reg	[15:0]	wb_dat_o,
	input							bus_stb,
	input							wb_we_i,
	input				[1:0]		wb_sel_i,
	output	reg				lp11_ack,
	
   // IRQ
	output reg             irq,         // запрос
   input                  iack        // подтверждение
) ;

	localparam CSR_ADR = 15'o77646 ;
	localparam DBR_ADR = 15'o77647 ;

	wire [14:0] adr_i = wb_adr_i[15:1] ;
	wire lp11_stb = bus_stb & ((adr_i == CSR_ADR) || (adr_i == DBR_ADR)) ;
	
	reg [15:0] csr = 16'b0 ; // command-status 177514
	reg [15:0] dbr = 16'b0 ; // data 177516
	
	localparam IS_IDLE = 2'd0 ;    // ожидание прерывания
	localparam IS_REQ  = 2'd1 ;     // запрос векторного прерывания
	localparam IS_WAIT = 2'd2 ;    // ожидание обработки прерывания со стороны процессора

	reg[1:0] interrupt_state = IS_IDLE ;
	reg interrupt_trigger = 1'b0 ;
	wire ie = csr[6] ; // interrupt enabled

	always @(posedge clk_p) begin
		if (sys_init) begin
			csr <= 16'o200 ;
			dbr <= 16'b0 ;
			irq <= 1'b0 ;
			lp11_ack	<= 1'b0 ;
			interrupt_trigger <= 1'b0 ;
			interrupt_state <= IS_IDLE ;
		end else begin
			case (interrupt_state)
				IS_IDLE : begin
					if (ie & interrupt_trigger) begin
						irq <= 1'b1 ;
						interrupt_state <= IS_REQ ;
					end else
						irq <= 1'b0 ;
				end
				
				IS_REQ : begin
					if (~ie) begin
						interrupt_trigger <= 1'b0 ;
						interrupt_state <= IS_IDLE ;
					end else if (iack) begin
						irq <= 1'b0 ;
						interrupt_trigger <= 1'b0 ;
						interrupt_state <= IS_WAIT ;
					end
				end
				
				IS_WAIT :
					if (~iack)
						interrupt_state <= IS_IDLE ;
			endcase
			
			if (lp11_stb) begin
				if (wb_we_i) begin
					if (wb_sel_i[0]) begin
						case (adr_i)
							CSR_ADR : begin
								csr[6] <= wb_dat_i[6] ;
								if (wb_dat_i[6] & csr[7])
									interrupt_trigger <= 1'b1 ;
							end
							
							DBR_ADR : begin
								if (csr[15]) begin
									if (ie)
										interrupt_trigger <= 1'b1 ;
								end else begin
									dbr[7:0] <= wb_dat_i[7:0] ;
									csr[7] <= 1'b0 ;
								end
							end
						endcase
					end
				end else begin
					case (adr_i)
						CSR_ADR : wb_dat_o <= csr ;
						DBR_ADR : wb_dat_o <= dbr ;
					endcase
				end
				
				lp11_ack <= 1'b1 ;
			end else begin
				lp11_ack <= 1'b0 ;
				
				// print emu
				if (~csr[7]) begin
					csr[7] <= 1'b1 ;
						if (ie)
							interrupt_trigger <= 1'b1 ;
				end
			end
			
		end
	end
	
endmodule
