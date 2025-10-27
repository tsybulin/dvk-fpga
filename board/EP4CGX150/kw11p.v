module kw11p(
	input							clk_p,
	input							sys_init,

	// BUS
	input				[21:0]	wb_adr_i,
	input				[15:0]	wb_dat_i,
	output	reg	[15:0]	wb_dat_o,
	input							bus_stb,
	input							wb_we_i,
	input				[1:0]		wb_sel_i,
	output	reg				kw11p_ack,

   // IRQ
	output reg             irq,         // запрос
   input                  iack        // подтверждение
) ;

	wire [14:0] adr_i = wb_adr_i[15:1] ;
	wire kw11p_stb = bus_stb & ((adr_i == 15'o75260) || (adr_i == 15'o75261) || (adr_i == 15'o75262)) ; // adr >> 1

	reg [15:0]	csr = 16'b0,  // 172540
					csb = 16'b0,  // 172542
					ctr = 16'b0 ; // 172544, read-only

	reg [19:0] counter = 20'b0 ;
	reg [19:0] divider = 20'd833333 ;
	
	wire [1:0] rate = csr[2:1] ;
					
	always @(posedge clk_p) begin
		if (sys_init) begin
			kw11p_ack <= 1'b0 ;
			csr = 16'b0 ;
			irq <= 1'b0 ;
		end else if (kw11p_stb) begin
			if (wb_we_i) begin
				if (wb_sel_i[0])
					case (adr_i)
						15'o75260 : begin
							csr[6:0] = wb_dat_i[6:0] ; // bits 15 cleared, 7 read-only, 8..14 unused
							csr[15] = 1'b0 ;
						end
						
						15'o75261 : begin
							csb[7:0] <= wb_dat_i[7:0] ;
							ctr[7:0] = wb_dat_i[7:0] ;
						end
					endcase
					
				if (wb_sel_i[1])
					case (adr_i)
						15'o75260 : csr[15] = 1'b0 ;
						15'o75261 : begin
							csb[15:8] <= wb_dat_i[15:8] ;
							ctr[15:8] = wb_dat_i[15:8] ;
						end
					endcase
			end else begin
				case (adr_i)
					15'o75260 : begin
						wb_dat_o <= csr & ~16'o40 ;
						csr[15] = 1'b0 ;
						counter <= 16'b0 ;
					end
					15'o75261 : wb_dat_o <= csb ;
					15'o75262 : wb_dat_o <= ctr ;
				endcase
			end
			
			kw11p_ack <= 1'b1 ;
		end else
			kw11p_ack <= 1'b0 ;
			
		if (iack)
			irq <= 1'b0 ;
			
		if (~sys_init & csr[0] & ~csr[15]) begin
			case (rate)
				2'b00 : divider = 20'd499 ; // 100 kHz
				2'b01 : divider = 20'd4999 ; // 10 kHz
				2'b10 : divider = 20'd833333 ; // line 60 Hz 833334
				2'b11 : divider = 20'hFFFFF ; // external
			endcase
		end
		
		counter <= counter + 1'd1 ;
		
		if (counter == divider) begin
			counter <= 20'b0 ;
			
			if (csr[4]) begin // UP
				if (rate != 2'b11) begin
					ctr = ctr + 1'd1 ;
				end else if (csr[5]) begin
					ctr = ctr + 1'd1 ;
					csr[5] = 1'b0 ;
				end
			end else begin // DOWN
				if (rate != 2'b11) begin
					ctr = ctr - 1'd1 ;
				end else if (csr[5]) begin
					ctr = ctr - 1'd1 ;
					csr[5] = 1'b0 ;
				end
			end
			
			// OVER/UNDER-FLOW?
			if ((csr[4] && ctr != 16'o177777) || (~csr[4] && ctr > 16'b0)) begin
				// yes
			end else begin
				csr[7] = 1'b1 ;
				
				if (csr[6])
					irq <= 1'b1 ;
				else
					irq <= 1'b0 ;
					
				// REPEAT?
				if (csr[3])
					ctr = csb ;
				else
					csr[0] = 1'd0 ;
			end
		end
	end
endmodule
