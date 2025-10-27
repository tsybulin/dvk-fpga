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
	
	reg [15:0] csr = 16'o200 ; // command-status register 177526

	reg [15:0]	dar_o = 16'b0, // 177530 : date register AAMMMMDD DDDYYYYY
					dar_i = 16'b0 ;
	reg [31:0]	tr_o  = 32'b0, // 177532 LSV .. 177534 HSV : time register 32-bit, time of day in 60Hz ticks
					tr_i  = 32'b0 ;
					
	reg [31:0] tr_o_ff ;
	reg [15:0] dar_o_ff ;
	
	reg i2cdone = 1'b1 ; 
	wire i2cerr ;
	
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
							15'o77653 : begin
								csr[1:0] <= wb_dat_i[1:0] ;
							end

							15'o77654 : begin
								dar_o[7:0] <= wb_dat_i[7:0] ;
							end

							15'o77655 : begin
								tr_o[7:0] <= wb_dat_i[7:0] ;
							end

							15'o77656 : begin
								tr_o[23:16] <= wb_dat_i[7:0] ;
							end
						endcase
					
					
					if (wb_sel_i[1])
						case (adr_i)
							15'o77654 : dar_o[15:8] <= wb_dat_i[15:8] ;
							15'o77655 : tr_o[15:8] <= wb_dat_i[15:8] ;
							15'o77656 : tr_o[31:24] <= wb_dat_i[15:8] ;
						endcase
				end else begin
					case (adr_i)
						15'o77653 : wb_dat_o <= csr ;
						15'o77654 : wb_dat_o <= dar_i ;
						15'o77655 : wb_dat_o <= tr_i[15:0] ;
						15'o77656 : wb_dat_o <= tr_i[31:16] ;
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
	reg [7:0]	seconds_r,
					minutes_r,
					hours_r,
					days_r,
					months_r,
					years_r ;
					
	reg dec_ena = 1'b0 ;
	wire dec_ready ;
	wire [15:0] date_i ;
	wire [31:0] time_i ;
	
	toy_decoder decoder(
		.ena(dec_ena),
		
		.years_i(years_r),
		.months_i(months_r),
		.days_i(days_r),
		.hours_i(hours_r),
		.minutes_i(minutes_r),
		.seconds_i(seconds_r),
		
		.date_o(date_i),
		.time_o(time_i),
		.ready(dec_ready)
	) ;
	
	reg enc_ena = 1'b0 ;
	wire enc_ready ;
	wire [7:0]	seconds_o,
					minutes_o,
					hours_o,
					days_o,
					months_o,
					years_o ;
	
	toy_encoder encoder(
		.ena(enc_ena),
		
		.date_i(dar_o_ff),
		.time_i(tr_o_ff),
		
		.years_o(years_o),
		.months_o(months_o),
		.days_o(days_o),
		.hours_o(hours_o),
		.minutes_o(minutes_o),
		.seconds_o(seconds_o),
		
		.ready(enc_ready)
	) ;

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
							tr_o_ff <= tr_o ;
							dar_o_ff <= dar_o ;
							tr_i <= tr_o ;
							dar_i <= dar_o ;
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
					dec_ena <= 1'b1 ;
					if (dec_ready) begin
						dar_i <= date_i ;
						tr_i  <= time_i ;
						dec_ena <= 1'b0 ;
						i2cdone <= 1'b1 ;
						ts <= TS_IDLE ;
					end
				end
				
				TS_PREP_WRDATA : begin
					i2cdone <= 1'b0 ;
					enc_ena <= 1'b1 ;

					if (enc_ready) begin
						years_r <= years_o ;
						months_r <= months_o ;
						days_r <= days_o ;
						hours_r <= hours_o ;
						minutes_r <= minutes_o ;
						seconds_r <= seconds_o ;
						
						ts <= TS_SET_WRITE_REG ;
						enc_ena <= 1'b0 ;
					end
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
							4'd7 : data_o  <= seconds_r ; // seconds
							4'd6 : data_o	<= minutes_r ; // minutes
							4'd5 : data_o	<= hours_r ; // hours
							4'd4 : data_o  <= 8'd1  ; // weekdays unused
							4'd3 : data_o	<= days_r ; // days
							4'd2 : data_o	<= months_r ; // months
							4'd1 : begin
								data_o	<= years_r ; // years
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

module toy_decoder(
	input ena,
	
	input	[7:0]	years_i,
	input	[7:0]	months_i,
	input	[7:0]	days_i,
	input	[7:0]	hours_i,
	input	[7:0]	minutes_i,
	input	[7:0]	seconds_i,
	
	output reg [15:0] date_o,
	output reg [31:0]	time_o,
	
	output reg			ready
) ;
	
	initial ready = 1'b0 ;
	
	reg [15:0] years ;
	reg [7:0] days, months, seconds, minutes, hours, iter, age ;

	always @* begin
		if (ena) begin
			years = 16'd2000 + years_i[3:0] + years_i[7:4] * 4'd10 ;
			age = 8'd0 ;
			iter = 8'd0 ;
			while (years > 32'd2003 && iter < 8'd10) begin
				years = years - 6'd32 ;
				age = age + 1'd1 ;
				iter = iter + 1'd1 ;
			end
			years = years - 12'd1972 ;
			
			days = days_i[3:0] + days_i[6:4] * 4'd10 ;
			months = months_i[3:0] + months_i[4] * 4'd10 ;
			
			date_o <=  {age[1:0], 14'b0} | years[4:0]  | {days[4:0], 5'b0} | {months[3:0], 10'b0} ;
			
			seconds = seconds_i[3:0] + seconds_i[6:4]	* 4'd10 ;
			minutes = minutes_i[3:0] + minutes_i[6:4]	* 4'd10 ;
			hours	  = hours_i[3:0]	 + hours_i[5:4]	* 4'd10 ;
			
			time_o <= (seconds + minutes * 6'd60 + hours * 12'd3600) * 6'd60 ;
		
			ready <= 1'b1 ;
		end else
			ready <= 1'b0 ;
	end
endmodule

module toy_encoder(
	input ena,
	
	input [15:0] date_i,
	input [31:0] time_i,
	
	output reg [7:0]	years_o,
	output reg [7:0]	months_o,
	output reg [7:0]	days_o,
	output reg [7:0]	hours_o,
	output reg [7:0]	minutes_o,
	output reg [7:0]	seconds_o,
	
	output reg			ready
) ;

	initial ready = 1'b0 ;

	reg [31:0] total ;
	reg [15:0] years ;
	reg [7:0] days, months, seconds, minutes, hours ;

	always @* begin
		if (ena) begin
			// DATE : AAMMMMDD DDDYYYYY
			days = date_i[9:5] ;
			months = date_i[13:10] ;
			years = date_i[4:0] + {date_i[15:14], 5'b0} - 5'd28 ; // 2000 - 1972 = 28
			
			days_o[3:0] <= days % 4'd10 ;
			days_o[5:4] <= days / 4'd10 ;

			months_o[3:0] <= months % 4'd10 ;
			months_o[4]   <= months / 4'd10 ;

			years_o[3:0] <= years % 4'd10 ;
			years_o[7:4] <= years / 4'd10 ;
		
			total   = time_i / 6'd60 ;
			hours	  = total / 12'd3600 ;
			minutes = (total % 12'd3600) / 6'd60 ;
			seconds = total - hours * 12'd3600 - minutes * 6'd60 ;
		
			seconds_o[3:0] <= seconds % 4'd10 ;
			seconds_o[6:4] <= seconds / 4'd10 ;

			minutes_o[3:0] <= minutes % 4'd10 ;
			minutes_o[6:4] <= minutes / 4'd10 ;

			hours_o[3:0] <= hours % 4'd10 ;
			hours_o[5:4] <= hours / 4'd10 ;

			ready <= 1'b1 ;
		end else
			ready <= 1'b0 ;
	end

endmodule
