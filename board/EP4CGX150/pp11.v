module pp11(
	input							clk_p,
	input							sys_init,

	// BUS
	input				[21:0]	wb_adr_i,
	input				[15:0]	wb_dat_i,
	output	reg	[15:0]	wb_dat_o,
	input							bus_stb,
	input							wb_we_i,
	input				[1:0]		wb_sel_i,
	output	reg				pp11_ack,
	
   // IRQ
	output reg             irq,         // запрос
   input                  iack,        // подтверждение

// интерфейс SD-карты
   output                 sdcard_cs, 
   output                 sdcard_mosi, 
   output                 sdcard_sclk, 
   input                  sdcard_miso, 
   output reg             sdreq,      // запрос доступа к карте
   input                  sdack,      // подтверждение доступа к карте
   
// тактирование SD-карты
   input                  sdclock,   

// Адрес начала банка на карте
   input [26:0]           start_offset
) ;

	
	//***********************************************
	//*  Контроллер SD-карты
	//***********************************************
	wire [26:0] sdaddr ;       	  // адрес сектора карты
	reg  [26:0] sdcard_addr ;       // адрес сектора карты
	wire sdcard_idle ;              // признак готовности контроллера
	wire sdcard_error ;             // флаг ошибки
	reg sdspi_start ;               // строб запуска sdspi
	wire sdspi_io_done ;            // флаг заверщение операции обмена с картой
	reg [7:0] sdbuf_addr ;          // адрес в буфере чтния/записи
	wire [15:0] sdbuf_dataout ;     // слово; читаемое из буфера чтения
	reg [15:0] sdbuf_datain ;
	reg  sdbuf_we = 1'b0 ;
	reg  sdspi_we = 1'b0 ;

	reg [7:0] sector = 8'b0 ;

	assign sdaddr = start_offset + sector ;

	sdspi sdcard (
		// интерфейс к карте
		.sdcard_cs(sdcard_cs), 
		.sdcard_mosi(sdcard_mosi), 
		.sdcard_miso(sdcard_miso),
		.sdcard_sclk(sdcard_sclk),

		.sdcard_addr(sdcard_addr),               // адрес блока на карте
		.sdcard_idle(sdcard_idle),               // сигнал готовности модуля к обмену
		.sdcard_error(sdcard_error),             // флаг ошибки

		// сигналы управления чтением - записью
		.sdspi_start(sdspi_start),                // строб запуска ввода вывода
		.sdspi_io_done(sdspi_io_done),            // флаг окончания обмена данными
		.sdspi_write_mode(sdspi_we), 					// режим: 0 - чтение, 1 - запись

		// интерфейс к буферной памяти контроллера
		.sdbuf_addr(sdbuf_addr),                 // текущий адрес в буферах чтения и записи
		.sdbuf_dataout(sdbuf_dataout),           // слово, читаемое из буфера чтения
		.sdbuf_datain(sdbuf_datain),       		  // слово, записываемое в буфер записи
		.sdbuf_we(sdbuf_we),                  	  // строб записи буфера

		.mode(1'b0),                             // режим ведущего-ведомого контроллера
		.controller_clk(clk_p),                  // синхросигнал общей шины
		.reset(sys_init),                        // сброс
		.sdclk(sdclock)                          // синхросигнал SD-карты
	) ; 

	wire [14:0] adr_i = wb_adr_i[15:1] ;
	wire pp11_stb = bus_stb & ((adr_i == 15'o77666) || (adr_i == 15'o77667)) ; // 177554..177556
	
	reg [15:0] csr = 16'b0 ; // command-status 177554
	reg [15:0] dbr = 16'b0 ; // data 177556
	
	localparam IS_IDLE = 2'd0 ;    // ожидание прерывания
	localparam IS_REQ  = 2'd1 ;     // запрос векторного прерывания
	localparam IS_WAIT = 2'd2 ;    // ожидание обработки прерывания со стороны процессора

	reg[1:0] interrupt_state = IS_IDLE ;
	reg interrupt_trigger = 1'b0 ;
	wire ie = csr[6] ; // interrupt enabled

	localparam SD_IDLE	= 3'd0 ;
	localparam SD_W01		= 3'd1 ;
	localparam SD_W02		= 3'd2 ;
	localparam SD_W03		= 3'd3 ;
	localparam SD_W04		= 3'd4 ;
	localparam SD_W05		= 3'd5 ;
	localparam SD_W06		= 3'd6 ;
	localparam SD_W07		= 3'd7 ;

	reg [2:0] sd_state = SD_IDLE ;

	always @(posedge clk_p) begin
		if (sys_init) begin
			csr <= 16'o200 ;
			dbr <= 16'b0 ;
			irq <= 1'b0 ;
			pp11_ack	<= 1'b0 ;
			sdspi_start <= 1'b0 ;
			sdspi_we <= 1'b0 ;
			sdbuf_we <= 1'b0 ;
			sdreq <= 1'b0 ;
			sdbuf_addr <= 8'b0 ;
			sector <= 8'b0 ;
			interrupt_trigger <= 1'b0 ;
			interrupt_state <= IS_IDLE ;
			sd_state <= SD_IDLE ;
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
			
			if (pp11_stb) begin
				if (wb_we_i) begin
					if (wb_sel_i[0]) begin
						case (adr_i)
							15'o77666 : begin
								csr[6] <= wb_dat_i[6] ;
								if (wb_dat_i[6] & csr[7])
									interrupt_trigger <= 1'b1 ;
							end
							
							15'o77667 : begin
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
						15'o77666 : wb_dat_o <= csr ;
						15'o77667 : wb_dat_o <= dbr ;
					endcase
				end
				
				pp11_ack <= 1'b1 ;
			end else begin
				pp11_ack <= 1'b0 ;
				
				case (sd_state)
					SD_IDLE : if (~csr[7]) begin
						sdbuf_datain <= {8'o377, dbr[7:0]} ;
						sdbuf_we <= 1'b1 ;
						sd_state <= SD_W01 ;
					end
					
					SD_W01 : sd_state <= SD_W02 ;

					SD_W02 : begin
						sdbuf_we <= 1'b0 ;
						sdbuf_addr <= sdbuf_addr + 1'b1 ;
						
						if (sdbuf_addr == 8'd255) begin
							sdreq <= 1'b1 ;
							sd_state <= SD_W03 ;
						end else begin
							csr[7] <= 1'b1 ;
								if (ie)
									interrupt_trigger <= 1'b1 ;
							sd_state <= SD_IDLE ;
						end
					end
					
					SD_W03 : if (sdack & sdcard_idle) begin
						sdspi_we <= 1'b1 ;
						sdcard_addr <= sdaddr ;
						sdspi_start <= 1'b1 ;
						sd_state <= SD_W04 ;
					end
					
					SD_W04 : if (sdspi_io_done) begin
						sdspi_start <= 1'b0 ;
						sector <= sector + 1'b1 ;
						sdbuf_datain <= 16'b0 ;
						sdbuf_we <= 1'b1 ;
						sd_state <= SD_W05 ;
					end
					
					SD_W05 : begin
						sdbuf_addr <= sdbuf_addr + 1'b1 ;
						sd_state <= SD_W06 ;
					end
					
					SD_W06 : begin
						if (sdbuf_addr == 8'd255) begin
							sdbuf_we <= 1'b0 ;
							sdbuf_addr <= 8'd0 ;
							sdcard_addr <= sdaddr ;
							sdspi_start <= 1'b1 ;
							sd_state <= SD_W07 ;
						end else
							sd_state <= SD_W05 ;
					end
					
					SD_W07 : if (sdspi_io_done) begin
						sdspi_start <= 1'b0 ;
						sdspi_we <= 1'b0 ;
						sdreq <= 1'b0 ;
						csr[7] <= 1'b1 ;
							if (ie)
								interrupt_trigger <= 1'b1 ;
						sd_state <= SD_IDLE ;
					end
					
				endcase
			end

			csr[15] <= sdcard_error ;
		end
	end
	
endmodule
