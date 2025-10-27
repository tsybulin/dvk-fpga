module pr11(
	input							clk_p,
	input							sys_init,

	// BUS
	input				[21:0]	wb_adr_i,
	input				[15:0]	wb_dat_i,
	output	reg	[15:0]	wb_dat_o,
	input							bus_stb,
	input							wb_we_i,
	input				[1:0]		wb_sel_i,
	output	reg				pr11_ack,
	
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
	reg eot = 1'b0 ;					  // end of tape
	
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
		.sdspi_write_mode(1'b0),      				// режим: 0 - чтение, 1 - запись

		// интерфейс к буферной памяти контроллера
		.sdbuf_addr(sdbuf_addr),                 // текущий адрес в буферах чтения и записи
		.sdbuf_dataout(sdbuf_dataout),           // слово, читаемое из буфера чтения
		.sdbuf_datain(16'b0),             		  // слово, записываемое в буфер записи
		.sdbuf_we(1'b0),                     	  // строб записи буфера

		.mode(1'b0),                             // режим ведущего-ведомого контроллера
		.controller_clk(clk_p),                  // синхросигнал общей шины
		.reset(sys_init),                        // сброс
		.sdclk(sdclock)                          // синхросигнал SD-карты
	) ; 

	wire [14:0] adr_i = wb_adr_i[15:1] ;
	wire pr11_stb = bus_stb & ((adr_i == 15'o77664) || (adr_i == 15'o77665)) ; // 177550, 177552
	
	reg [15:0] csr = 16'b0 ; // command-status 177550
	reg [15:0] dbr = 16'b0 ;   // data 177552
	
	reg sd_go = 1'b0 ;
	reg sd_done = 1'b0 ;
	reg [7:0] sd_byte ;
	
	localparam IS_IDLE = 2'd0 ;    // ожидание прерывания
	localparam IS_REQ  = 2'd1 ;     // запрос векторного прерывания
	localparam IS_WAIT = 2'd2 ;    // ожидание обработки прерывания со стороны процессора

	reg[1:0] interrupt_state = IS_IDLE ;
	reg interrupt_trigger = 1'b0 ;
	wire ie = csr[6] ; // interrupt enabled
	
	localparam PS_IDLE	= 3'd0 ;
	localparam PS_GO		= 3'd1 ;
	localparam PS_WAIT	= 3'd2 ;
	
	reg [2:0] pr_state = PS_IDLE ;
	
	
	always @(posedge clk_p) begin
		if (sys_init) begin
			csr <= 16'b0 ;
			dbr <= 16'b0 ;
			irq <= 1'b0 ;
			pr11_ack	<= 1'b0 ;
			sd_go <= 1'b0 ;
			interrupt_trigger <= 1'b0 ;
			pr_state <= PS_IDLE ;
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
		
			if (pr11_stb) begin
				if (wb_we_i) begin
					if (wb_sel_i[0])
						case (adr_i)
							15'o77664 : begin
								csr[0] <= wb_dat_i[0] ; //only bits 6,0 is write-able
								csr[6] <= wb_dat_i[6] ;
								
								if (wb_dat_i[0]) begin
									if (csr[15]) begin  // Operation of this bit is disabled if Error = 1; attempting to set it when Error = 1 will cause an immediate interrupt if Interrupt Enable = 1. 
										csr[0] <= 1'b0 ;
										if (ie | wb_dat_i[6])
											interrupt_trigger <= 1'b1 ;
									end else begin  // if GO - clear buffer; clear done and set busy
										dbr <= 16'b0 ;
										csr[7] <= 1'b0 ;
										csr[11] <= 1'b1 ;
									end
								end
							end
						endcase
				end else begin
					case (adr_i)
						15'o77664 : wb_dat_o <= csr ;
						
						15'o77665 : begin
							wb_dat_o <= dbr ;
							csr[7]	<= 1'b0 ; // clear done
						end
					endcase
				end
				pr11_ack <= 1'b1 ;
			end else begin
				pr11_ack	<= 1'b0 ;
				
				case (pr_state)
					PS_IDLE : begin
						if (csr[0]) begin
							sd_go <= 1'b1 ;
							csr[0] <= 1'b0 ;
							pr_state <= PS_GO ;
						end
					end
					
					PS_GO : begin
						if (sd_done) begin
							sd_go <= 1'b0 ;
							csr[7] <= 1'b1 ;
							if (ie)
								interrupt_trigger <= 1'b1 ;
							csr[11] <= 1'b0 ;
							dbr[7:0] <= sd_byte ;
							pr_state <= PS_WAIT ;
						end
					end
					
					PS_WAIT : begin
						if (~sd_done)
							pr_state <= PS_IDLE ;
					end
				endcase
			end
			
			csr[15] <= sdcard_error | eot ;
		end
	end
	
	localparam SD_IDLE	= 3'd0 ;
	localparam SD_GO		= 3'd1 ;
	localparam SD_WAIT	= 3'd2 ;
	localparam SD_REQ		= 3'd3 ;
	localparam SD_ACK		= 3'd4 ;
	localparam SD_READ	= 3'd5 ;
	localparam SD_FREE	= 3'd6 ;
	
	reg [2:0] sd_state = SD_IDLE ;
	
	reg [7:0] sector = 8'b0 ;
	reg sector_loaded = 1'b0 ;

	always @(posedge clk_p) begin
		if (sys_init) begin
			sd_done <= 1'b0 ;
			eot <= 1'b0 ;
			sdreq <= 1'b0 ;
			sdbuf_addr <= 8'b0 ;
			sector <= 8'b0 ;
			sector_loaded <= 1'b0 ;
			sd_state <= SD_IDLE ;
		end else
			case (sd_state)
				SD_IDLE :
					if (sd_go) begin
						sd_state <= SD_GO ;
					end
					
				SD_GO : begin
					if (sector_loaded) begin
						sd_byte <= sdbuf_dataout[7:0] ;
						eot <= sdbuf_dataout[15:8] != 8'o377 ;
						sdbuf_addr <= sdbuf_addr + 1'b1 ;
						if (sdbuf_addr == 8'd255) begin
							sector_loaded <= 1'b0 ;
						end
						sd_done <= 1'b1 ;
						sd_state <= SD_WAIT ;
					end else begin
						sd_state <= SD_REQ ;
					end
				end
				
				SD_WAIT :
					if (~sd_go) begin
						sd_done <= 1'b0 ;
						sd_state <= SD_IDLE ;
					end
				
				SD_REQ : begin
					sdreq <= 1'b1 ;
					sd_state <= SD_ACK ;
				end
				
				SD_ACK : begin
					if (sdack) begin
						sdcard_addr <= sdaddr ;
						if (sdcard_idle) begin
							sdspi_start <= 1'b1 ;
							sd_state <= SD_READ ;
						end
					end
				end
				
				SD_READ : begin
					if (sdspi_io_done) begin
						sdspi_start <= 1'b0 ;
						sector_loaded <= 1'b1 ;
						sdbuf_addr <= 8'b0 ;
						sector <= sector + 1'b1 ;
						sd_state <= SD_FREE ;
					end
				end
					
				SD_FREE : begin
					sdreq <= 1'b0 ;
					if (~sdack)
						sd_state <= SD_GO ;
				end
				
			endcase
	end
	
	assign sdaddr = start_offset + sector ;
endmodule
