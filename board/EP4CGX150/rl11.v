/*
	128	words per sector
	 40	sectors per surface
	  2	surfaces per cylinder
	512	cylinders per drive (RL02)
	
	===   10_485_760 bytes per drive, 4096 sectors per drive
*/

module rl11(
	input							clk_p,
	input							sys_init,

	// BUS
	input				[21:0]	wb_adr_i,
	input				[15:0]	wb_dat_i,
	output	reg	[15:0]	wb_dat_o,
	input							bus_stb,
	input							wb_we_i,
	input				[1:0]		wb_sel_i,
	output	reg				rl11_ack,

	// IRQ
	output	reg           irq,         // запрос
   input                  iack,        // подтверждение

// DMA
   output	reg           dma_req,    // запрос DMA
   input                  dma_gnt,    // подтверждение DMA
   output 	reg	[17:0]  dma_adr_o,  // выходной адрес при DMA-обмене
   input				[15:0]  dma_dat_i,  // входная шина данных DMA
   output	reg	[15:0]  dma_dat_o,  // выходная шина данных DMA
   output	reg           dma_stb_o,  // строб цикла шины DMA
   output	reg           dma_we_o,   // направление передачи DMA (0 - память->диск, 1 - диск->память) 
   input                  dma_ack_i,  // Ответ от устройства, с которым идет DMA-обмен
	
// интерфейс SD-карты
   output                 sdcard_cs, 
   output                 sdcard_mosi, 
   output                 sdcard_sclk, 
   input                  sdcard_miso, 
   output	reg           sdreq,      // запрос доступа к карте
   input                  sdack,      // подтверждение доступа к карте
   
// тактирование SD-карты
   input                  sdclock,   

// Адрес начала банка на карте
   input				[26:0]  start_offset
) ;

	wire	[26:0] 	sdaddr ;						// адрес сектора карты
	reg 	[26:0] 	sdcard_addr ;				// адрес сектора карты
	wire				sdcard_idle ;           // признак готовности контроллера
	wire				sdcard_error ;          // флаг ошибки
	reg				sdspi_start = 1'b0 ;		// строб запуска sdspi
	wire				sdspi_io_done ;			// флаг заверщение операции обмена с картой
	reg	[7:0]		sdbuf_addr ;				// адрес в буфере чтния/записи
	wire	[15:0] 	sdbuf_dataout ;			// слово; читаемое из буфера чтения
	reg	[15:0] 	sdbuf_datain = 16'b0 ;
	reg				sdbuf_we = 1'b0 ;
	reg				sdspi_we = 1'b0 ;

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
		.sdspi_write_mode(sdspi_we),    				// режим: 0 - чтение, 1 - запись

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

	localparam CSR_ADR = 15'o76200 ;
	localparam BAR_ADR = 15'o76201 ;
	localparam DAR_ADR = 15'o76202 ;
	localparam MPR_ADR = 15'o76203 ;
	localparam BAE_ADR = 15'o76204 ;

	reg [15:0]	csr, // 174400 Control Status (CS)
					bar, // 174402 Bus Address (BA)
					dar, // 174404 Disk Address (DA)
					mpr, // 174406 Multipurpose (MP)
					bae ;// 174410 Bus Address Extension (BAE)

	wire [14:0] adr_i = wb_adr_i[15:1] ;
	wire rl11_stb = bus_stb & ((adr_i == CSR_ADR) || (adr_i == BAR_ADR) || (adr_i == DAR_ADR) || (adr_i == MPR_ADR) || (adr_i == BAE_ADR)) ;

	localparam IS_IDLE = 2'd0 ;	// ожидание прерывания
	localparam IS_REQ  = 2'd1 ;   // запрос векторного прерывания
	localparam IS_WAIT = 2'd2 ;	// ожидание обработки прерывания со стороны процессора

	wire			ie						= csr[6] ; // interrupt enabled
	reg	[1:0]	interrupt_state	= IS_IDLE ;
	reg			interrupt_trigger	= 1'b0 ;

	wire			CS_CRDY	= csr[7] ;
	wire  [2:0] RL_FUNC	= csr[3:1] ;
	wire [12:0] RLWC		= mpr[12:0] ;
	
	wire  [1:0]	dn = csr[9:8] ; // drive number
	wire  [8:0] cyl = dar[15:7] ;
	wire			head = dar[6] ;
	wire  [5:0] sector = dar[5:0] ;

	localparam RL_FUNC_NOP				= 3'b000 ;
	localparam RL_FUNC_WRITE_CHECK	= 3'b001 ;
	localparam RL_FUNC_GET_STATUS		= 3'b010 ;
	localparam RL_FUNC_SEEK				= 3'b011 ;
	localparam RL_FUNC_READ_HEADER	= 3'b100 ;
	localparam RL_FUNC_WRITE_DATA		= 3'b101 ;
	localparam RL_FUNC_READ_DATA		= 3'b110 ;
	localparam RL_FUNC_READ_DATA_WH	= 3'b111 ;
	
	localparam RS_IDLE	= 4'd0 ;
	localparam RS_R01		= 4'd1 ;
	localparam RS_R02		= 4'd2 ;
	localparam RS_R03		= 4'd3 ;
	localparam RS_R04		= 4'd4 ;
	localparam RS_R05		= 4'd5 ;
	localparam RS_R06		= 4'd6 ;
	localparam RS_R07		= 4'd7 ;
	localparam RS_W01		= 4'd8 ;
	localparam RS_W02		= 4'd9 ;
	localparam RS_W03		= 4'd10 ;
	localparam RS_W04		= 4'd11 ;
	localparam RS_W05		= 4'd12 ;
	localparam RS_W06		= 4'd13 ;

	reg [3:0] rl_state = RS_IDLE ;

	always @(posedge clk_p) begin
		if (sys_init) begin
			csr <= 16'o201 ;
			dar <= 16'b0 ;
			bae <= 16'b0 ;
			irq <= 1'b0 ;
			rl11_ack	<= 1'b0 ;
			rl_state <= RS_IDLE ;
			dma_req <= 1'b0 ;
			dma_we_o <= 1'b0 ;
			sdreq <= 1'b0 ;
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
			
			if (rl11_stb) begin
				if (wb_we_i) begin
					case (adr_i)
						CSR_ADR : begin
							if (wb_sel_i[0])
								csr[7:1] <= wb_dat_i[7:1] ;
							if (wb_sel_i[1])
								csr[9:8] <= wb_dat_i[9:8] ;
						end
						
						BAR_ADR : begin
							if (wb_sel_i[0])
								bar[7:0] <= {wb_dat_i[7:1], 1'b0} ;
							if (wb_sel_i[1])
								bar[15:8] <= wb_dat_i[15:8] ;
						end
						
						DAR_ADR : begin
							if (wb_sel_i[0])
								dar[7:0] <= wb_dat_i[7:0] ;
							if (wb_sel_i[1])
								dar[15:8] <= wb_dat_i[15:8] ;
						end
						
						MPR_ADR : begin
							if (wb_sel_i[0])
								mpr[7:0] <= wb_dat_i[7:0] ;
							if (wb_sel_i[1])
								mpr[15:8] <= wb_dat_i[15:8] ;
						end

						BAE_ADR : begin
							if (wb_sel_i[0])
								bae[5:0] <= wb_dat_i[5:0] ;
						end
					endcase
				end else begin
					case (adr_i)
						CSR_ADR : wb_dat_o <= csr ;
						BAR_ADR : wb_dat_o <= bar ;
						DAR_ADR : wb_dat_o <= dar ;
						MPR_ADR : wb_dat_o <= mpr ;
						BAE_ADR : wb_dat_o <= bae ;
					endcase
				end

				rl11_ack <= 1'b1 ;
			end else begin
				rl11_ack <= 1'b0 ;
				
				if (~CS_CRDY) begin
					case (RL_FUNC)
						RL_FUNC_NOP,
						RL_FUNC_WRITE_CHECK : begin
							csr[7] <= 1'b1 ;
							csr[15:10] <= 6'b0 ;
							if (ie)
								interrupt_trigger <= 1'b1 ;
						end
						
						RL_FUNC_GET_STATUS : begin
							csr[15:10] <= 6'b0 ;
							
							if (dar[1:0] == 2'b11) begin
								mpr <= 16'o235 ; // LockOn, BrushHome, HeadsOn, DriveType RL02
							end
								else mpr <= 16'b0 ;
							
							csr[7] <= 1'b1 ;
							if (ie)
								interrupt_trigger <= 1'b1 ;
						end
						
						RL_FUNC_SEEK : begin
//							if (dar[6:5] == 2'b00 && dar[1:0] == 2'b01) begin
//								if (dar[2]) begin // forward
//									
//								end else begin    // backward
//								end
//							end
							csr[0] <= 1'b1 ;
							csr[7] <= 1'b1 ;
							if (ie)
								interrupt_trigger <= 1'b1 ;
						end
						
						RL_FUNC_READ_HEADER : begin
							mpr <= 16'b0 ;
							csr[7] <= 1'b1 ;
							if (ie)
								interrupt_trigger <= 1'b1 ;
						end
					endcase
				end
				
				case (rl_state)
					RS_IDLE : begin
						if (~CS_CRDY && (RL_FUNC == RL_FUNC_READ_DATA || RL_FUNC == RL_FUNC_READ_DATA_WH)) begin
							csr[15:10] <= 6'b0 ;
							rl_state <= RS_R01 ;
						end else if (~CS_CRDY && RL_FUNC == RL_FUNC_WRITE_DATA) begin
							csr[15:10] <= 6'b0 ;
							rl_state <= RS_W01 ;
						end
					end
					
					// ----------------------- READ ---------------------------------
					
					RS_R01 : begin
						if (sector > 6'd39) begin
							{csr[15], csr[10], csr[7]} <= 3'b111 ; // Err, OPI, Ready
							if (ie)
								interrupt_trigger <= 1'b1 ;
							rl_state <= RS_IDLE ;
						end else begin
							sdreq <= 1'b1 ;
							rl_state <= RS_R02 ;
						end
					end
					
					RS_R02 : if (sdack & sdcard_idle) begin
						sdcard_addr <= sdaddr ;
						sdbuf_addr <= 8'b0 ;
						sdspi_we <= 1'b0 ;
						sdspi_start <= 1'b1 ;
						rl_state <= RS_R03 ;
					end
					
					RS_R03 : if (sdspi_io_done) begin
						sdspi_start <= 1'b0 ;
						sdreq <= 1'b0 ;
						dar[5:0] <= dar[5:0] + 1'b1 ;
						sdbuf_we <= 1'b0 ;
						dma_req <= 1'b1 ;
						rl_state <= RS_R04 ;
					end
					
					RS_R04 : if (dma_gnt & ~dma_ack_i) begin
						rl_state <= RS_R05 ;
					end
					
					RS_R05 : begin
						if (RLWC == 13'b0) begin
							dma_req <= 1'b0 ;
							csr[7] <= 1'b1 ;
							if (ie)
								interrupt_trigger <= 1'b1 ;
							rl_state <= RS_IDLE ;
						end else begin
							dma_adr_o <= {csr[5:4], bar} ;
							dma_dat_o <= sdbuf_dataout ;
							dma_we_o <= 1'b1 ;
							dma_stb_o <= 1'b1 ;
							rl_state <= RS_R06 ;
						end
					end
					
					RS_R06 : if (dma_ack_i) begin
						dma_stb_o <= 1'b0 ;
						rl_state <= RS_R07 ;
					end
					
					RS_R07 : if (~dma_ack_i) begin
						dma_we_o <= 1'b0 ;
						mpr <= mpr + 1'b1 ;
						bar <= bar + 2'd2 ;
						sdbuf_addr <= sdbuf_addr + 1'b1 ;
						rl_state <= RS_R04 ;
						
						if (sdbuf_addr == 8'd127 && RLWC != 13'o17777) begin
							dma_req <= 1'b0 ;
							rl_state <= RS_R01 ;
						end
					end
					
					// ----------------------- WRITE ---------------------------------
					
					RS_W01 : begin
						if (sector > 6'd39) begin
							{csr[15], csr[10], csr[7]} <= 3'b111 ; // Err, OPI, Ready
							if (ie)
								interrupt_trigger <= 1'b1 ;
							rl_state <= RS_IDLE ;
						end else begin
							dma_req <= 1'b1 ;
							sdbuf_addr <= 8'b0 ;
							rl_state <= RS_W02 ;
						end
					end
					
					RS_W02 : if (dma_gnt & ~dma_ack_i) begin
						if (sdbuf_addr < 8'd128) begin
							dma_adr_o <= {csr[5:4], bar} ;
							dma_we_o <= 1'b0 ;
							dma_stb_o <= 1'b1 ;
						end
						rl_state <= RS_W03 ;
					end
					
					RS_W03 : if (dma_ack_i | sdbuf_addr > 8'd127) begin
						sdbuf_datain <= sdbuf_addr < 8'd128 ? dma_dat_i : 16'b0 ;
						sdbuf_we <= 1'b1 ;
						dma_stb_o <= 1'b0 ;
						rl_state <= RS_W04 ;
					end
					
					RS_W04 : if (~dma_ack_i) begin
						sdbuf_we <= 1'b0 ;
						if (sdbuf_addr < 8'd128 && RLWC != 13'b0) begin
							mpr <= mpr + 1'b1 ;
							bar <= bar + 2'd2 ;
						end
						sdbuf_addr <= sdbuf_addr + 1'b1 ;
						
						rl_state <= RS_W02 ;
						
						if (sdbuf_addr == 8'd255) begin
							dma_req <= 1'b0 ;
							sdreq <= 1'b1 ;
							rl_state <= RS_W05 ;
						end
						
					end
					
					RS_W05 : if (sdack & sdcard_idle) begin
						sdcard_addr <= sdaddr ;
						sdspi_we <= 1'b1 ;
						sdspi_start <= 1'b1 ;
						rl_state <= RS_W06 ;
					end
					
					RS_W06 : if (sdspi_io_done) begin
						dar[5:0] <= dar[5:0] + 1'b1 ;
						sdspi_start <= 1'b0 ;
						sdreq <= 1'b0 ;
						
						if (RLWC == 13'b0) begin
							csr[7] <= 1'b1 ;
							if (ie)
								interrupt_trigger <= 1'b1 ;
							rl_state <= RS_IDLE ;
						end else begin
							rl_state <= RS_W01 ;
						end
					end
				endcase
			end
			
			csr[14] <= sdcard_error ;
//			csr[13:10] <= rl_state ;
		end
	end
	
/*
	 40	sectors per surface
	  2	surfaces per cylinder
	512	cylinders per drive (RL02)
	
	===   40960 sectors per drive,
	      4 drives per controller
	
*/
	
   wire [16:0]	dn_offset = {dn, 15'd0} + {dn, 13'b0} ; // dn * 40960 = dn << 15 + dn << 13 ;
	wire [17:0]	cyl_offset = {cyl, 6'b0} + {cyl, 4'b0}; // cyl * 80 = cyl << 6 + cyl << 4
	wire  [5:0]	head_offset = {head, 5'b0} + {head, 3'b0} ; // head * 40 = head << 5 + head << 3

	assign sdaddr = start_offset + dn_offset + cyl_offset + head_offset + sector ;
	
endmodule
