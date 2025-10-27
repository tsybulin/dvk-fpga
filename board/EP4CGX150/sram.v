module sram(
   input          	clk_p,        // основной синхросигнал, прямая фаза
   input         		sdram_reset,  // сброс/переинициализация SDRAM
   input         		sdram_stb,    // строб транзакции
   input         		sdram_we,     // разрешение записи
   input		[1:0]		sdram_sel,    // выбор байтов
	input 	[21:1]	sdram_adr,    // шина адреса
   input 	[15:0]	sdram_out,    // шина данных хост -> память
	output				sdram_ack,    // подтверждение транзакции
	output	[15:0]	sdram_dat,    // шина данных память -> хост  
   output          	sdram_ready   // готовность SDRAM
) ;

	wire sdram_wr = sdram_we & sdram_stb ;
	wire sdram_rd = (~sdram_we) & sdram_stb ;
	assign sdram_ready = 1'b1 ;  // модуль всегда готов

	wire ramsel0 = sdram_adr[21:17] == 5'b00000 ;
	wire ramsel1 = sdram_adr[21:17] == 5'b00001 ;
	wire ramsel2 = sdram_adr[21:17] == 5'b00010 ;
	wire ramsel3 = sdram_adr[21:17] == 5'b00011 ;
	wire ramsel4 = sdram_adr[21:17] == 5'b00100 ;
	
	wire [15:0] sdram_dat_0 ;
	wire [15:0] sdram_dat_1 ;
	wire [15:0] sdram_dat_2 ;
	wire [15:0] sdram_dat_3 ;
	wire [15:0] sdram_dat_4 ;

	assign sdram_dat 	= (ramsel0 ? sdram_dat_0 : 16'd0)
							| (ramsel1 ? sdram_dat_1 : 16'd0)
							| (ramsel2 ? sdram_dat_2 : 16'd0)
							| (ramsel3 ? sdram_dat_3 : 16'd0)
							| (ramsel4 ? sdram_dat_4 : 16'd0)
							;

	// Модуль altsyncram размером 64К
	baseram ram0 (
		.address(sdram_adr[16:1]),
		.byteena(sdram_sel),
		.clock(clk_p),
		.data(sdram_out),
		.rden(sdram_rd && ramsel0),
		.wren(sdram_wr && ramsel0),
		.q(sdram_dat_0)
   ) ;

	baseram ram1 (
		.address(sdram_adr[16:1]),
		.byteena(sdram_sel),
		.clock(clk_p),
		.data(sdram_out),
		.rden(sdram_rd && ramsel1),
		.wren(sdram_wr && ramsel1),
		.q(sdram_dat_1)
   ) ;

	baseram ram2 (
		.address(sdram_adr[16:1]),
		.byteena(sdram_sel),
		.clock(clk_p),
		.data(sdram_out),
		.rden(sdram_rd && ramsel2),
		.wren(sdram_wr && ramsel2),
		.q(sdram_dat_2)
   ) ;

	baseram ram3 (
		.address(sdram_adr[16:1]),
		.byteena(sdram_sel),
		.clock(clk_p),
		.data(sdram_out),
		.rden(sdram_rd && ramsel3),
		.wren(sdram_wr && ramsel3),
		.q(sdram_dat_3)
   ) ;

	baseram ram4 (
		.address(sdram_adr[16:1]),
		.byteena(sdram_sel),
		.clock(clk_p),
		.data(sdram_out),
		.rden(sdram_rd && ramsel4),
		.wren(sdram_wr && ramsel4),
		.q(sdram_dat_4)
   ) ;

	// формирователь сигнала подверждения транзакции
	reg [1:0]dack ;
	assign sdram_ack = sdram_stb & (dack[1]) ;
	
	always @ (posedge clk_p)  begin
		dack[0] <= sdram_stb;
		dack[1] <= sdram_stb & dack[0];
	end


endmodule
