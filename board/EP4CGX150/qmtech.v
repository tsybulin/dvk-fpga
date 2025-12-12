//  Проект DVK-FPGA
//
//  Интерфейсный модуль для платы QMTECH Cyclone V SoC
//=================================================================
//

`include "config.v"

module qmtech(
	input					clk50,
	input					rst_n,
	input 		[1:0] button,
	input			[2:0]	sw,			// переключатели конфигурации
	output				led2812,		//	индикаторы состояния

   // Интерфейс SDRAM
   inout  [15:0]  DRAM_DQ,      //   SDRAM Data bus 16 Bits
   output [12:0]  DRAM_ADDR,    //   SDRAM Address bus 12 Bits
   output         DRAM_LDQM,    //   SDRAM Low-byte Data Mask 
   output         DRAM_UDQM,    //   SDRAM High-byte Data Mask
   output         DRAM_WE_N,    //   SDRAM Write Enable
   output         DRAM_CAS_N,   //   SDRAM Column Address Strobe
   output         DRAM_RAS_N,   //   SDRAM Row Address Strobe
   output         DRAM_CS_N,    //   SDRAM Chip Select
   output         DRAM_BA_0,    //   SDRAM Bank Address 0
   output         DRAM_BA_1,    //   SDRAM Bank Address 0
   output         DRAM_CLK,     //   SDRAM Clock
   output         DRAM_CKE,     //   SDRAM Clock Enable

   // интерфейс SD-карты
   output				sdcard_cs, 
   output				sdcard_mosi, 
   output				sdcard_sclk, 
   input					sdcard_miso, 

   // дополнительный UART 
   output				irps_txd,
   input					irps_rxd,

	// console panel
	output 	[2:0]		cons_row,
	inout		[11:0]	cons_col,
	output	[5:0]		cons_ledrow,
	
	// toy
	inout					i2c_SDA,		//SDA line
	inout					i2c_SCL		//SCL line
) ;

	//********************************************
	//* Светодиоды
	//********************************************
	wire disk_led, timer_led, led2, led1, led3 ;

//	assign led[0] = disk_led ;    // запрос обмена диска 
//	assign led[1] = ~led1 ;      // Индикатор состояния процессора 1
//	assign led[2] = ~led2 ;      // Индикатор состояния процессора 2
//	assign led[3] = ~led3 ;      // Индикатор состояния процессора 3
//	assign led[4] = ~timer_led ; // индикация включения таймера

	//************************************************
	//* тактовый генератор 
	//************************************************
	wire clk_p, clk_n, sdclock, clkrdy, clkcons ;

	pll pll1 (
		.inclk0(clk50),
		.areset(~rst_n),
		.c0(clk_p),     // основная тактовая частота, прямая фаза
		.c1(sdclock),   // тактовый сигнал SD-карты
		.c2(clkcons),   // console 1MHz
		.c3(clk_n),		 // // основная тактовая частота, инверсная фаза
		.locked(clkrdy) // флаг готовности PLL
	) ;

	ws2812 leds(
		.clk(clk_p),
		.led({disk_led, led3, led1, timer_led, led2}),
		.led2812(led2812)
	) ;

	reg [1:0] slow_filter ;

	always @ (posedge clk_p)  begin
		slow_filter[0] <= ~sw[2] ;
		slow_filter[1] <= slow_filter[0] & ~sw[2] ;
	end

	wire sdram_reset;
	wire sdram_we;
	wire sdram_stb;
	wire [1:0] sdram_sel;
	wire sdram_ack, sram_ack ;
	wire [21:1] sdram_adr;
	wire [15:0] sdram_out;
	
	wire [15:0] sdram_dat, sram_dat, dram_dat ;
	
	wire sdram_ready;
	wire sdram_wr;
	wire sdram_rd;

	reg [1:0] dreset;
	reg [1:0] dr_cnt;
	reg drs;
	
	// формирователь сброса
	always @(posedge clk_p) begin
		dreset[0] <= sdram_reset ; // 1 - сброс
		dreset[1] <= dreset[0] ;
		if (dreset[1] == 1) begin
		  // системный сброс активен
			drs <= 0 ;         // активируем сброс DRAM
			dr_cnt <= 2'b0 ;   // запускаем счетчик задержки
		end else 
		  // системный сброс снят
		  if (dr_cnt != 2'd3)
				dr_cnt <= dr_cnt + 1'b1 ; // счетчик задержки ++
		  else
				drs <= 1'b1 ;                          // задержка окончена - снимаем сигнал сброса DRAM
	end
	
	
	assign DRAM_UDQM = dram_h ; 
	assign DRAM_LDQM = dram_l ; 

	// стробы подтверждения
	wire sdr_wr_ack, sdr_rd_ack ;
	// тактовый сигнал на память - инверсия синхросигнала шины
	assign DRAM_CLK = clk_n ;

	wire sram_stb = sdram_stb && (sdram_adr[21:17] == 5'b00000) ;
	wire dram_stb = sdram_stb & ~sram_stb ;
	
	assign sdram_dat = (sram_stb ? sram_dat : 16'b0) | (dram_stb ? dram_dat : 16'b0) ;

	// стробы чтения и записи в sdram
	assign sdram_wr = sdram_we & dram_stb ;
	assign sdram_rd = (~sdram_we) & dram_stb ;
	
	// Сигналы выбора старших-младших байтов
	reg dram_h, dram_l ;

	always @ (posedge sdram_stb) begin
		if (sdram_we == 1'b0) begin
			// чтение - всегда словное
			dram_h <= 1'b0 ;
			dram_l <= 1'b0 ;
		end else begin
		// определение записываемых байтов
			dram_h <= ~sdram_sel[1] ;  // старший
			dram_l <= ~sdram_sel[0] ;  // младший
		end
	end  
	
	sram ram(
		.clk_p(clk_p),
		.sdram_reset(sdram_reset),
		.sdram_stb(sram_stb),
		.sdram_we(sdram_we),
		.sdram_sel(sdram_sel),
		.sdram_adr(sdram_adr[21:1]),
		.sdram_out(sdram_out),
		.sdram_ack(sram_ack),
		.sdram_dat(sram_dat)
//		.sdram_ready(sdram_ready)
	) ;

	sdram_top sdram(
		 .clk(clk_p),
		 .rst_n(drs), // запускаем модуль, как только pll выйдет в рабочий режим, запуска процессора не ждем
		 .sdram_wr_req(sdram_wr),
		 .sdram_rd_req(sdram_rd),
		 .sdram_wr_ack(sdr_wr_ack),
		 .sdram_rd_ack(sdr_rd_ack),
		 .sdram_byteenable(sdram_sel),
		 .sys_wraddr({1'b0, sdram_adr[21:1]}),
		 .sys_rdaddr({1'b0, sdram_adr[21:1]}),
		 .sys_data_in(sdram_out),
		 .sys_data_out(dram_dat),
		 .sdwr_byte(1),
		 .sdrd_byte(4),
		 .sdram_cke(DRAM_CKE),
		 .sdram_cs_n(DRAM_CS_N),
		 .sdram_ras_n(DRAM_RAS_N),
		 .sdram_cas_n(DRAM_CAS_N),
		 .sdram_we_n(DRAM_WE_N),
		 .sdram_ba({DRAM_BA_1,DRAM_BA_0}),
		 .sdram_addr(DRAM_ADDR[12:0]),
		 .sdram_data(DRAM_DQ),
		 .sdram_init_done(sdram_ready)     // выход готовности SDRAM
	) ;
	
	assign sdram_ack = (dram_stb & (sdr_rd_ack | sdr_wr_ack))  | (sram_stb & sram_ack) ;

	
	//************************************
	//* Соединительная плата
	//************************************

	`TOPBOARD kernel(
		.clk50(clk50),                  // 50 МГц
		.clk_p(clk_p),                  // тактовая частота процессора, прямая фаза
		.sdclock(sdclock),              // тактовая частота SD-карты
		.clkrdy(clkrdy),                // готовность PLL
		
		.bt_reset(~button[0]),          // общий сброс
		.bt_halt(~button[1]),           // режим программа-пульт / выход из состояния HALT
		.bt_terminal_rst(1'b0),         // сброс терминальной подсистемы
		.bt_timer(1'b0),               // выключатель таймера
		
		.sw_diskbank({2'b00, ~sw[1], ~sw[0]}),   // выбор дискового банка
		.sw_cpuslow(slow_filter[1]),             // режим замедления процессора
		
		// индикаторные светодиоды      
		.disk_led(disk_led),               // запрос обмена диска
		.timer_led(timer_led),         // индикация включения таймера
		.led1(led1),               // признак ожидания прерывания по WAIT
		.led3(led3),                // признак включения MMU 
		.led2(led2),                // признак ативности секвенсера
		
		// Интерфейс SDRAM
		.sdram_reset(sdram_reset),     // сброс
		.sdram_stb(sdram_stb),         // строб начала транзакции
		.sdram_we(sdram_we),           // разрешение записи
		.sdram_sel(sdram_sel),         // выбор байтов
		.sdram_ack(sdram_ack),         // подтверждение транзакции
		.sdram_adr(sdram_adr),         // шина адреса
		.sdram_out(sdram_out),         // выход шины данных
		.sdram_dat(sdram_dat),         // вход шины данных
		.sdram_ready(sdram_ready),     // флаг готовности SDRAM

		// интерфейс SD-карты
		.sdcard_cs(sdcard_cs), 
		.sdcard_mosi(sdcard_mosi), 
		.sdcard_sclk(sdcard_sclk), 
		.sdcard_miso(sdcard_miso), 
		
		// дополнительный UART 
		.irps_txd(irps_txd),
		.irps_rxd(irps_rxd),

		.clkcons(clkcons),
		.cons_row(cons_row),
		.cons_col(cons_col),
		.cons_ledrow(cons_ledrow),
		
		.i2c_SDA(i2c_SDA),
		.i2c_SCL(i2c_SCL)
	);
	
endmodule
