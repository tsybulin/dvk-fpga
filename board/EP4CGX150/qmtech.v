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
	input			[2:0]	sw,           // переключатели конфигурации
	output		[4:0]	led,          // индикаторные светодиоды   

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

	assign led[0] = disk_led ;    // запрос обмена диска 
	assign led[1] = ~led1 ;      // Индикатор состояния процессора 1
	assign led[2] = ~led2 ;      // Индикатор состояния процессора 2
	assign led[3] = ~led3 ;      // Индикатор состояния процессора 3
	assign led[4] = ~timer_led ; // индикация включения таймера  

	//************************************************
	//* тактовый генератор 
	//************************************************
	wire clk_p, sdclock, clkrdy, clkcons ;

	pll pll1 (
		.inclk0(clk50),
		.areset(~rst_n),
		.c0(clk_p),     // основная тактовая частота, прямая фаза
		.c1(sdclock),   // тактовый сигнал SD-карты
		.c2(clkcons),   // console 1MHz
		.locked(clkrdy) // флаг готовности PLL
	) ;

	reg [1:0] slow_filter ;

	always @ (posedge clk_p)  begin
		slow_filter[0] <= ~sw[2] ;
		slow_filter[1] <= slow_filter[0] & ~sw[2] ;
	end

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
