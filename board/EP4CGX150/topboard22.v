//
// FPGA-версия советских PDP-11-совместимых ЭВМ
//
//================================================================================
// Этот модуль - верхний модуль ядра проекта. Представляет собой корзину с общей шиной Wishbone,
// в которую вставлена процессорная плата и модули периферийных устройств.
// 
// Процессорная плата выбирается из нескольких доступных, остальные модули общие для всех используемых процессоров
// и включаются в конфигурацию по выбору, сделанному в файле конфигурации config.v
//
//  Между этим модулем и физическими выводами FPGA находится интерфейсный  модуль под конкретную плату.
//  Интерфейсный модуль содержит в себе контроллер SDRAM, уникальный для каждой платы FPGA.
//  Интерфейсный модуль объявляется корневым модулем проекта.
//===============================================================================================
//
//!  Этот вариант модуля предназначен для подключения процессорных плат с 22-битным адресом
//
//
module topboard22 (

   input          clk50,        // Тактовый сигнал 50 MHz
   input          clk_p,        // основной синхросигнал, прямая фаза
   input          sdclock,      // синхросигнал Sd-карты
   input          clkrdy,       // готовность PLL
   
   // кнопки
   input          bt_reset,         // общий сброс
   input          bt_halt,          // вход в HALT-режим
   input          bt_terminal_rst,  // сброс терминальной подсистемы
   input          bt_timer,         // в этом варианте не используется
   
   // переключатели конфигурации
   input [3:0]    sw_diskbank,  // дисковый банк
   input          sw_cpuslow,   // замедление процессора

   // индикаторные светодиоды      
   output         disk_led,          // запрос обмена диска
   output         timer_led,         // индикация включения таймера
   output         led1,              // индикатор состояния процессора 1
   output         led2,              // индикатор состояния процессора 2
   output         led3,              // индикатор состояния процессора 3
   
   // Интерфейс SDRAM
   output         sdram_reset,       // сброс/переинициализация SDRAM
   output         sdram_stb,         // строб транзакции
   output         sdram_we,          // разрешение записи
   output [1:0]   sdram_sel,         // выбор байтов
   input          sdram_ack,         // подтверждение транзакции
   output [21:1]  sdram_adr,         // шина адреса
   output [15:0]  sdram_out,         // шина данных хост -> память
   input  [15:0]  sdram_dat,         // шина данных память -> хост  
   input          sdram_ready,       // готовность SDRAM

   // интерфейс SD-карты
   output         sdcard_cs, 
   output         sdcard_mosi, 
   output         sdcard_sclk, 
   input          sdcard_miso, 
   
   // дополнительный UART 
   output         irps_txd,
   input          irps_rxd,

   // console panel
	input					clkcons,
	output 	[2:0]		cons_row,
	inout		[11:0]	cons_col,
	output	[5:0]		cons_ledrow,

	inout					i2c_SDA,		//SDA line
	inout					i2c_SCL		//SCL line
);

wire        sys_init;         // общий сброс

// шина WISHBONE                                       
wire        wb_clk;                  // тактовая частота
wire [21:0] wb_adr;                  // полный 22-битный адрес  
wire [15:0] wb_out;                  // выход данных от процессора   
wire [15:0] wb_mux;                  // вход данных в процессор  
wire        wb_we;                   // разрешение записи  
wire [1:0]  wb_sel;                  // выбор байтов из слова  
wire        wb_stb;                  // строб обмена по шине  
wire        global_ack;              // подтверждение обмена от устройств на шине      
wire [17:0] dma_adr18;               // 18-битный unibus-адрес от устройств, использующих DMA в режиме UMR
wire        dma_stb_ubm;             // строб обмена от устройств, работающих с DMA через механизм UMR

// Основная шина процессора
wire        cpu_access_req;          // разрешение доступа к шине
wire [21:0] cpu_adr;                 // шина адреса
wire [15:0] cpu_data_out;            // выход шины данных
wire        cpu_we;                  // направление передачи (1 - от процессора)
wire [1:0]  cpu_bsel;                // выбор байтов из слова
wire        cpu_ram_stb;             // строб доступа к памяти со  стороны процессора

// сигналы выбора периферии
wire uart1_stb ;
wire uart2_stb ;
wire rk11_stb ;
wire rh70_stb ;

wire bus_stb;

// линии подтверждения обмена, исходяшие из устройства
wire uart1_ack ;
wire uart2_ack ;
wire rk11_ack ;
wire rl11_ack ;
wire rh70_ack ;
wire kw11p_ack ;
wire pr11_ack ;
wire pp11_ack ;
wire lp11_ack ;

//  Шины данных от периферии
wire [15:0] uart1_dat;
wire [15:0] uart2_dat;
wire [15:0] rk11_dat;
wire [15:0] rl11_dato ;
wire [15:0] rh70_dato ;
wire [15:0] kw11p_dato ;
wire [15:0] pr11_dato ;
wire [15:0] pp11_dato ;
wire [15:0] lp11_dato ;

// линии процессорных сбросов 
wire        vm_dclo_in;                // вход сброса

// линии прерывания внешних устройств                                       
wire uart1_tx_irq, uart1_tx_iack;            
wire uart1_rx_irq, uart1_rx_iack;            
wire rk11_irq, rk11_iack;
wire rl11_irq, rl11_iack ;
wire rh70_irq, rh70_iack ;
wire kw11p_irq, kw11p_iack;
wire pr11_irq, pr11_iack ;
wire pp11_irq, pp11_iack ;
wire lp11_irq, lp11_iack ;

wire global_reset;   // повторитель кнопки сброса
//wire sdram_ready ;
wire [8:0] vector;      // передаваемый процессору вектор прерывания
wire [15:0] irq4_ivec;  // выход вектора приоритета 4
wire [15:0] irq5_ivec;  // выход вектора приоритета 5
wire [15:0] irq7_ivec;  // выход вектора приоритета 7
wire [7:4]  istb;       // стробы запроса вектора от CPU для каждого приоритета

// линии запроса прерывания к процессору
wire br4_irq;
wire br5_irq;
wire br6_irq;
wire br7_irq;
// линии подтверждения приема вектора от процессора
wire br4_iack;
wire br5_iack;
wire br6_iack = 1'b0 ;
wire br7_iack;


// Линии обмена с SD-картой от разных контроллеров
wire         rk_mosi;       // mosi от RK11
wire         rk_cs;         // cs от RK11
wire         rk_sclk;       // sclk от RK11

wire         rh70_mosi ;    // mosi от DB
wire         rh70_cs ;      // cs от DB
wire         rh70_sclk ;

wire         pr11_mosi;
wire         pr11_cs;
wire         pr11_sclk;

wire         pp11_mosi;
wire         pp11_cs;
wire         pp11_sclk;

// Сигналы диспетчера доступа к SD-карте
wire        rk_sdreq;       // запрос доступа
reg         rk_sdack;       // разрешение доступа
wire        rh70_sdreq ;
reg         rh70_sdack ; 
wire        pr11_sdreq;       // запрос доступа
reg         pr11_sdack;       // разрешение доступа
wire        pp11_sdreq;       // запрос доступа
reg         pp11_sdack;       // разрешение доступа

// основная тактовая частота шины
assign wb_clk=clk_p;

//**************************************************************
//*   Модуль формирования сбросов
//**************************************************************

wbc_rst reset
(
   .osc_clk(clk50),             // основной клок 50 МГц
   .sys_clk(wb_clk),            // сигнал синхронизации  wishbone
   .pll_lock(clkrdy),           // сигнал готовности PLL
   .button(~bt_reset & ~cons_sigo[0]),  // кнопка сброса
   .sys_ready(sdram_ready),     // вход готовности системных компонентов (влияет на sys_rst)
   .sys_dclo(vm_dclo_in),                 // .sys_dclo(vm_dclo_in)
   .global_reset(global_reset)  // выход повторителя кнопки сброса, с отфильтрованным дребезгом
);

//*********************************************
//*  Интерфейс к модулю SDRAM
//*********************************************
	assign sdram_reset=global_reset;       // сигнал сброса модуля SDRAM
	assign sdram_we=wb_we;                 // признак транзакции записи
	assign sdram_sel=wb_sel;               // выбор байтов
	assign sdram_adr=wb_adr[21:1];         // шина адреса
	assign sdram_out=wb_out;               // выходная шина данных

// Строб SDRAM                          
	assign sdram_stb =
	`ifdef RAM256
		(cpu_ram_stb && (wb_adr[21:18] == 4'b0000)); // обрезка памяти до 256К
	`elsif RAM1M	
		(cpu_ram_stb && (wb_adr[21:20] == 2'b00)); // обрезка памяти до 1M
	`else
		cpu_ram_stb;   // полные 4М памяти
	`endif
	
//*****************************************************************************
//* Диспетчер доступа к общей шине по запросу от разных мастеров (арбитр DMA)
//*****************************************************************************
wire rk11_dma_req;
reg rk11_dma_state ;
wire rl11_dma_req;
reg rl11_dma_gnt ;
wire rh70_dma_req;
reg rh70_dma_gnt ;

wire dma_req;  // запрос DMA
wire dma_ack;  // подтверждение DMA

wire [3:0]	cons_sigo ;
wire			cons_dma_req ;
reg			cons_dma_gnt ;
wire [17:0]	cons_dma_adr_o ;
wire [15:0]	cons_dma_dat_o ;
wire 			cons_dma_stb_o ;
wire			cons_dma_we_o ;

// запрос DMA к процессору
assign dma_req = rk11_dma_req | rl11_dma_req | rh70_dma_req | cons_dma_req ;

//**********************************************************
//*       Процессорная плата
//**********************************************************
`BOARD cpu(
// Синхросигналы  
   .clk_p(clk_p),
   .cpuslow(sw_cpuslow),          // Режим замедления процессора

   .wb_adr_o(cpu_adr),            // выход шины адреса
   .wb_dat_o(cpu_data_out),       // выход шины данных
   .wb_dat_i(wb_mux),             // вход шины данных
   .wb_we_o(cpu_we),              // разрешение записи
   .wb_sel_o(cpu_bsel),           // выбор байтов для передачи
   .global_ack(global_ack),       // подтверждение обмена от памяти и устройств страницы ввода-вывода
   .ram_stb(cpu_ram_stb),         // строб обращения к основной памяти
   .bus_stb(bus_stb),             // строб обращения к общей шине
   
// DMA   
   .dma_req(dma_req),             // запрс DMA 
   .dma_ack(dma_ack),             // подтверждение DMA - процессор освободил шину
   .dma_adr18(dma_adr18),         // ввод 18-битного адреса для устройств, работающих через Unibsus Mapping 
   .dma_stb(dma_stb_ubm),         // строб данных для устройств, работающих через Unibsus Mapping 

// Сбросы и прерывания
   .bus_reset(sys_init),           // Выход сброса для периферии
   .dclo(vm_dclo_in),              // Вход сброса процессора
   
// Ручное управление   
   .bt_halt(bt_halt),              // вход в режим HALT или возобновление работы секвенсера
   
// Индикаторы   
   .led1(led1),              // индикатор 1
   .led2(led2),              // индикатор 2
   .led3(led3),              // индикатор 3
   .led_timer(timer_led),    // индикация включения таймера
   
// Шины обработки прерываний                                       
   .irq_i({br7_irq, br6_irq, br5_irq, br4_irq}),     // Запрос на векторное прерывание 
   .istb_o(istb),                  // Строб от процессора, разрешающий выдачу вектора 
   .ivec(vector),                  // Шина приема вектора прерывания
   .iack_i(br4_iack | br5_iack | br6_iack | br7_iack),     // Подтверждение приема вектора прерывания
   
	.clkcons(clkcons),
	.cons_sigo(cons_sigo),
	
	.cons_dma_req(cons_dma_req),
	.cons_dma_gnt(cons_dma_gnt),
	.cons_dma_adr_o(cons_dma_adr_o),
	.cons_dma_dat_i(wb_mux),
	.cons_dma_dat_o(cons_dma_dat_o),
	.cons_dma_stb_o(cons_dma_stb_o),
	.cons_dma_we_o(cons_dma_we_o),
	
	.cons_row(cons_row),
	.cons_col(cons_col),
	.cons_ledrow(cons_ledrow)
);

//**********************************************
// Выбор скорости последовательных портов
//**********************************************
wire [15:0] uart1_speed;  // скорость ИРПС 1
wire [15:0] baud2;        // делитель скорости второго порта ИРПС

// Согласование скорости с терминальным модулем
// Выбор скорости второго UART                        
assign baud2 = 
  (`UART2SPEED == 3'd0)   ? 16'd767: // 1200
  (`UART2SPEED == 3'd1)   ? 16'd383: // 2400
  (`UART2SPEED == 3'd2)   ? 16'd191: // 4800
  (`UART2SPEED == 3'd3)   ? 16'd95:  // 9600
  (`UART2SPEED == 3'd4)   ? 16'd47:  // 19200
  (`UART2SPEED == 3'd5)   ? 16'd23:  // 38400
  (`UART2SPEED == 3'd6)   ? 16'd15:  // 57600
                            16'd7;   // 115200

assign uart1_speed = baud2;

//**********************************
//*     ирпс1 (консоль)
//**********************************
wbc_uart #(.REFCLK(`clkref)) uart1
(
   .wb_clk_i(wb_clk),
   .wb_rst_i(sys_init),
   .wb_adr_i(wb_adr[2:0]),
   .wb_dat_i(wb_out),
   .wb_dat_o(uart1_dat),
   .wb_cyc_i(1'b1),
   .wb_we_i(wb_we),
   .wb_stb_i(uart1_stb),
   .wb_ack_o(uart1_ack),

   .txd(irps_txd),
   .rxd(irps_rxd),

   .tx_irq_o(uart1_tx_irq),
   .tx_iack_i(uart1_tx_iack),
   .rx_irq_o(uart1_rx_irq),
   .rx_iack_i(uart1_rx_iack),

   .cfg_bdiv(uart1_speed),
   .cfg_nbit(2'b11),
   .cfg_nstp(1'b1),
   .cfg_pena(1'b0),
   .cfg_podd(1'b0)
);

//****************************************************
//*  Дисковый контроллер RK11D (DK:/RK:)
//****************************************************

// выходная шина DMA
wire [17:0] rk11_adr;
wire        rk11_dma_stb;
wire        rk11_dma_we;
wire [15:0] rk11_dma_out;

`ifdef RK_module

rk11 rkdisk (

// шина wishbone
   .wb_clk_i(wb_clk),      // тактовая частота шины
   .wb_rst_i(sys_init),    // сброс
   .wb_adr_i(wb_adr[3:0]), // адрес 
   .wb_dat_i(wb_out),      // входные данные
   .wb_dat_o(rk11_dat),    // выходные данные
   .wb_cyc_i(1'b1),        // начало цикла шины
   .wb_we_i(wb_we),        // разрешение записи (0 - чтение)
   .wb_stb_i(rk11_stb),    // строб цикла шины
   .wb_sel_i(wb_sel),      // выбор конкретных байтов для записи - старший, младший или оба
   .wb_ack_o(rk11_ack),    // подтверждение выбора устройства

// обработка прерывания   
   .irq(rk11_irq),         // запрос
   .iack(rk11_iack),       // подтверждение
   
// DMA
   .dma_req(rk11_dma_req), // запрос DMA
   .dma_gnt(rk11_dma_state), // подтверждение DMA
   .dma_adr_o(rk11_adr),   // выходной адрес при DMA-обмене
   .dma_dat_i(wb_mux),     // входная шина данных DMA
   .dma_dat_o(rk11_dma_out), // выходная шина данных DMA
   .dma_stb_o(rk11_dma_stb), // строб цикла шины DMA
   .dma_we_o(rk11_dma_we),   // направление передачи DMA (0 - память->диск, 1 - диск->память) 
   .dma_ack_i(global_ack), // Ответ от устройства, с которым идет DMA-обмен
   
// интерфейс SD-карты
   .sdcard_cs(rk_cs), 
   .sdcard_mosi(rk_mosi), 
   .sdcard_miso(sdcard_miso), 
   .sdcard_sclk(rk_sclk),

   .sdclock(sdclock),
   .sdreq(rk_sdreq),
   .sdack(rk_sdack),
   .sdmode(`RK_sdmode),           // режим ведущего-ведомого
   
// Адрес массива дисков на карте
   .start_offset({1'b0, sw_diskbank, 22'h0})
) ; 

`else 
assign rk11_ack=1'b0;
assign rk11_dma_req=1'b0;
assign rk_sdreq = 1'b0;
assign rk11_irq=1'b0;
`endif


wire [17:0]	rl11_dma_adr_o ;
wire [15:0]	rl11_dma_dat_o ;
wire 			rl11_dma_stb_o ;
wire			rl11_dma_we_o ;

wire         rl11_mosi;
wire         rl11_cs;
wire         rl11_sclk;

wire        rl11_sdreq;       // запрос доступа
reg         rl11_sdack;       // разрешение доступа

rl11 rldisk(
	.clk_p(clk_p),
	.sys_init(sys_init),
	
	.wb_adr_i(wb_adr),
	.wb_dat_i(wb_out),
	.wb_dat_o(rl11_dato),
	.bus_stb(bus_stb),
	.wb_we_i(wb_we),
	.wb_sel_i(wb_sel),
	.rl11_ack(rl11_ack),
	
	.irq(rl11_irq),
	.iack(rl11_iack),
	
   .dma_req(rl11_dma_req), // запрос DMA
   .dma_gnt(rl11_dma_gnt), // подтверждение DMA
   .dma_adr_o(rl11_dma_adr_o),   // выходной адрес при DMA-обмене
   .dma_dat_i(wb_mux),     // входная шина данных DMA
   .dma_dat_o(rl11_dma_dat_o), // выходная шина данных DMA
   .dma_stb_o(rl11_dma_stb_o), // строб цикла шины DMA
   .dma_we_o(rl11_dma_we_o),   // направление передачи DMA (0 - память->диск, 1 - диск->память) 
   .dma_ack_i(global_ack), // Ответ от устройства, с которым идет DMA-обмен

   .sdcard_cs(rl11_cs), 
   .sdcard_mosi(rl11_mosi), 
   .sdcard_miso(sdcard_miso), 
   .sdcard_sclk(rl11_sclk),
	
   .sdclock(sdclock),
   .sdreq(rl11_sdreq),
   .sdack(rl11_sdack),

   .start_offset({1'b0, sw_diskbank, 22'h9840})
) ;

wire [21:0] rh70_dma_adr ;
wire        rh70_dma_stb ;
wire        rh70_dma_we ;
wire [15:0] rh70_dma_out ;

rh70 db_disk (
// шина wishbone
   .wb_clk_i(wb_clk),      // тактовая частота шины
   .wb_rst_i(sys_init),    // сброс
   .wb_adr_i(wb_adr[5:0]), // адрес 
   .wb_dat_i(wb_out),      // входные данные
   .wb_dat_o(rh70_dato),    // выходные данные
   .wb_cyc_i(1'b1),        // начало цикла шины
   .wb_we_i(wb_we),        // разрешение записи (0 - чтение)
   .wb_stb_i(rh70_stb),    // строб цикла шины
   .wb_sel_i(wb_sel),      // выбор конкретных байтов для записи - старший, младший или оба
   .wb_ack_o(rh70_ack),    // подтверждение выбора устройства

// обработка прерывания   
   .irq(rh70_irq),         // запрос
   .iack(rh70_iack),       // подтверждение
   
// DMA
   .dma_req(rh70_dma_req),   // запрос DMA
   .dma_gnt(rh70_dma_gnt), // подтверждение DMA
   .dma_adr_o(rh70_dma_adr), // выходной адрес при DMA-обмене
   .dma_dat_i(wb_mux),       // входная шина данных DMA
   .dma_dat_o(rh70_dma_out), // выходная шина данных DMA
   .dma_stb_o(rh70_dma_stb), // строб цикла шины DMA
   .dma_we_o(rh70_dma_we),   // направление передачи DMA (0 - память->диск, 1 - диск->память) 
   .dma_ack_i(global_ack), // Ответ от устройства, с которым идет DMA-обмен
   
// интерфейс SD-карты
   .sdcard_cs(rh70_cs), 
   .sdcard_mosi(rh70_mosi), 
   .sdcard_miso(sdcard_miso), 
   .sdcard_sclk(rh70_sclk),

   .sdclock(sdclock),
   .sdreq(rh70_sdreq),
   .sdack(rh70_sdack),
   .sdmode(1'b0),           // режим ведущего-ведомого
   
// Адрес массива дисков на карте
   .start_offset({1'b0, sw_diskbank, 22'h31908})
) ;

wire [15:0] toy_dato ;
wire toy_ack ;

toy11 toy (
	.clk_p(clk_p),
	.sys_init(sys_init),
	.wb_adr_i(wb_adr),
	.wb_dat_i(wb_out),
	.wb_dat_o(toy_dato),
	.bus_stb(bus_stb),
	.wb_we_i(wb_we),
	.wb_sel_i(wb_sel),
	.toy_ack(toy_ack),
	
	.i2c_SDA(i2c_SDA),
	.i2c_SCL(i2c_SCL)
) ;

kw11p kwp(
	.clk_p(clk_p),
	.sys_init(sys_init),

	.wb_adr_i(wb_adr),
	.wb_dat_i(wb_out),
	.wb_dat_o(kw11p_dato),
	.bus_stb(bus_stb),
	.wb_we_i(wb_we),
	.wb_sel_i(wb_sel),
	.kw11p_ack(kw11p_ack),
	
	.irq(kw11p_irq),
	.iack(kw11p_iack)
) ;

pr11 paper_reader(
	.clk_p(clk_p),
	.sys_init(sys_init),

	.wb_adr_i(wb_adr),
	.wb_dat_i(wb_out),
	.wb_dat_o(pr11_dato),
	.bus_stb(bus_stb),
	.wb_we_i(wb_we),
	.wb_sel_i(wb_sel),
	.pr11_ack(pr11_ack),
	
	.irq(pr11_irq),
	.iack(pr11_iack),

	// интерфейс SD-карты
   .sdcard_cs(pr11_cs), 
   .sdcard_mosi(pr11_mosi), 
   .sdcard_miso(sdcard_miso), 
   .sdcard_sclk(pr11_sclk),

   .sdclock(sdclock),
   .sdreq(pr11_sdreq),
   .sdack(pr11_sdack),
   
// Адрес массива дисков на карте
   .start_offset({1'b0, sw_diskbank, 22'h31840})
) ;

pp11 paper_punch(
	.clk_p(clk_p),
	.sys_init(sys_init),

	.wb_adr_i(wb_adr),
	.wb_dat_i(wb_out),
	.wb_dat_o(pp11_dato),
	.bus_stb(bus_stb),
	.wb_we_i(wb_we),
	.wb_sel_i(wb_sel),
	.pp11_ack(pp11_ack),
	
	.irq(pp11_irq),
	.iack(pp11_iack),

	// интерфейс SD-карты
   .sdcard_cs(pp11_cs), 
   .sdcard_mosi(pp11_mosi), 
   .sdcard_miso(sdcard_miso), 
   .sdcard_sclk(pp11_sclk),

   .sdclock(sdclock),
   .sdreq(pp11_sdreq),
   .sdack(pp11_sdack),
   
// Адрес массива дисков на карте
   .start_offset({1'b0, sw_diskbank, 22'h31840})
) ;

lp11 printer(
	.clk_p(clk_p),
	.sys_init(sys_init),

	.wb_adr_i(wb_adr),
	.wb_dat_i(wb_out),
	.wb_dat_o(lp11_dato),
	.bus_stb(bus_stb),
	.wb_we_i(wb_we),
	.wb_sel_i(wb_sel),
	.lp11_ack(lp11_ack),
	
	.irq(lp11_irq),
	.iack(lp11_iack)
) ;
  
//**********************************
//*  Диспетчер доступа к SD-карте
//**********************************
reg [1:0] rk_sdreq_filter;
reg [1:0] rl11_sdreq_filter;
reg [1:0] rh70_sdreq_filter;
reg [1:0] pr11_sdreq_filter;
reg [1:0] pp11_sdreq_filter;

// фильтрация сигналов запроса
always @(posedge sdclock) begin
  rk_sdreq_filter[0]=rk_sdreq;
  rk_sdreq_filter[1]=rk_sdreq_filter[0];

  rl11_sdreq_filter[0] = rl11_sdreq;
  rl11_sdreq_filter[1] = rl11_sdreq_filter[0];

  rh70_sdreq_filter[0] = rh70_sdreq;
  rh70_sdreq_filter[1] = rh70_sdreq_filter[0];

  pr11_sdreq_filter[0] = pr11_sdreq;
  pr11_sdreq_filter[1] = pr11_sdreq_filter[0];

  pp11_sdreq_filter[0] = pp11_sdreq;
  pp11_sdreq_filter[1] = pp11_sdreq_filter[0];
end  
  
always @(posedge sdclock) begin
   // сброс
   if (sys_init == 1'b1) begin
      rk_sdack <= 1'b0;
		rl11_sdack <= 1'b0 ;
		rh70_sdack <= 1'b0 ;
		pr11_sdack <= 1'b0 ;
		pp11_sdack <= 1'b0 ;
   end else if ((rk_sdack == 1'b0) && (rl11_sdack == 1'b0) && (rh70_sdack == 1'b0) && (pr11_sdack == 1'b0) && (pp11_sdack == 1'b0)) begin // поиск контроллера, желающего доступ к карте
      if (rk_sdreq_filter[1] == 1'b1) // неактивное состояние - ищем источник запроса
			rk_sdack <= 1'b1 ;
		else if (rl11_sdreq_filter[1] == 1'b1)
			rl11_sdack <= 1'b1 ;
		else if (rh70_sdreq_filter[1] == 1'b1)
			rh70_sdack <= 1'b1 ;
		else if (pr11_sdreq_filter[1] == 1'b1)
			pr11_sdack <= 1'b1 ;
		else if (pp11_sdreq_filter[1] == 1'b1)
			pp11_sdack <= 1'b1 ;
   end else // активное состояние - ждем освобождения карты
      if ((rk_sdack == 1'b1) && (rk_sdreq_filter[1] == 1'b0))
			rk_sdack <= 1'b0;
		else if ((rl11_sdack == 1'b1) && (rl11_sdreq_filter[1] == 1'b0))
			rl11_sdack <= 1'b0;
		else if ((rh70_sdack == 1'b1) && (rh70_sdreq_filter[1] == 1'b0))
			rh70_sdack <= 1'b0;
		else if ((pr11_sdack == 1'b1) && (pr11_sdreq_filter[1] == 1'b0))
			pr11_sdack <= 1'b0;
		else if ((pp11_sdack == 1'b1) && (pp11_sdreq_filter[1] == 1'b0))
			pp11_sdack <= 1'b0;
end
   
//**********************************
//* Мультиплексор линий SD-карты
//**********************************
assign sdcard_mosi =
			pp11_sdack ? pp11_mosi :
			pr11_sdack ? pr11_mosi :
			rh70_sdack ? rh70_mosi :
			rl11_sdack ? rl11_mosi :
         rk_sdack   ? rk_mosi   : // RK
                   `def_mosi ; // по умолчанию - контроллер с ведущим SDSPI

assign sdcard_cs =
			pp11_sdack ? pp11_cs :
			pr11_sdack ? pr11_cs :
			rh70_sdack ? rh70_cs :
			rl11_sdack ? rl11_cs :
         rk_sdack   ? rk_cs   :   // RK
                   `def_cs;   // по умолчанию - контроллер с ведущим SDSPI
                   
assign sdcard_sclk =
			pp11_sdack ? pp11_sclk :
			pr11_sdack ? pr11_sclk :
			rh70_sdack ? rh70_sclk :
			rl11_sdack ? rl11_sclk :
         rk_sdack   ? rk_sclk   :   // RK
                   `def_sclk ;   // по умолчанию - контроллер с ведущим SDSPI
            

//********************************************
//* Светодиоды дисковой активности
//********************************************
assign disk_led = rk_sdreq | pr11_sdreq | pr11_sdreq | rl11_sdreq | rh70_sdreq ;   // запрос обмена диска

//************************************************
//*  Контроллеры прерываний
//************************************************

// приоритет 4
wbc_vic #(.N(5)) vic4
(
   .wb_clk_i(wb_clk),
   .wb_rst_i(sys_init),
   .wb_irq_o(br4_irq),
   .wb_dat_o(irq4_ivec),
   .wb_stb_i(istb[4]),
   .wb_ack_o(br4_iack),
//         UART1-Tx       UART1-Rx      PR11       PP11       LP11
   .ivec({16'o64,         16'o60,       16'o70,    16'o74,    16'o200}),
   .ireq({uart1_tx_irq,  uart1_rx_irq,  pr11_irq,  pp11_irq,  lp11_irq}),
   .iack({uart1_tx_iack, uart1_rx_iack, pr11_iack, pp11_iack, lp11_iack})
);

// приоритет 5
wbc_vic #(.N(3)) vic5
(
   .wb_clk_i(wb_clk),
   .wb_rst_i(sys_init),
   .wb_irq_o(br5_irq),
   .wb_dat_o(irq5_ivec),
   .wb_stb_i(istb[5]),
   .wb_ack_o(br5_iack),
//         RK11      RL11       RH70
   .ivec({16'o220,   16'o160,   16'o254}),
   .ireq({rk11_irq,  rl11_irq,  rh70_irq}),
   .iack({rk11_iack, rl11_iack, rh70_iack})
);

// приоритет 7
wbc_vic #(.N(1)) vic7
(
   .wb_clk_i(wb_clk),
   .wb_rst_i(sys_init),
   .wb_irq_o(br7_irq),
   .wb_dat_o(irq7_ivec),
   .wb_stb_i(istb[7]),
   .wb_ack_o(br7_iack),
//      KW11P 
   .ivec({16'o000104}),
   .ireq({kw11p_irq}),
   .iack({kw11p_iack})
);

// коммутатор источника вектора прерывания
assign vector= (istb[4]) ? irq4_ivec[8:0] : 9'o0
             | (istb[5]) ? irq5_ivec[8:0] : 9'o0
             | (istb[7]) ? irq7_ivec[8:0] : 9'o0
				 ;


// арбитр DMA
always @(posedge wb_clk) begin
   if (sys_init) begin
      // сброс арбитра
      rk11_dma_state <= 1'b0;
		rl11_dma_gnt <= 1'b0 ;
		rh70_dma_gnt <= 1'b0 ;
		cons_dma_gnt <= 1'b0 ;
   end   
   // поиск активного запроса DMA
   else if (dma_ack) begin
		// Нет активного DMA-устройства - выбор устройства, которому предоставляется доступ к шине
		if (~(rk11_dma_state | rl11_dma_gnt | rh70_dma_gnt | cons_dma_gnt)) begin
			if (cons_dma_req)
				cons_dma_gnt <= 1'b1 ;
			else
				if (rh70_dma_req == 1'b1) 
					rh70_dma_gnt <= 1'b1;
			else
				if (rl11_dma_req == 1'b1) 
					rl11_dma_gnt <= 1'b1;
			else
				if (rk11_dma_req == 1'b1) 
					rk11_dma_state <= 1'b1;  // запрос от RK11
		end  
		else begin
			// Имеется активное DMA-устройство - ожидание освобождения шины
			if (cons_dma_req == 1'b0)
				cons_dma_gnt <= 1'b0 ;

			if (rh70_dma_req == 1'b0)
				rh70_dma_gnt <= 1'b0 ;

				if (rl11_dma_req == 1'b0)
				rl11_dma_gnt <= 1'b0 ;

				if (rk11_dma_req == 1'b0)
				rk11_dma_state <= 1'b0;       
		end  
   end
end

 
//*******************************************************************
//*  Коммутатор источника управления (мастера) шины wishbone
//*******************************************************************

// Основная адресная шина
// адрес переключается только для контроллеров DB/RH70 и MY
// адреса остальных контроллеров идут через  Unibus Mapping
// если процессор не поддерживает massbus (все кроме 11/70) то DB/RH70 таже идет через UM
assign wb_adr = (rh70_dma_gnt) ? rh70_dma_adr : cpu_adr ;

// Адресная шина UNIBUS - DMA-запрсы идут через MMU подсистему Unibus Mapping
assign dma_adr18 = cons_dma_gnt ? cons_dma_adr_o : rk11_dma_state ? rk11_adr : rl11_dma_gnt ? rl11_dma_adr_o : rh70_dma_gnt ? rh70_dma_adr : 18'o0 ;

// Выходная шина данных, от мастера DMA к ведомому устройству
assign wb_out = cons_dma_gnt ? cons_dma_dat_o : rk11_dma_state ? rk11_dma_out : rl11_dma_gnt ? rl11_dma_dat_o : rh70_dma_gnt ? rh70_dma_out : 16'o0 | (~dma_ack) ? cpu_data_out: 16'o0 ;

// Сигнал направления передачи - 1 = от устройства в память, 0 = из памяти в устройство
assign wb_we = cons_dma_we_o | rk11_dma_we | rl11_dma_we_o | rh70_dma_we | (~dma_ack & cpu_we);

// Выбор байтов для записи                                           
assign wb_sel =   (dma_ack) ? 2'b11: cpu_bsel;
                          
// Строб данных от DMA-мастера, работающего через Unibus Mapping
assign dma_stb_ubm = (rk11_dma_state & rk11_dma_stb) | (rl11_dma_gnt & rl11_dma_stb_o) | (rh70_dma_gnt & rh70_dma_stb) | (cons_dma_gnt & cons_dma_stb_o) ;
 
//*******************************************************************
//*  Сигналы управления шины wishbone
//******************************************************************* 
// Страница ввода-вывода
assign uart1_stb  = bus_stb & (wb_adr[15:3] == (16'o177560 >> 3));   // ИРПС консольный (TT) - 177560-177566 
assign rk11_stb   = bus_stb & (wb_adr[15:4] == (16'o177400 >> 4));   // RK - 177400-177416
assign rh70_stb   = bus_stb & (wb_adr[15:6] == (16'o176700 >> 6));   // RH70 - 176700-176776 для massbus-конфигураций

// Сигналы подтверждения - собираются через OR со всех устройств
assign global_ack  	= sdram_ack
							| uart1_ack
							| rk11_ack
							| rl11_ack
							| rh70_ack
							| toy_ack
							| kw11p_ack
							| pr11_ack
							| pp11_ack
							| lp11_ack ;

// Мультиплексор выходных шин данных всех устройств
assign wb_mux = 
       (sdram_stb ? sdram_dat   : 16'o000000)
     | (uart1_stb ? uart1_dat	  : 16'o000000)
     | (rk11_stb  ? rk11_dat    : 16'o000000)
     | (rl11_ack  ? rl11_dato   : 16'o000000)
     | (rh70_ack  ? rh70_dato   : 16'o000000)
     | (toy_ack   ? toy_dato    : 16'o000000)
     | (kw11p_ack ? kw11p_dato  : 16'o000000)
     | (pr11_ack  ? pr11_dato   : 16'o000000)
     | (pp11_ack  ? pp11_dato   : 16'o000000)
     | (lp11_ack  ? lp11_dato   : 16'o000000)
;

endmodule
