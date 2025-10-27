//
//  Процессорный модуль - плата PDP2011 (Электроника-79)
//  Центральный процессор - PDP11/70
// 
// ======================================================================================

module pdp2011 (
// Синхросигналы  
   input          clk_p,         // тактовый сигнал, прямая фаза
   input          cpuslow,       // Режим замедления процессора
                                       
// Шина Wishbone                                       
   output [21:0] wb_adr_o,       // выход шины адреса
   output [15:0] wb_dat_o,       // выход шины данных
   input  [15:0] wb_dat_i,       // вход шины данных
   output wb_we_o,               // разрешение записи
   output [1:0] wb_sel_o,        // выбор байтов для передачи

   output ram_stb,               // строб обращения к системной памяти
   output bus_stb,               // строб обращения к общей шине
   input  global_ack,            // подтверждение обмена от памяти и устройств страницы ввода-вывода

// DMA   
   input   dma_req,              // запрос DMA
   output  dma_ack,              // подтверждение DMA
   input   [17:0] dma_adr18,     // 18-битный UNIBUS-адрес для устройств, работающих через UBM
   input   dma_stb,              // строб данных для устройств, работающих через UBM
   
// Сбросы и прерывания
   output bus_reset,             // Выход сброса для периферии
   input dclo,                   // Вход сброса процессора
   
// сигналы ручного управления   
   input bt_halt,                 // Сигнал запуска процессора после HALT
// Информационные индикаторы   
   output led1,                  // индикация бездействия (WAIT)
   output led2,                  // индикация работы процессора (~HALT)
   output led3,                  // индикация включения MMU
   output led_timer,             // индикация включения таймера
   
// Шины обработки прерываний                                       
   input [7:4] irq_i,            // Запрос прерывания
   output [7:4] istb_o,          // Строб приема вектора
   input [8:0] ivec,             // Шина приема вектора прерывания
   input iack_i,                  // Подтверждение приема вектора прерывания

   // Console
	input					clkcons,
	output				cons_dma_req,
	input					cons_dma_gnt,
	output 	[17:0]	cons_dma_adr_o,
	input		[15:0]	cons_dma_dat_i,
	output	[15:0]	cons_dma_dat_o,
	output				cons_dma_stb_o,
	output				cons_dma_we_o,
	
	output	[3:0]		cons_sigo,
	output 	[2:0]		cons_row,
	inout		[11:0]	cons_col,
	output	[5:0]		cons_ledrow
);   
   
//==========================================================================================
wire sys_init;              
                                      
wire [15:0] wb_mux;    // сборная шина данных от периферии к процессору                
wire        cpu_stb;   // строб данных от процессор на шину            
wire        wb_ack;    // подтверждения обмена от шины к процессору            

// шины данных внутренней периферии
wire [15:0] ccr_dat;   // блок управляющих регистров          
wire [15:0] kw11l_dat; // таймер
wire [15:0] mmu_dat;   // MMU

// сигналы подтверждения обмена
wire ccr_ack;             
wire cons_ack;
reg kw11l_ack;
wire cpu_ack;      

// стробы выбора периферии
wire swr_stb;
wire kw11l_stb;             
wire ccr_stb;

// PSW
wire [15:0] cpu_psw_in;   // Ввод PSW , записываемого под адресу 177776
wire cpu_psw_we_even;     // Разрешение записи младшего байта PSW
wire cpu_psw_we_odd;      // Разрешение записи старшего байта PSW
wire [15:0] cpu_psw_out;  // Вывод PSW , читаемого под адресу 177776

// флаги ошибок
wire oddabort;
wire cpu_ysv;            // желтая граница стека
wire cpu_rsv;            // красная граница стека
wire cpu_illegal_halt;   // HALT не в режиме kernel
wire bus_timeout;        // таймаут обмена по шине

// регистр границы стека
wire [15:0] cpu_stack_limit;

// индикация состояния
wire iwait;
wire run;
assign led3= ~(cons_map18 | cons_map22);
assign led2=~run;
assign led1=~iwait;

// сброс системы
assign      sys_init = bus_reset;

// интерфейс cpu<->mmu
wire cpu_cp;
wire cpu_id;
wire mmutrap;
wire ack_mmutrap;
wire mmuabort;
wire sr0_ic;
wire [15:0] sr1;
wire [15:0] sr2;
wire cons_map16;
wire cons_map18;
wire cons_map22;
wire cons_id;
wire cons_ubm;
wire ifetch;
wire dstfreference;
wire dw8;
wire [15:0] cpu_adr;
wire ack_mmuabort;
wire mmuoddabort;

// таймер
reg timer_ie;    // разрешение прерывания
reg timer_rdy;   // готовность таймера
reg timer_irq;   // запрос прерывания
wire timer_istb; // строб запроса вектора
wire timer_iack=timer_istb;

// Прерывания 
wire [7:4] vstb;          // строб приема вектора
wire [7:4] virq;          // запрос прерывания
wire [15:0] cpu_pir_in;   // регистр программных прерываний
// линии запроса прерывания
assign virq[7]=irq_i[7];      // уровень kw11p
assign virq[6]=timer_irq; // уровень 6 - таймер
assign virq[5]=irq_i[5];  // уровень 5 - быстрая (блочная) периферия
assign virq[4]=irq_i[4];  // уровень 4 - медленная (байтовая) периферия
// запрос на прием вектора
assign istb_o[7] = vstb[7] ;
assign timer_istb = vstb[6] ;
assign istb_o[5] = vstb[5] ;
assign istb_o[4] = vstb[4] ;

// шина ввода вектора прерывания в процессор
wire [8:0] vector = (vstb[6])? 9'o100:             // таймер
                    ivec;                          // входной вектор от контроллеров прерывания  


wire [15:0] pc ;
wire [15:0] datapath ;
wire [1:0]	cpu_mode ;
wire [15:0]	startup_adr ;
wire			cons_start ;
wire			ctrl_slow ;

//******************************************
//*  Процессор PDP2011
//******************************************

cpu2011 cpu (
   .wbm_adr_o(cpu_adr),    // виртуальный адрес
   .wbm_dat_o(wb_dat_o),   // выход данных
   .wbm_dat_i(wb_mux),     // вход данных
   .wbm_we_o(wb_we_o),     // запись - передача данных из процессора на шину
   .wbm_sel_o(wb_sel_o),   // выбор байтов для записи           
   .wbm_stb_o(cpu_stb),    // строб данных
   .wbm_ack_i(cpu_ack),    // подтверждение обмена
   
   .cp(cpu_cp),            // 1 - обмен по шине происходит в предыдущем режиме процессора
   .ifetch(ifetch),        // признак цикла выборки инструкции
   .id(cpu_id),            // 0 - выборка инструкции, 1 - обращение к данным

   // прерывания
   .virq(virq),                // запросы прерывания
   .vector(vector),            // ввод адреса вектора
   .vstb(vstb),                // строб приема вектора
   .vack(iack_i | timer_iack), // подтверждение приема вектора
   .pirq(cpu_pir_in),          // Регистр PIRQ, формирователь программных прерываний
    
    // линии прерываний от MMU
   .mmutrap(mmutrap),          // запрос прерывания от MMU по окончании обработки текущей инструкции
   .ack_mmutrap(ack_mmutrap),  // подтверждение прерывания от MMU
   .mmuabort(mmuabort),        // запрос от MMU на отмену обработки текущей инструкции
   .ack_mmuabort(ack_mmuabort),// подтверждение отмены обработки инструкции
   
   // DMA
   .npr(dma_req),  // запрос DMA
   .npg(dma_ack),              // подтверждение DMA

   // Флаги ошибок и состояния процессора
   .oddabort(oddabort),           // словное обращение по нечетному адресу
   .illhalt(cpu_illegal_halt),    // признак недопустимости инструкции HALT в текущем режиме процессора
   .bus_timeout(bus_timeout),     // таймаут шины
   .ysv(cpu_ysv),                 // желтое состояние стека
   .rsv(cpu_rsv),                 // красное состояние стека
   .sr0_ic(sr0_ic),               // sr0/mmr0 IC - флаг завершения декодирования инструкции
   .sr1(sr1),                     // sr1/mmr1 - адрес текущей инструкции
   .sr2(sr2),                     // sr2/mmr2 - информация об автоинкременте/декременте регистров в текущей инструкции
   .dstfreference(dstfreference), // признак цикла обращения к операнду-приемнику
   .dw8(dw8),                     // признак байтового обмена

   .cpu_stack_limit(cpu_stack_limit), // значение нижней границы стека 

   // PSW
   .psw_in(cpu_psw_in),               // Ввод PSW , записываемого под адресу 177776
   .psw_in_we_even(cpu_psw_we_even),  // Разрешение записи младшего байта PSW
   .psw_in_we_odd(cpu_psw_we_odd),    // Разрешение записи старшего байта PSW
   .psw_out(cpu_psw_out),             // Вывод PSW , читаемого под адресу 177776

   // управление и индикация
   .sw_cont(bt_halt | cons_sigo[1]),  // сигнал продолжения работы
   .cpuslow(cpuslow | cons_sigo[2] | ctrl_slow),  // Включение замедленного режима процессора
   .iwait(iwait),                 // признак приостановки: 1 - процессор стоит на команде wait и ждет прерывания
   .run(run),                     // признак работы секвенсора
   
   // конфигурация
   .fpu_enable(`fpu_present),     // Признак наличия FPU в схеме процессора
   .startup_adr(startup_adr),      // Адрес старта по сбросу
   .startup_psw(16'o340),         // Стартовое PSW

   .clk(clk_p),              // тактовый синхросигнал
   .reset(dclo),             // сброс процессора
   .init(bus_reset),          // выход сброса для всей периферии на шине
	
	.pc(pc),
	.datapath(datapath),
	.cpu_halt(cons_sigo[3]),
	.cpu_mode(cpu_mode),
	.cons_start(cons_start)
);

//******************************************
//*  Управляющие регистры процессора
//******************************************
cpu_control_regs ccr(
   .wb_clk_i(clk_p),
   .wb_rst_i(sys_init),
   .wb_adr_i(wb_adr_o[4:0]),
   .wb_dat_i(wb_dat_o),
   .wb_dat_o(ccr_dat),
   .wb_cyc_i(1'b1),
   .wb_we_i(wb_we_o),
   .wb_stb_i(ccr_stb),
   .wb_ack_o(ccr_ack),
   .wb_sel_i(wb_sel_o),

   .psw_in(cpu_psw_in),                // Ввод PSW , записываемого под адресу 177776
   .psw_in_we_even(cpu_psw_we_even),   // Разрешение записи младшего байта PSW
   .psw_in_we_odd(cpu_psw_we_odd),     // Разрешение записи старшего байта PSW
   .psw_out(cpu_psw_out),              // Вывод PSW , читаемого под адресу 177776
   
   .cpu_stack_limit(cpu_stack_limit),  // нижняя граница стека, для схем контроля стека
   .pir_in(cpu_pir_in),                // сигналы программных прерываний
   
   // сигналы регистра ошибок
   .cpu_illegal_halt(cpu_illegal_halt), 
   .cpu_address_error(oddabort), 
   .cpu_nxm(bus_timeout & ram_stb), 
   .cpu_iobus_timeout(bus_timeout & ~ram_stb), 
   .cpu_ysv(cpu_ysv), 
   .cpu_rsv(cpu_rsv),
	
	.cpu_slow(ctrl_slow)
);

//*******************************************
//*  Диспетчер памяти
//*******************************************
wire dma_iopage_mapped ;

mmu mmu1(

   // Интерфейс к CPU
   .cpu_addr_v(cpu_adr),    // виртуальный 16-битный адрес
   .mmu_dat_o(mmu_dat),     // выход локальной шины данных
   .mmu_dat_i(wb_dat_o),    // вход локальной шины данных
   .cpu_we(wb_we_o),        // признак цикла записи на шину
   .cpu_sel(wb_sel_o),      // выбор байтов для записи
   .cpu_stb(cpu_stb),       // строб обмена по шине
   .cpu_ack(cpu_ack),       // подтверждение транзакции (REPLY)
   .cpu_cp(cpu_cp),         // 1 - обмен по шине происходит в предыдущем режиме процессора
   .psw(cpu_psw_out),       // текущий PSW
   .id(cpu_id),             // 0-обращение к инструкции, 1 - к данным
   
   // Линии прерываний
   .mmutrap(mmutrap),           // запрос прерывания от MMU по окончании обработки текущей инструкции
   .ack_mmutrap(ack_mmutrap),   // подтверждение
   .mmuabort(mmuabort),         // запрос от MMU на отмену обработки текущей инструкции
   .ack_mmuabort(ack_mmuabort), // подтверждение
   .mmuoddabort(mmuoddabort),   // прерывание из-за словного обращения по нечетному адресу
   
   // регистры MMR
   .sr0_ic(sr0_ic),             // регистр MMR0, бит 7 (IC) - флаг завершения декодирования инструкции
   .sr1_in(sr1),                // регистр MMR1 - информация об автоинкременте/автодекременте текущей инструкции
   .sr2_in(sr2),                // регистр MMR2 - виртуальный адрес текущей инструкции
   
   .dstfreference(dstfreference), // 1 - адрес приемника данных поностью сформирован
   .ifetch(ifetch),               // 1 - идет выборка инструкции
   .cpu_dw8(dw8),                 // 1- байтовый обмен
   
   // Интерфейс к внешней шине
   .mmu_adr_o(wb_adr_o),     // выходная полная 22-битная шина адреса
   .RAM_stb_o(ram_stb),      // строб обращения к памяти
   .BUS_stb_o(bus_stb),      // строб обращения к странице ввода-вывода
   .wb_ack_i(wb_ack),        // подтверждение от устройства
      
   // Шины DMA-режима
   .DMA_gnt(dma_ack),        // признак входа процессора в режим DMA
   .DMA_addr_i(dma_adr18),   // ввод 18-битного адреса в режиме DMA для устройств, работающих через UBM
   .DMA_stb_i(dma_stb),      // строб транзакции DMA для устройств, работающих через UBM

   .dma_iopage_mapped(dma_iopage_mapped),
   
   // индикаторы режима работы
   .cons_map18(cons_map18),  // режим 18-битного адреса 
   .cons_map22(cons_map22),  // режим 22-битного адреса
   .cons_id(cons_id),         // режим разделения I/D
   .cons_ubm(cons_ubm),       // режим Unibus Mapping
   
   .reset(sys_init),         // общий сброс
   .clk(clk_p)               // синхросигнал
);

// Console

	wire [15:0] cons_dat_o ;
	
	console cons(
		.clkcons(clkcons),
		.pc(pc),
		.datapath(datapath),
		.sigi({cpu_id, ~dma_ack, cpu_mode, cpuslow | cons_sigo[2] | ctrl_slow, cons_map22, cons_map18, iwait, run}),
		.sigo(cons_sigo),
		
		.clk_p(clk_p),
		.sys_init(sys_init),
		.wb_adr_i(wb_adr_o),
		.wb_dat_i(wb_dat_o),
		.wb_dat_o(cons_dat_o),
		.bus_stb(bus_stb),
		.wb_we_i(wb_we_o),
		.cons_ack(cons_ack),
		
		.dma_req(cons_dma_req),
		.dma_gnt(cons_dma_gnt),
		.dma_adr_o(cons_dma_adr_o),
		.dma_dat_i(cons_dma_dat_i),
		.dma_dat_o(cons_dma_dat_o),
		.dma_stb_o(cons_dma_stb_o),
		.dma_we_o(cons_dma_we_o),
		.dma_ack_i(global_ack),
		
		.startup_adr(startup_adr),
		.cons_start(cons_start),
		
		.cons_row(cons_row),
		.cons_col(cons_col),
		.cons_ledrow(cons_ledrow)
	) ;

//*******************************************
//* ПЗУ монитора-загрузчика
//*******************************************

// эмулятор пульта и набор загрузчиков m9312, состоит из 2 частей:
// - консоль, 165000-165777
// - загрузчики, 173000-173777
// Обе части лежат в одном блоке ПЗУ, консоль в младшей части, загрузчики в старшей.

`ifdef bootrom_module

wire bootrom0_sel;
wire bootrom1_sel;
wire bootrom_stb;
wire bootrom_ack;
wire [15:0] bootrom_dat;
reg [1:0]bootrom_ack_reg;

boot_rom bootrom(
   .address({bootrom1_sel, wb_adr_o[8:1]}),
   .clock(clk_p),
   .q(bootrom_dat));

// сигнал ответа
always @ (posedge clk_p) begin
   bootrom_ack_reg[0] <= bootrom_stb & ~wb_we_o;
   bootrom_ack_reg[1] <= bootrom_stb & bootrom_ack_reg[0] & ~wb_we_o;
end
assign bootrom_ack = bus_stb & bootrom_ack_reg[1];

// сигналы выбора частей ПЗУ
assign bootrom1_sel =  (wb_adr_o[15:9] == 7'o173);                // загрузчики, 173000-173776
assign bootrom0_sel =  (wb_adr_o[15:9] == 7'o165);                // консоль, 165000-165776
assign bootrom_stb  = bus_stb & (bootrom0_sel | bootrom1_sel);    // строб выбора ПЗУ

`endif

//*************************************************************************
//* Генератор прерываний от таймера
//* Сигнал имеет частоту 60 Гц и ширину импульса в 1 такт
//*************************************************************************
reg timer_60;
reg [21:0] timercnt;

wire [21:0] timer_limit = 31'd`clkref / 6'd60 -1'b1 ;

always @ (posedge clk_p) begin
  if (timercnt >= timer_limit) begin
     // заворот счетчика
     timercnt <= 22'd0;  // перезагрузка
     timer_60 <= 1'b1;   // формируем импульс таймера
  end  
  else begin
     // счет тактов
     timercnt <= timercnt + 1'b1; // счетчик++
     timer_60 <= 1'b0;            // завершаем импульс таймера
  end     
end

//************************************************
//* Сетевой таймер KW11-L 177546
//************************************************
reg tirq_prev_state;  // состояние таймера в предыдущем такте

// чтение регистра таймера
//                           7           6
assign kw11l_dat = {8'o0, timer_rdy, timer_ie, 6'o0};
assign led_timer=~timer_ie;

always @ (posedge clk_p) 
  // сброс системы
  if (sys_init == 1'b1) begin 
     timer_ie <= 1'b0;
     timer_rdy <= 1'b1;
     timer_irq <= 1'b0;
  end     
  else begin     
    // обмен с общей шиной
    if ((kw11l_stb == 1'b1) && (wb_we_o == 1'b1)) begin
        // запись
        timer_ie  <= wb_dat_o[6];
        timer_rdy <= wb_dat_o[7];
    end
    tirq_prev_state <= timer_60;   // сохранение предыдущего состояния сигнала
    // детектор перепадов сигнала таймера - только  0->1
    if ((tirq_prev_state != timer_60) && (timer_60 == 1'b1)) begin
        timer_rdy <= 1'b1;  // взводим сигнал готовности таймера
        // формирователь сигнала прерывания
        if (timer_ie) timer_irq <= 1'b1;
    end
     // формирователь вектора
     if (timer_irq == 1'b1) begin 
       if (~timer_ie | timer_istb) timer_irq <= 1'b0;
     end
  end     
  
// формирователь ответа       
wire kw11l_reply= kw11l_stb & ~kw11l_ack;
always @(posedge clk_p)
    if (sys_init == 1'b1) kw11l_ack <= 1'b0;
    else kw11l_ack <= kw11l_reply;


//*******************************************************************
//*  Формирователь сигналов выбора устройств на шине
//*******************************************************************

// стробы выбора периферии
assign kw11l_stb  = bus_stb & (wb_adr_o[15:1] == (16'o177546 >> 1));   // KW11-L - 177546
assign ccr_stb = bus_stb & (wb_adr_o[15:5] == 11'b11111111111);        // 177740 - 177777 (на полной шине - 777740-777777) - внутренние регистры процессора

// сигнал ответа
assign wb_ack     = global_ack 
						| ccr_ack
						| cons_ack
						| kw11l_ack
`ifdef bootrom_module
						| bootrom_ack
`endif
						;

// сборная шина входных данных к процессору
assign wb_mux     = mmu_dat | wb_dat_i
                  | (ccr_stb ? ccr_dat : 16'o000000)
                  | (kw11l_stb ? kw11l_dat : 16'o000000)
                  | (cons_ack ? cons_dat_o : 16'o000000) 
`ifdef bootrom_module
                  | (bootrom_stb ? bootrom_dat : 16'o000000)
`endif
						;

endmodule
