`define pdp2011_board
`define RK_module
`define TERMINAL_SPEED 3'd7
`define UART2SPEED 3'd7
`define cas_latency 2

`define bootrom_module

`ifdef pdp2011_board
    `define BOARD pdp2011       // имя подключаемого модуля процессорной платы
    `define adr22               // признак 22-битной процессорной платы
    `define PLL_MUL 1           // умножитель PLL
    `define PLL_DIV 1           // делитель PLL
    `define CPUSLOW 240         // число тактов, пропускаемых процессором в режиме замедления
    `define fpu_present 1'b1    // признак наличия FPP: 0-нет, 1-есть
    `define massbus             // признак наличия шины MASSBUS с 22-битной адресацией DMA
`endif

`include "../../hdl/common-config.v"
