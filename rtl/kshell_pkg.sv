`timescale 1ns / 1ps
// =====================================================================
//  kshell_pkg — параметры и перечисления универсальной обёртки ядра.
//  Значения по умолчанию — docs/design.md §3. Компилируется ПЕРВЫМ.
// =====================================================================
package kshell_pkg;

  localparam int DW = 256;  // ширина данных AXI4 MM / AXIS / scratchpad
  localparam int AW = 40;  // ширина адреса AXI4 MM
  localparam int BURST = 8;  // beat'ов в burst'е
  localparam int WR_OUT = 8;  // максимум burst'ов записи в полёте
  localparam int SPM_WORDS = 1024;  // слов scratchpad по DW бит
  localparam int NSEM = 8;  // число семафоров
  localparam int CQ_DEPTH = 16;  // глубина командной FIFO
  localparam int EQ_DEPTH = 4;  // глубина очереди каждого движка

  localparam int BB = DW / 8;  // байт на beat
  localparam int BURST_BYTES = BURST * BB;  // байт на burst
  localparam int SPMAW = $clog2(SPM_WORDS);  // ширина адреса scratchpad
  localparam int SEMW = $clog2(NSEM);  // ширина индекса семафора
  localparam int NBW = 24;  // ширина числа burst'ов в команде

  typedef enum logic [3:0] {
    OP_LOAD   = 4'd1,
    OP_STREAM = 4'd2,
    OP_STORE  = 4'd3,
    OP_WAIT   = 4'd4,
    OP_SIGNAL = 4'd5,
    OP_RUN    = 4'd6
  } op_e;

  typedef enum logic [3:0] {
    E_NONE    = 4'd0,
    E_RRESP   = 4'd1,
    E_BRESP   = 4'd2,
    E_ALIGN   = 4'd3,
    E_RANGE   = 4'd4,
    E_OPCODE  = 4'd5,
    E_KERNEL  = 4'd6,
    E_TIMEOUT = 4'd7
  } err_e;

endpackage
