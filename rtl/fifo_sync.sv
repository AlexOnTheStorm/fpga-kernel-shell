`timescale 1ns / 1ps
// =====================================================================
//  fifo_sync — синхронная FIFO на распределённой памяти (LUTRAM).
//  Нулевая латентность чтения: out_d = элемент головы (асинхронное
//  чтение массива), out_v = !empty. in_rdy = !full (не зависит от
//  out_rdy — нет комбинаторного пути выход→вход). clr — синхронная
//  очистка, приоритетнее push/pop. Память resetless (датапат).
// =====================================================================
module fifo_sync #(
    parameter int W     = 8,
    parameter int DEPTH = 4
) (
    input  logic                       clk,
    input  logic                       rst_n,
    input  logic                       clr,      // синхронная очистка
    input  logic                       in_v,
    output logic                       in_rdy,
    input  logic [              W-1:0] in_d,
    output logic                       out_v,
    input  logic                       out_rdy,
    output logic [              W-1:0] out_d,
    output logic [$clog2(DEPTH+1)-1:0] count,
    output logic                       empty
);

  localparam int PW = (DEPTH > 1) ? $clog2(DEPTH) : 1;
  localparam int CW = $clog2(DEPTH + 1);
  localparam logic [PW-1:0] LAST_PTR = PW'(DEPTH - 1);
  localparam logic [CW-1:0] FULL_CNT = CW'(DEPTH);

  logic [W-1:0] mem_q[DEPTH];
  logic [PW-1:0] wr_ptr_q, rd_ptr_q;

  logic push, pop;

  assign empty  = (count == '0);
  assign in_rdy = (count != FULL_CNT);
  assign out_v  = !empty;
  assign out_d  = mem_q[rd_ptr_q];
  assign push   = in_v && in_rdy;
  assign pop    = out_v && out_rdy;

  // данные: запись по wr_ptr
  always_ff @(posedge clk) begin
    if (push) mem_q[wr_ptr_q] <= in_d;
  end

  // указатель записи
  always_ff @(posedge clk) begin
    if (!rst_n || clr) wr_ptr_q <= '0;
    else if (push) wr_ptr_q <= (wr_ptr_q == LAST_PTR) ? '0 : wr_ptr_q + PW'(1);
  end

  // указатель чтения
  always_ff @(posedge clk) begin
    if (!rst_n || clr) rd_ptr_q <= '0;
    else if (pop) rd_ptr_q <= (rd_ptr_q == LAST_PTR) ? '0 : rd_ptr_q + PW'(1);
  end

  // счётчик заполнения
  always_ff @(posedge clk) begin
    if (!rst_n || clr) count <= '0;
    else if (push && !pop) count <= count + CW'(1);
    else if (!push && pop) count <= count - CW'(1);
  end

endmodule
