`timescale 1ns / 1ps
// =====================================================================
//  axil_regs — AXI4-Lite slave и регистры хоста kshell (docs/design.md §4, §6, §13).
//
//  Запись: AW и W принимаются НЕЗАВИСИМО (в любом порядке или вместе) в
//  однословные буферы; запись выполняется (wr_fire), когда есть оба и B не
//  висит. Одна запись в работе: пока буфер занят, его READY = 0. BRESP = OKAY.
//  Чтение: одна транзакция в работе, мультиплексор → регистр RDATA на
//  handshake AR; незанятые и WO-адреса читаются 0, RRESP = OKAY.
//  wstrb учитывается для CMD0..3 и SEM (побайтовое слияние); CTRL — по
//  байту 0; DOORBELL — любая запись.
//
//  Выходы-импульсы (cq_push, sem_we, soft_rst_req, perf_snap) — комбинаторные
//  от регистров буфера в такте wr_fire (на такт раньше, чем регистровые).
//  CTRL: [0] ENABLE (хранится), [1] SOFT_RESET, [2] PERF_SNAP (самосброс:
//  только импульс, читаются 0). SOFT_RESET применяет kshell (только при
//  AXI_QUIET); здесь он лишь запрос.
//  DOORBELL при полной FIFO: cq_push всё равно выдаётся — kshell ставит
//  cq_ovf, а fifo_sync не принимает (in_rdy = 0). Вход cq_full (интерфейс
//  плана) здесь не нужен: решение о переполнении целиком в kshell.
// =====================================================================
module axil_regs
  import kshell_pkg::*;
(
    input logic clk,
    input logic rst_n,

    // AXI4-Lite slave: 12 бит адреса, 32 бита данных
    input  logic        s_axil_awvalid,
    output logic        s_axil_awready,
    input  logic [11:0] s_axil_awaddr,
    input  logic        s_axil_wvalid,
    output logic        s_axil_wready,
    input  logic [31:0] s_axil_wdata,
    input  logic [ 3:0] s_axil_wstrb,
    output logic        s_axil_bvalid,
    input  logic        s_axil_bready,
    output logic [ 1:0] s_axil_bresp,
    input  logic        s_axil_arvalid,
    output logic        s_axil_arready,
    input  logic [11:0] s_axil_araddr,
    output logic        s_axil_rvalid,
    input  logic        s_axil_rready,
    output logic [31:0] s_axil_rdata,
    output logic [ 1:0] s_axil_rresp,

    // управление
    output logic enable,
    output logic soft_rst_req,
    output logic perf_snap,

    // командная FIFO: дескриптор {CMD3, CMD2, CMD1, CMD0}
    output logic         cq_push,
    output logic [127:0] cq_d,
    input  logic         cq_full,

    // запись семафора хостом (приоритетнее приращений, §11)
    output logic            sem_we,
    output logic [SEMW-1:0] sem_idx,
    output logic [    31:0] sem_wval,

    // чтение
    input logic [           31:0]       status,
    input logic [            3:0]       err_code,
    input logic [           31:0]       err_cmd,
    input logic [           31:0]       cmd_count,
    input logic [       NSEM-1:0][31:0] sem_val,
    input logic [            7:0][31:0] perf
);

  // адреса §13 (в словах: addr[11:2])
  localparam logic [9:0] A_CTRL = 10'h000;
  localparam logic [9:0] A_STATUS = 10'h001;
  localparam logic [9:0] A_ERR_CODE = 10'h002;
  localparam logic [9:0] A_ERR_CMD = 10'h003;
  localparam logic [9:0] A_CMD0 = 10'h004;  // CMD0..CMD3: 0x010..0x01C
  localparam logic [9:0] A_DOORBELL = 10'h008;
  localparam logic [9:0] A_CMD_COUNT = 10'h009;
  // блоки по 8 слов: SEM 0x040..0x05C, PERF 0x080..0x09C (addr[11:5])
  localparam logic [6:0] B_SEM = 7'h02;
  localparam logic [6:0] B_PERF = 7'h04;
  localparam logic [1:0] RESP_OKAY = 2'b00;
  localparam int CTRL_ENABLE = 0;
  localparam int CTRL_SOFT_RESET = 1;
  localparam int CTRL_PERF_SNAP = 2;

  // слияние слова по байтовой маске AXI
  function automatic logic [31:0] merge(input logic [31:0] old, input logic [31:0] wd,
                                        input logic [3:0] strb);
    for (int i = 0; i < 4; i++) begin
      if (strb[i]) old[8*i+:8] = wd[8*i+:8];
    end
    return old;
  endfunction

  // ------------------------------------------------------------ запись

  logic aw_v_q, w_v_q, bvalid_q;
  logic [11:0] awaddr_q;
  logic [31:0] wdata_q;
  logic [3:0] wstrb_q;
  logic aw_hs, w_hs, wr_fire;
  logic [9:0] wa;

  assign s_axil_awready = !aw_v_q;
  assign s_axil_wready  = !w_v_q;
  assign aw_hs          = s_axil_awvalid && s_axil_awready;
  assign w_hs           = s_axil_wvalid && s_axil_wready;
  // оба буфера полны и предыдущий B снят
  assign wr_fire        = aw_v_q && w_v_q && !bvalid_q;
  assign wa             = awaddr_q[11:2];

  always_ff @(posedge clk) begin
    if (!rst_n) aw_v_q <= 1'b0;
    else if (aw_hs) aw_v_q <= 1'b1;
    else if (wr_fire) aw_v_q <= 1'b0;
  end

  always_ff @(posedge clk) begin
    if (!rst_n) w_v_q <= 1'b0;
    else if (w_hs) w_v_q <= 1'b1;
    else if (wr_fire) w_v_q <= 1'b0;
  end

  // буферы адреса/данных — датапат (читаются только при aw_v_q/w_v_q)
  always_ff @(posedge clk) begin
    if (aw_hs) awaddr_q <= s_axil_awaddr;
  end

  always_ff @(posedge clk) begin
    if (w_hs) begin
      wdata_q <= s_axil_wdata;
      wstrb_q <= s_axil_wstrb;
    end
  end

  always_ff @(posedge clk) begin
    if (!rst_n) bvalid_q <= 1'b0;
    else if (wr_fire) bvalid_q <= 1'b1;
    else if (s_axil_bready) bvalid_q <= 1'b0;
  end

  assign s_axil_bvalid = bvalid_q;
  assign s_axil_bresp  = RESP_OKAY;

  // CTRL
  logic ctrl_wr;
  assign ctrl_wr = wr_fire && (wa == A_CTRL) && wstrb_q[0];

  always_ff @(posedge clk) begin
    if (!rst_n) enable <= 1'b0;
    else if (ctrl_wr) enable <= wdata_q[CTRL_ENABLE];
  end

  assign soft_rst_req = ctrl_wr && wdata_q[CTRL_SOFT_RESET];
  assign perf_snap    = ctrl_wr && wdata_q[CTRL_PERF_SNAP];

  // CMD0..CMD3 (WO) — датапат: хост пишет все четыре перед DOORBELL
  logic [3:0][31:0] cmd_q;

  for (genvar k = 0; k < 4; k++) begin : g_cmd
    always_ff @(posedge clk) begin
      if (wr_fire && (wa == A_CMD0 + 10'(k))) cmd_q[k] <= merge(cmd_q[k], wdata_q, wstrb_q);
    end
  end

  assign cq_d     = cmd_q;
  assign cq_push  = wr_fire && (wa == A_DOORBELL);

  // SEM[i]: слияние по маске с текущим значением
  assign sem_we   = wr_fire && (wa[9:3] == B_SEM);
  assign sem_idx  = wa[SEMW-1:0];
  assign sem_wval = merge(sem_val[sem_idx], wdata_q, wstrb_q);

  // ------------------------------------------------------------ чтение

  logic rvalid_q;
  logic [31:0] rdata_q, rd_mux;
  logic [9:0] ra;
  logic ar_hs;

  assign ra             = s_axil_araddr[11:2];
  assign s_axil_arready = !rvalid_q;
  assign ar_hs          = s_axil_arvalid && s_axil_arready;

  always_comb begin
    rd_mux = 32'd0;
    case (ra)
      A_CTRL:      rd_mux = {31'd0, enable};
      A_STATUS:    rd_mux = status;
      A_ERR_CODE:  rd_mux = {28'd0, err_code};
      A_ERR_CMD:   rd_mux = err_cmd;
      A_CMD_COUNT: rd_mux = cmd_count;
      default: begin
        if (ra[9:3] == B_SEM) rd_mux = sem_val[ra[SEMW-1:0]];
        else if (ra[9:3] == B_PERF) rd_mux = perf[ra[2:0]];
      end
    endcase
  end

  always_ff @(posedge clk) begin
    if (!rst_n) rvalid_q <= 1'b0;
    else if (ar_hs) rvalid_q <= 1'b1;
    else if (s_axil_rready) rvalid_q <= 1'b0;
  end

  always_ff @(posedge clk) begin
    if (ar_hs) rdata_q <= rd_mux;
  end

  assign s_axil_rvalid = rvalid_q;
  assign s_axil_rdata  = rdata_q;
  assign s_axil_rresp  = RESP_OKAY;

endmodule
