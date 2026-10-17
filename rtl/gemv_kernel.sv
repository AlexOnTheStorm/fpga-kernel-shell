`timescale 1ns / 1ps
// =====================================================================
//  gemv_kernel — демо-ядро kshell: y = W·x (docs/design.md §5, §15).
//  W, x — INT8 со знаком; накопление и y — INT32 со знаком, по модулю 2^32.
//
//  Аргументы (защёлкиваются в такте k_start):
//    arg0 — адрес x в spm (слова), arg1 — адрес y в spm (слова),
//    arg2 — M (строк), arg3 — K (кратно 32). KW = K/32 beat'ов на строку.
//  Раскладка: x[k] — байт k%32 слова arg0 + k/32; W — поток строк, строка m
//  = KW beat'ов, элемент k — байт k%32 beat'а k/32; y[m] — 32-битная дорожка
//  m%8 слова arg1 + m/8 (запись по wstrb, остальные дорожки не трогаются).
//
//  Конвейер (beat принят в такте t → запись y его строки в такте t+5):
//    t   : handshake beat'а; в этом же такте чтение x[j] через порт B
//          (адрес — счётчик xaddr_q, «на такт вперёд» относительно умножения);
//    t+1 : w_q (регистр beat'а) и k_spm_rdata (регистр spm) → 32 умножения
//          INT8×INT8 → prod_q (16 бит);
//    t+2 : дерево, ступень A: 32 → 8 (2 уровня сумматоров, 18 бит) → ps_q;
//    t+3 : дерево, ступень B: 8 → 1 (3 уровня, 21 бит) → dot_q;
//    t+4 : аккумулятор строки acc_q (32 бит, первый beat строки — загрузка);
//          на последнем beat'е строки yv_q = 1;
//    t+5 : yv_q → запись acc_q в spm через порт B.
//  Глубина 4 регистра от входа умножителя до аккумулятора: регистровые входы
//  умножителей (w_q и выход BRAM) и ≤ 3 уровня сумматоров на ступень — путь
//  укладывается в DSP/carry-chain без ретайминга; аккумулятор — отдельная
//  ступень (32-битная carry-chain). Регистры датапата resetless, управление
//  (ctl-флаги ступеней, FSM, счётчики) — с синхронным сбросом k_rst_n.
//
//  Порт B делят чтение x (на каждый handshake) и запись y: в такте записи
//  (yv_q) tready = 0 → одна пауза на строку, установившийся темп
//  KW / (KW + 1) beat'а за такт при непрерывном потоке. tready зависит
//  только от регистров (state_q, yv_q) — нет комбинаторного пути tvalid → tready.
//
//  FSM: IDLE → RUN (приём M·KW beat'ов) → FLUSH (слив конвейера до записи
//  последнего y) → IDLE + импульс k_done в такте после записи последнего y.
//  k_err (импульс, без k_done, ядро → IDLE; конвейер гасится в такте нарушения,
//  так что с такта k_err порт B молчит — недописанные y не пишутся; §5.5):
//    - tlast не на beat'е M·KW или его нет на этом beat'е;
//    - аргументы вне контракта: M = 0, K = 0 или K не кратно 32.
//  Вне scope (документировано): адреса x/y заворачиваются по модулю
//  SPM_WORDS; пересечение областей x и y — ответственность программы;
//  k_start вне IDLE игнорируется (обёртка его не выдаёт, §5.4).
// =====================================================================
module gemv_kernel
  import kshell_pkg::*;
(
    input  logic                   clk,
    input  logic                   k_rst_n,
    input  logic                   k_start,
    input  logic [      3:0][31:0] k_arg,
    output logic                   k_done,
    output logic                   k_err,
    input  logic                   k_s_tvalid,
    output logic                   k_s_tready,
    input  logic [   DW-1:0]       k_s_tdata,
    input  logic                   k_s_tlast,
    output logic                   k_spm_en,
    output logic                   k_spm_we,
    output logic [SPMAW-1:0]       k_spm_addr,
    output logic [   DW-1:0]       k_spm_wdata,
    output logic [   BB-1:0]       k_spm_wstrb,
    input  logic [   DW-1:0]       k_spm_rdata
);

  localparam int LANES = BB;  // INT8-элементов на beat
  localparam int EW = 8;  // ширина элемента
  localparam int PW = 2 * EW;  // произведение INT8×INT8
  localparam int G1 = 4;  // слагаемых на сумматор ступени A
  localparam int N1 = LANES / G1;  // частичных сумм после ступени A
  localparam int S1W = PW + 2;  // сумма 4 произведений
  localparam int DOTW = S1W + 3;  // сумма 8 частичных = 32 произведений
  localparam int YW = 32;  // y — INT32
  localparam int YLANES = DW / YW;  // y на слово spm
  localparam int YLW = $clog2(YLANES);
  localparam int KWW = 27;  // ширина K/32 (arg3[31:5])

  // дерево ниже написано явно под 32 дорожки (DW = 256)
  if (LANES != 32) begin : g_bad_dw
    $error("gemv_kernel: adder tree is written for 32 INT8 lanes (DW = 256)");
  end

  typedef enum logic [1:0] {
    S_IDLE,
    S_RUN,
    S_FLUSH
  } gv_state_e;

  // управляющие флаги beat'а, едут по конвейеру вместе с данными
  typedef struct packed {
    logic v;      // ступень несёт beat
    logic first;  // первый beat строки (загрузка аккумулятора)
    logic rend;   // последний beat строки (запись y)
    logic fin;    // последний beat матрицы (k_done после записи)
  } ctl_t;

  gv_state_e state_q, state_d;

  // аргументы RUN
  logic [SPMAW-1:0] x_base_q;
  logic [KWW-1:0] kw_m1_q;
  logic [31:0] m_m1_q;
  logic bad_args, start;

  // входная сторона: позиция принимаемого beat'а
  logic [KWW-1:0] j_q;
  logic [31:0] row_q;
  logic [SPMAW-1:0] xaddr_q;
  logic hs, row_end, mat_end, bad;

  // конвейер
  ctl_t ctl1_q, ctl2_q, ctl3_q, ctl4_q;
  logic [DW-1:0] w_q;
  logic signed [PW-1:0] prod_q[LANES];
  logic signed [S1W-1:0] ps_q[N1];
  logic signed [DOTW-1:0] dot_q;
  logic [YW-1:0] acc_q;
  logic yv_q, yfin_q, ywr;

  // выходная сторона: куда пишется следующий y
  logic [SPMAW-1:0] yaddr_q;
  logic [  YLW-1:0] ylane_q;

  assign hs = k_s_tvalid && k_s_tready;
  assign row_end = (j_q == kw_m1_q);
  assign mat_end = row_end && (row_q == m_m1_q);
  // tlast раньше последнего beat'а матрицы или его нет на последнем
  assign bad = hs && (k_s_tlast != mat_end);
  assign bad_args = (k_arg[2] == 32'd0) || (k_arg[3][31:5] == 27'd0) || (k_arg[3][4:0] != 5'd0);
  assign ywr = yv_q && k_rst_n;
  // запуск принимается только в IDLE (вне IDLE обёртка k_start не выдаёт, §5.4)
  assign start = k_start && (state_q == S_IDLE);

  // ------------------------------------------------------------ FSM

  always_comb begin
    state_d    = state_q;
    k_s_tready = 1'b0;
    case (state_q)
      S_IDLE:  if (start && !bad_args) state_d = S_RUN;
      S_RUN: begin
        // такт записи y занимает порт B → пауза приёма
        k_s_tready = k_rst_n && !yv_q;
        if (bad) state_d = S_IDLE;
        else if (hs && mat_end) state_d = S_FLUSH;
      end
      S_FLUSH: if (ywr && yfin_q) state_d = S_IDLE;
      default: state_d = S_IDLE;
    endcase
  end

  always_ff @(posedge clk) begin
    if (!k_rst_n) state_q <= S_IDLE;
    else state_q <= state_d;
  end

  always_ff @(posedge clk) begin
    if (!k_rst_n) begin
      k_done <= 1'b0;
      k_err  <= 1'b0;
    end else begin
      k_done <= (state_q == S_FLUSH) && yv_q && yfin_q;
      k_err  <= bad || (start && bad_args);
    end
  end

  // ------------------------------------------------------------ аргументы

  // датапат: читаются только после k_start
  always_ff @(posedge clk) begin
    if (start) begin
      x_base_q <= k_arg[0][SPMAW-1:0];
      kw_m1_q  <= k_arg[3][31:5] - 27'd1;
      m_m1_q   <= k_arg[2] - 32'd1;
    end
  end

  // ------------------------------------------------------------ входная сторона

  always_ff @(posedge clk) begin
    if (!k_rst_n || start) j_q <= '0;
    else if (hs) j_q <= row_end ? '0 : j_q + 27'd1;
  end

  always_ff @(posedge clk) begin
    if (!k_rst_n || start) row_q <= 32'd0;
    else if (hs && row_end) row_q <= row_q + 32'd1;
  end

  // адрес x[j]: чтение в такте handshake, данные — к ступени умножения
  always_ff @(posedge clk) begin
    if (start) xaddr_q <= k_arg[0][SPMAW-1:0];
    else if (hs) xaddr_q <= row_end ? x_base_q : xaddr_q + SPMAW'(1);
  end

  // ------------------------------------------------------------ конвейер: управление

  // при k_err конвейер гасится: незавершённые строки не пишутся
  always_ff @(posedge clk) begin
    if (!k_rst_n || bad) begin
      ctl1_q <= '0;
      ctl2_q <= '0;
      ctl3_q <= '0;
      ctl4_q <= '0;
    end else begin
      ctl1_q <= '{v: hs, first: (j_q == '0), rend: row_end, fin: mat_end};
      ctl2_q <= ctl1_q;
      ctl3_q <= ctl2_q;
      ctl4_q <= ctl3_q;
    end
  end

  always_ff @(posedge clk) begin
    if (!k_rst_n || bad) begin
      yv_q   <= 1'b0;
      yfin_q <= 1'b0;
    end else begin
      yv_q   <= ctl4_q.v && ctl4_q.rend;
      yfin_q <= ctl4_q.v && ctl4_q.fin;
    end
  end

  // ------------------------------------------------------------ конвейер: датапат

  always_ff @(posedge clk) begin
    w_q <= k_s_tdata;
  end

  // t+1: 32 умножения INT8×INT8 (x — регистр чтения spm, пришёл вместе с w_q)
  always_ff @(posedge clk) begin
    for (int i = 0; i < LANES; i++) begin
      prod_q[i] <= $signed(w_q[EW*i+:EW]) * $signed(k_spm_rdata[EW*i+:EW]);
    end
  end

  // t+2: ступень A — суммы четвёрок (2 уровня)
  always_ff @(posedge clk) begin
    for (int g = 0; g < N1; g++) begin
      ps_q[g] <= (S1W'(prod_q[G1*g]) + S1W'(prod_q[G1*g+1]))
               + (S1W'(prod_q[G1*g+2]) + S1W'(prod_q[G1*g+3]));
    end
  end

  // t+3: ступень B — сбалансированное дерево 8 → 1 (3 уровня)
  always_ff @(posedge clk) begin
    dot_q <= ((DOTW'(ps_q[0]) + DOTW'(ps_q[1])) + (DOTW'(ps_q[2]) + DOTW'(ps_q[3])))
           + ((DOTW'(ps_q[4]) + DOTW'(ps_q[5])) + (DOTW'(ps_q[6]) + DOTW'(ps_q[7])));
  end

  // t+4: аккумулятор строки; переполнение INT32 — по модулю 2^32
  always_ff @(posedge clk) begin
    if (ctl4_q.v) begin
      acc_q <= (ctl4_q.first ? '0 : acc_q) + {{(YW - DOTW) {dot_q[DOTW-1]}}, dot_q};
    end
  end

  // ------------------------------------------------------------ выходная сторона

  always_ff @(posedge clk) begin
    if (start) begin
      yaddr_q <= k_arg[1][SPMAW-1:0];
      ylane_q <= '0;
    end else if (ywr) begin
      ylane_q <= ylane_q + YLW'(1);
      if (ylane_q == YLW'(YLANES - 1)) yaddr_q <= yaddr_q + SPMAW'(1);
    end
  end

  // порт B: запись y (такт yv_q, tready = 0) или чтение x (такт handshake)
  always_comb begin
    k_spm_en    = hs || ywr;
    k_spm_we    = ywr;
    k_spm_addr  = ywr ? yaddr_q : xaddr_q;
    // y во все дорожки, wstrb выбирает одну — без сдвигателя данных
    k_spm_wdata = {YLANES{acc_q}};
    for (int l = 0; l < YLANES; l++) begin
      k_spm_wstrb[(YW/8)*l+:(YW/8)] = {(YW / 8) {ylane_q == YLW'(l)}};
    end
  end

endmodule
