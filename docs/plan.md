# kshell — план реализации

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** реализовать универсальную обёртку ядра `kshell` и демо-ядро `gemv_kernel` по спецификации, с проверкой в cocotb + Verilator, SVA, покрытием и демо-отчётом для интервью.

**Architecture:** обёртка из небольших модулей (по файлу на модуль), соединённых в `kshell`: регистры AXI-Lite → командная FIFO → секвенсор → три очереди движков (rd, wr, ядро) → rd_dma / wr_dma / контроллер ядра; scratchpad и семафоры общие. Тесты: модульные для примитивов и движков, системные на `kshell_top` с Python-моделями памяти (OoO по ID) и хоста.

**Tech Stack:** SystemVerilog (Verilator 5.050), cocotb 2.0.1 в `~/fpga-venv`, Python 3 + numpy, Verible lint.

**Spec:** `docs/design.md` (ревизия 2). Номера разделов `§N` ниже — оттуда. Поведение, не расписанное в задаче, берётся из спецификации буквально.

**Как читать задачи.** Каждая задача: файлы → интерфейсы (точные порты) → тест, который сначала падает → реализация → тест проходит → lint → коммит. Код тестов дан полностью для ключевых проверок. RTL-модули описаны точными портами и требованиями со ссылками на спецификацию; сам RTL пишется при выполнении задачи под гейтами lint + тест + SVA (так же, как в tick_to_trade).

## Global Constraints

- Стиль: `docs/style_notes.md` — синхронный сброс `rst_n` (активный низкий), датапат без сброса, двухпроцессные FSM с `case … default`, `always_ff` на 1–3 связанных регистра, явные ширины, `localparam` в UPPER_SNAKE_CASE.
- Один модуль на файл; SVA — `rtl/<mod>_sva.sv` через `bind`, покрытие — `rtl/<mod>_cov.sv` через `bind`.
- `make lint` чистый перед каждым коммитом с RTL.
- Комбинаторные петли запрещены (`-Werror-UNOPTFLAT`).
- Параметры по умолчанию — §3: `DW=256, AW=40, NTAG=16, BURST=8, WR_OUT=8, SPM_WORDS=1024, NSEM=8, CQ_DEPTH=16, EQ_DEPTH=4, WDOG_CYCLES=2**20`.
- Порты AXI4 master во всех модулях называются `m_axi_<канал><сигнал>` (`m_axi_arvalid`, `m_axi_rdata` …), AXI-Lite — `s_axil_<…>`, поток к ядру — `k_s_t<…>`: одна Python-модель обслуживает и модульные, и системные тесты.
- cocotb 2.x: `Clock(dut.clk, 10, unit="ns")`; сэмплинг handshake — `await ReadOnly()` перед фронтом.
- Все числа в README — из симуляции, с пометкой «симуляция с моделью памяти, не кремний».
- Коммиты — в репозиторий `practice/kernel_shell`, с подписью `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.

---

## Карта файлов

```
kernel_shell/
  .gitignore                  sim_build/, results.xml, *.vcd, *.fst, coverage.dat, annot/, __pycache__/
  .rules.verible              копия из tick_to_trade
  README.md                   задача 12
  docs/design.md              спецификация (есть)
  docs/plan.md                этот план
  docs/style_notes.md         копия из tick_to_trade
  scripts/Makefile            копия из tick_to_trade + PKG и SRCS списком
  scripts/run.sh              копия из tick_to_trade
  scripts/demo.sh             задача 12
  rtl/kshell_pkg.sv           параметры, коды команд и ошибок
  rtl/fifo_sync.sv            синхронная FIFO (командная, очереди движков, W-FIFO)
  rtl/sem_file.sv             семафоры
  rtl/spm.sv                  scratchpad + арбитр порта A
  rtl/rd_dma.sv  rd_dma_sva.sv  rd_dma_cov.sv
  rtl/wr_dma.sv  wr_dma_sva.sv
  rtl/cmd_seq.sv cmd_seq_cov.sv
  rtl/axil_regs.sv
  rtl/kshell_perf.sv
  rtl/kshell.sv  kshell_sva.sv  kshell_cov.sv
  rtl/kshell_kif_sva.sv       контракт ядра (bind на модуль ядра)
  rtl/gemv_kernel.sv
  rtl/kshell_top.sv           kshell + gemv_kernel
  tb/sink_kernel.sv           тестовое ядро: поток → spm
  tb/kshell_sink_top.sv       kshell + sink_kernel (для T2)
  tb/axil.py                  драйвер AXI4-Lite
  tb/hbm_model.py             AXI4 slave: память, псевдоканалы, OoO, ошибки
  tb/isa.py                   кодировщик команд, Program, check_program
  tb/shell.py                 помощники: reset, submit, wait_sem, perf
  tb/test_isa.py              pytest для isa.py
  tb/test_prims.py            fifo_sync, sem_file, spm
  tb/test_rd_dma.py
  tb/test_wr_dma.py
  tb/test_cmd_seq.py
  tb/test_gemv.py             ядро отдельно
  tb/test_system.py           T1–T9 на kshell_top / kshell_sink_top
```

---

### Task 1: каркас репозитория, пакет, примитивы

**Files:**
- Create: `.gitignore`, `.rules.verible`, `docs/style_notes.md`, `scripts/Makefile`, `scripts/run.sh` (копии из `../tick_to_trade/` с правками ниже)
- Create: `rtl/kshell_pkg.sv`, `rtl/fifo_sync.sv`, `rtl/sem_file.sv`, `rtl/spm.sv`
- Test: `tb/test_prims.py`

**Interfaces (Produces):**

```systemverilog
package kshell_pkg;
  localparam int DW = 256, AW = 40, BURST = 8, WR_OUT = 8;
  localparam int SPM_WORDS = 1024, NSEM = 8, CQ_DEPTH = 16, EQ_DEPTH = 4;
  localparam int BB = DW / 8, BURST_BYTES = BURST * BB;
  localparam int SPMAW = $clog2(SPM_WORDS), SEMW = $clog2(NSEM);
  localparam int NBW = 24;                      // ширина числа burst'ов в команде
  typedef enum logic [3:0] {OP_LOAD = 4'd1, OP_STREAM = 4'd2, OP_STORE = 4'd3,
                            OP_WAIT = 4'd4, OP_SIGNAL = 4'd5, OP_RUN = 4'd6} op_e;
  typedef enum logic [3:0] {E_NONE = 4'd0, E_RRESP = 4'd1, E_BRESP = 4'd2, E_ALIGN = 4'd3,
                            E_RANGE = 4'd4, E_OPCODE = 4'd5, E_KERNEL = 4'd6,
                            E_TIMEOUT = 4'd7} err_e;
endpackage

module fifo_sync #(parameter int W = 8, parameter int DEPTH = 4) (
  input  logic clk, input logic rst_n, input logic clr,     // clr: синхронная очистка
  input  logic in_v, output logic in_rdy, input logic [W-1:0] in_d,
  output logic out_v, input logic out_rdy, output logic [W-1:0] out_d,
  output logic [$clog2(DEPTH+1)-1:0] count, output logic empty);

module sem_file (
  input  logic clk, input logic rst_n, input logic clr,
  input  logic inc_rd_v,  input logic [kshell_pkg::SEMW-1:0] inc_rd_idx,
  input  logic inc_wr_v,  input logic [kshell_pkg::SEMW-1:0] inc_wr_idx,
  input  logic inc_k_v,   input logic [kshell_pkg::SEMW-1:0] inc_k_idx,
  input  logic inc_sig_v, input logic [kshell_pkg::SEMW-1:0] inc_sig_idx, input logic [31:0] inc_sig_val,
  input  logic host_we,   input logic [kshell_pkg::SEMW-1:0] host_idx,    input logic [31:0] host_val,
  output logic [kshell_pkg::NSEM-1:0][31:0] sem_val);

module spm (
  input  logic clk,
  // порт A: запись rd_dma (абсолютный приоритет)
  input  logic a_wr_v, input logic [kshell_pkg::SPMAW-1:0] a_wr_addr, input logic [kshell_pkg::DW-1:0] a_wr_data,
  // порт A: чтение wr_dma — выполняется только в такт без a_wr_v
  input  logic a_rd_req, input logic [kshell_pkg::SPMAW-1:0] a_rd_addr,
  output logic a_rd_gnt,                 // = a_rd_req && !a_wr_v (комбинаторно)
  output logic [kshell_pkg::DW-1:0] a_rd_data,   // данные через 1 такт после gnt
  // порт B: ядро
  input  logic b_en, input logic b_we, input logic [kshell_pkg::SPMAW-1:0] b_addr,
  input  logic [kshell_pkg::DW-1:0] b_wdata, input logic [kshell_pkg::BB-1:0] b_wstrb,
  output logic [kshell_pkg::DW-1:0] b_rdata);    // через 1 такт после b_en && !b_we
```

Требования: `sem_file` — §11 (сумма приращений за такт, запись хоста приоритетна, `clr` обнуляет); `spm` — §10; `fifo_sync` — FIFO с регистровым выходом не обязательна: `out_d` = элемент головы (LUTRAM, асинхронное чтение), `in_rdy = !full`.

**Правки Makefile** относительно tick_to_trade: добавить `PKG := $(CURDIR)/../rtl/kshell_pkg.sv` первым в `VERILOG_SOURCES`; для lint — `RTL_DESIGN` начинается с `$(PKG)`; `PYTHONPATH` = `tb/`; добавить `SVA_EXTRA ?=` (дополнительные bind-файлы, например `kshell_kif_sva.sv`) в ветку `--assert`.

- [ ] **Step 1: скопировать инфраструктуру**

```bash
cd practice/kernel_shell
cp ../tick_to_trade/.rules.verible . && cp ../tick_to_trade/docs/style_notes.md docs/
mkdir -p rtl tb scripts && cp ../tick_to_trade/scripts/{Makefile,run.sh} scripts/
printf 'sim_build/\nresults.xml\n*.vcd\n*.fst\ncoverage.dat\nannot/\n__pycache__/\n' > .gitignore
```

Затем внести правки Makefile (см. выше).

- [ ] **Step 2: написать тест примитивов (падает: модулей нет)**

```python
# tb/test_prims.py
import os, random
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly

async def reset(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst_n.value = 0
    for s in ("clr",):
        if hasattr(dut, s): getattr(dut, s).value = 0
    await RisingEdge(dut.clk); await RisingEdge(dut.clk)
    dut.rst_n.value = 1

@cocotb.test(skip=os.environ.get("TOPLEVEL") != "sem_file")
async def test_sem_sum_and_host_priority(dut):
    await reset(dut)
    for n in ("inc_rd_v", "inc_wr_v", "inc_k_v", "inc_sig_v", "host_we"):
        getattr(dut, n).value = 0
    # три источника на семафор 2 и SIGNAL +5 на него же в одном такте → +8
    dut.inc_rd_v.value = 1; dut.inc_rd_idx.value = 2
    dut.inc_wr_v.value = 1; dut.inc_wr_idx.value = 2
    dut.inc_k_v.value = 1;  dut.inc_k_idx.value = 2
    dut.inc_sig_v.value = 1; dut.inc_sig_idx.value = 2; dut.inc_sig_val.value = 5
    await RisingEdge(dut.clk)
    for n in ("inc_rd_v", "inc_wr_v", "inc_k_v", "inc_sig_v"):
        getattr(dut, n).value = 0
    await ReadOnly()
    assert int(dut.sem_val.value) >> (32 * 2) & 0xFFFFFFFF == 8
    await RisingEdge(dut.clk)
    # запись хоста побеждает приращение в том же такте
    dut.host_we.value = 1; dut.host_idx.value = 2; dut.host_val.value = 100
    dut.inc_rd_v.value = 1; dut.inc_rd_idx.value = 2
    await RisingEdge(dut.clk)
    dut.host_we.value = 0; dut.inc_rd_v.value = 0
    await ReadOnly()
    assert int(dut.sem_val.value) >> (32 * 2) & 0xFFFFFFFF == 100

@cocotb.test(skip=os.environ.get("TOPLEVEL") != "fifo_sync")
async def test_fifo_random(dut):
    await reset(dut)
    rng = random.Random(1); src = [rng.randrange(256) for _ in range(500)]; got = []
    i = 0
    for _ in range(3000):
        dut.in_v.value = 1 if (i < len(src) and rng.random() < 0.6) else 0
        dut.in_d.value = src[i] if i < len(src) else 0
        dut.out_rdy.value = 1 if rng.random() < 0.5 else 0
        await ReadOnly()
        push = int(dut.in_v.value) and int(dut.in_rdy.value)
        pop = int(dut.out_v.value) and int(dut.out_rdy.value)
        if pop: got.append(int(dut.out_d.value))
        await RisingEdge(dut.clk)
        if push: i += 1
    assert got == src[:len(got)] and len(got) > 400

@cocotb.test(skip=os.environ.get("TOPLEVEL") != "spm")
async def test_spm_port_a_priority(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.b_en.value = 0; dut.a_rd_req.value = 0
    dut.a_wr_v.value = 1; dut.a_wr_addr.value = 5; dut.a_wr_data.value = 0xABC
    dut.a_rd_req.value = 1; dut.a_rd_addr.value = 5
    await ReadOnly(); assert int(dut.a_rd_gnt.value) == 0      # запись R побеждает
    await RisingEdge(dut.clk)
    dut.a_wr_v.value = 0
    await ReadOnly(); assert int(dut.a_rd_gnt.value) == 1
    await RisingEdge(dut.clk); dut.a_rd_req.value = 0
    await ReadOnly(); assert int(dut.a_rd_data.value) == 0xABC
```

Run: `cd scripts && ./run.sh sem_file test_prims "../rtl/kshell_pkg.sv ../rtl/sem_file.sv"`
Expected: ошибка сборки (нет `sem_file.sv`).

- [ ] **Step 3: написать `kshell_pkg.sv`, `fifo_sync.sv`, `sem_file.sv`, `spm.sv`** по интерфейсам выше и §10–§11.

- [ ] **Step 4: прогнать все три теста**

```bash
cd scripts
./run.sh sem_file  test_prims "../rtl/kshell_pkg.sv ../rtl/sem_file.sv"
./run.sh fifo_sync test_prims "../rtl/kshell_pkg.sv ../rtl/fifo_sync.sv"
./run.sh spm       test_prims "../rtl/kshell_pkg.sv ../rtl/spm.sv"
make lint
```

Expected: каждый прогон — `PASS=1` (остальные тесты SKIP); `lint OK`.

- [ ] **Step 5: коммит**

```bash
git add .gitignore .rules.verible docs/style_notes.md scripts rtl tb
git commit -m "scaffold + примитивы: kshell_pkg, fifo_sync, sem_file, spm (+тесты)"
```

---

### Task 2: Python-модели — ISA, проверщик программ, драйвер AXI-Lite, модель памяти

**Files:**
- Create: `tb/isa.py`, `tb/axil.py`, `tb/hbm_model.py`
- Test: `tb/test_isa.py` (pytest, без симулятора)

**Interfaces (Produces):**

```python
# tb/isa.py
OP = dict(LOAD=1, STREAM=2, STORE=3, WAIT=4, SIGNAL=5, RUN=6)
BURST_BYTES = 256; BB = 32; SPM_WORDS = 1024

def encode(op, sem=0, spm=0, hbm=0, length=0, w1=None) -> tuple[int, int, int, int]:
    """Вернуть (CMD0, CMD1, CMD2, CMD3) по §6. Для WAIT/SIGNAL w1 = thr/inc → CMD1.
    Для RUN аргументы передаются через run()."""

class Program:
    def __init__(self): self.cmds = []          # list[dict]: op, поля, encoded
    def load(self, sem, spm, hbm, length): ...
    def stream(self, hbm, length): ...
    def store(self, sem, spm, hbm, length): ...
    def wait(self, sem, thr): ...
    def signal(self, sem, inc): ...
    def run(self, sem, a0, a1, a2, a3): ...     # a3 ≤ 0xFFFF → CMD0[15:0]
    def encoded(self) -> list[tuple[int, int, int, int]]: ...
    def increments(self, sem) -> int: ...       # сколько приращений выдано на sem

class ProgramError(Exception): ...

def check_program(prog: Program, ntag: int = 16, run_regions="gemv") -> None:
    """Абстрактное исполнение §6.1; ProgramError при нарушении правил 1–3:
    1) пары STREAM–RUN: число STREAM == число RUN, k-я STREAM раньше k-го RUN;
    2) head-of-line: WAIT, условие которого зависит от команды rd, стоящей после
       STREAM, чей RUN ещё не выдан, — тупик;
    3) пересечение областей spm/памяти без WAIT на семафор предыдущей команды.
    run_regions: "gemv" — области RUN по правилу GEMV; None — RUN не трогает spm
    (для программ без RUN или с sink_kernel); callable(args) -> (reads, writes)."""
```

```python
# tb/axil.py
class AxilMaster:
    def __init__(self, dut, prefix="s_axil"): ...
    async def write(self, addr: int, data: int) -> None: ...
    async def read(self, addr: int) -> int: ...
```

```python
# tb/hbm_model.py
class HbmModel:
    def __init__(self, dut, prefix="m_axi", n_pc=8, lat=(20, 60), seed=0,
                 p_stall=0.2, interleave=False, slverr=None, has_read=True, has_write=True): ...
    def write_bytes(self, addr: int, data: bytes) -> None: ...
    def read_bytes(self, addr: int, n: int) -> bytes: ...
    def freeze(self) -> None: ...                # перестать отвечать (для watchdog)
    def start(self) -> None: ...                 # запустить корутины AR/R и AW/W/B
    def stop(self) -> None: ...                  # убить корутины (перед повторным bringup в одном тесте)
    # проверки протокола внутри модели (assert): AxLEN == 7, AxSIZE == 5,
    # burst не пересекает 4 KB, число W == AxLEN+1, WLAST на последнем.
    # PC = (addr >> 8) % n_pc; у PC своя очередь и задержка randint(*lat) тактов;
    # R по разным ID — в порядке готовности (OoO); B для одного ID — по порядку.
    # slverr: set адресов burst'ов, для которых RRESP/BRESP = 2 (SLVERR).
```

- [ ] **Step 1: тест ISA и проверщика (падает: модулей нет)**

```python
# tb/test_isa.py
import pytest
from isa import Program, check_program, ProgramError, encode

def test_encode_load():
    c0, c1, c2, c3 = encode("LOAD", sem=3, spm=0x40, hbm=0x12_3456_7800, length=512)
    assert c0 == (1 << 28) | (3 << 24) | 0x40
    assert c1 == 0x34567800 and c2 == 0x12 and c3 == 512

def demo_program(M=64, K=256):
    p = Program()
    p.load(0, 0, 0x1000, K); p.stream(0x10000, M * K); p.wait(0, 1)
    p.run(1, 0, 64, M, K)
    p.load(0, 16, 0x2000, K); p.stream(0x10000, M * K); p.wait(0, 2)
    p.run(1, 16, 72, M, K)
    p.wait(1, 1); p.store(2, 64, 0x80000, 4 * M)
    p.wait(1, 2); p.store(2, 72, 0x90000, 4 * M)
    p.wait(2, 2); p.signal(7, 1)
    return p

def test_demo_program_ok():
    check_program(demo_program())

def test_hol_deadlock_rejected():
    p = Program()
    p.stream(0x10000, 64 * 256 * 64)            # больше NTAG burst'ов
    p.load(0, 0, 0x1000, 256)
    p.wait(0, 1)                                # ждём LOAD за STREAM, RUN ещё не выдан
    p.run(1, 0, 64, 64, 256)
    with pytest.raises(ProgramError, match="head-of-line"):
        check_program(p)

def test_missing_wait_rejected():
    p = Program()
    p.load(0, 0, 0x1000, 256)
    p.run(1, 0, 64, 1, 256)                     # RUN читает spm[0..] без WAIT на LOAD
    p.stream(0x10000, 256)
    with pytest.raises(ProgramError):
        check_program(p)
```

Run: `~/fpga-venv/bin/python -m pytest tb/test_isa.py -q`
Expected: FAIL (`ModuleNotFoundError: isa`).

- [ ] **Step 2: реализовать `isa.py`.** Проверщику нужны области данных: LOAD пишет `spm[spm, spm+len/32)`, STORE читает их; RUN для ядра GEMV читает `spm[a0, a0+a3/32)` и пишет `spm[a1, a1+ceil(a2/8))` — функцию «какие области трогает RUN» передавать параметром `run_regions=lambda a: (reads, writes)`; по умолчанию — правило GEMV. Пример неправильного порядка в `test_missing_wait_rejected` нарушает и правило 1 (STREAM после RUN) — проверщик должен ругаться на любое из нарушений.

- [ ] **Step 3: реализовать `axil.py` и `hbm_model.py`** по интерфейсам выше (модель проверяется в Task 3 на rd_dma).

- [ ] **Step 4: тесты проходят**

Run: `~/fpga-venv/bin/python -m pytest tb/test_isa.py -q`
Expected: `4 passed`.

- [ ] **Step 5: коммит**

```bash
git add tb/isa.py tb/axil.py tb/hbm_model.py tb/test_isa.py
git commit -m "tb: ISA-кодировщик, check_program, AXI-Lite драйвер, модель HBM (OoO)"
```

---

### Task 3: rd_dma

**Files:**
- Create: `rtl/rd_dma.sv`, `rtl/rd_dma_sva.sv`
- Test: `tb/test_rd_dma.py`

**Interfaces:**

```systemverilog
module rd_dma import kshell_pkg::*; #(parameter int NTAG = 16) (
  input  logic clk, input logic rst_n,
  input  logic soft_rst,                  // §12.4
  input  logic go,                        // ENABLE && !error: резчик может выдавать AR
  input  logic abort,                     // ошибка: ROB отбрасывается, sem не растёт
  input  logic [31:0] now,                // счётчик тактов (для задержки)
  // команда из очереди rd
  input  logic cmd_v, output logic cmd_rdy,
  input  logic cmd_stream, input logic [SEMW-1:0] cmd_sem, input logic [SPMAW-1:0] cmd_spm,
  input  logic [AW-1:0] cmd_hbm, input logic [NBW-1:0] cmd_nb, input logic [31:0] cmd_no,
  // AXI4 AR/R
  output logic m_axi_arvalid, input logic m_axi_arready, output logic [AW-1:0] m_axi_araddr,
  output logic [$clog2(NTAG)-1:0] m_axi_arid, output logic [7:0] m_axi_arlen,
  output logic [2:0] m_axi_arsize, output logic [1:0] m_axi_arburst, output logic [3:0] m_axi_arcache,
  input  logic m_axi_rvalid, output logic m_axi_rready, input logic [DW-1:0] m_axi_rdata,
  input  logic [$clog2(NTAG)-1:0] m_axi_rid, input logic [1:0] m_axi_rresp, input logic m_axi_rlast,
  // запись в spm (порт A, приоритетная)
  output logic spm_wr_v, output logic [SPMAW-1:0] spm_wr_addr, output logic [DW-1:0] spm_wr_data,
  // поток к ядру
  output logic k_s_tvalid, input logic k_s_tready, output logic [DW-1:0] k_s_tdata, output logic k_s_tlast,
  // коммит LOAD
  output logic sem_inc_v, output logic [SEMW-1:0] sem_inc_idx,
  // ошибка RRESP (импульс) и номер команды
  output logic err_v, output logic [31:0] err_cmd,
  // статус и perf
  output logic axi_quiet,                 // нет AR без последнего R
  output logic idle,                      // нет команды в резчике, окно пусто, выход AXIS пуст
  output logic [7:0] inflight,
  output logic pf_ar_hs, output logic pf_win_full, output logic pf_lat_v, output logic [31:0] pf_lat);
```

Требования: §8.1–§8.6 буквально. `m_axi_rready = 1`. `cmd_nb` = `len / 256` (считает секвенсор). Выходной skid на 2 записи (§8.5).

- [ ] **Step 1: тест (падает: модуля нет)**

```python
# tb/test_rd_dma.py
import random, cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly
from hbm_model import HbmModel

async def setup(dut, **mk):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    for s in ("soft_rst", "abort", "cmd_v", "k_s_tready"): getattr(dut, s).value = 0
    dut.go.value = 1; dut.now.value = 0; dut.rst_n.value = 0
    await RisingEdge(dut.clk); await RisingEdge(dut.clk); dut.rst_n.value = 1
    mem = HbmModel(dut, has_write=False, **mk); mem.start()
    async def tick():
        n = 0
        while True:
            await RisingEdge(dut.clk); n += 1; dut.now.value = n
    cocotb.start_soon(tick())
    return mem

async def push(dut, stream, sem, spm, hbm, nb, no):
    dut.cmd_v.value = 1; dut.cmd_stream.value = stream; dut.cmd_sem.value = sem
    dut.cmd_spm.value = spm; dut.cmd_hbm.value = hbm; dut.cmd_nb.value = nb; dut.cmd_no.value = no
    while True:
        await ReadOnly()
        ok = int(dut.cmd_rdy.value); await RisingEdge(dut.clk)
        if ok: break
    dut.cmd_v.value = 0

@cocotb.test()
async def test_load_ooo_commit_in_order(dut):
    mem = await setup(dut, lat=(5, 80), seed=7)
    data = bytes(random.Random(1).randrange(256) for _ in range(256 * 40))
    mem.write_bytes(0x10000, data)
    spm = {}; sems = []
    async def watch():
        while True:
            await ReadOnly()
            if int(dut.spm_wr_v.value):
                spm[int(dut.spm_wr_addr.value)] = int(dut.spm_wr_data.value)
            if int(dut.sem_inc_v.value): sems.append(int(dut.sem_inc_idx.value))
            await RisingEdge(dut.clk)
    cocotb.start_soon(watch())
    await push(dut, 0, 3, 0, 0x10000, 40, 1)            # 40 burst'ов > NTAG
    for _ in range(4000):
        await RisingEdge(dut.clk)
        if sems: break
    assert sems == [3]
    words = [int.from_bytes(data[i*32:(i+1)*32], "little") for i in range(320)]
    assert [spm[i] for i in range(320)] == words
    assert int(dut.axi_quiet.value) == 1

@cocotb.test()
async def test_stream_order_under_backpressure(dut):
    mem = await setup(dut, lat=(5, 80), seed=9, interleave=True)
    data = bytes(random.Random(2).randrange(256) for _ in range(256 * 24))
    mem.write_bytes(0x20000, data)
    got = []; lasts = []; rng = random.Random(3)
    async def sink():
        while len(got) < 24 * 8:
            dut.k_s_tready.value = 1 if rng.random() < 0.6 else 0
            await ReadOnly()
            if int(dut.k_s_tvalid.value) and int(dut.k_s_tready.value):
                got.append(int(dut.k_s_tdata.value)); lasts.append(int(dut.k_s_tlast.value))
            await RisingEdge(dut.clk)
    s = cocotb.start_soon(sink())
    await push(dut, 1, 0, 0, 0x20000, 24, 2)
    await s
    assert got == [int.from_bytes(data[i*32:(i+1)*32], "little") for i in range(192)]
    assert lasts == [0] * 191 + [1]

@cocotb.test()
async def test_rresp_error_abort(dut):
    mem = await setup(dut, lat=(5, 40), seed=11, slverr={0x30000 + 256 * 3})
    sems = []; errs = []
    async def watch():
        while True:
            await ReadOnly()
            if int(dut.sem_inc_v.value): sems.append(1)
            if int(dut.err_v.value): errs.append(int(dut.err_cmd.value))
            await RisingEdge(dut.clk)
    cocotb.start_soon(watch())
    await push(dut, 0, 1, 0, 0x30000, 8, 42)
    for _ in range(50):
        await RisingEdge(dut.clk)
        if errs: dut.abort.value = 1; dut.go.value = 0
    for _ in range(500):
        await RisingEdge(dut.clk)
        if int(dut.axi_quiet.value): break
    assert errs[:1] == [42] and sems == [] and int(dut.axi_quiet.value) == 1
```

Run: `cd scripts && ./run.sh rd_dma test_rd_dma "../rtl/kshell_pkg.sv ../rtl/rd_dma.sv"`
Expected: ошибка сборки.

- [ ] **Step 2: написать `rd_dma.sv`** по §8. Структура: FSM резчика (IDLE → ISSUE, счётчик burst'ов команды); окно — массивы слотов (`type, spm_addr, beat_cnt, done, last_of_cmd, sem, cmd_no, t_issue`); `cmd_err` на команду — регистр на «текущую коммитящуюся команду» (команды коммитятся по порядку, достаточно одного накопителя на голову + флага на каждый слот); ROB — `NTAG*BURST × DW` память; выдача из ROB через skid на 2 записи; `abort` — резчик стоп, голова снимает слоты без выдачи и без `sem_inc_v`.

- [ ] **Step 3: написать `rd_dma_sva.sv`** (bind на `rd_dma`): `arvalid` держится до `arready` и `araddr/arid` стабильны; `arlen == BURST-1`; RVALID только для слота в полёте; `rlast` ⇔ `beat_cnt[rid] == BURST-1`; `k_s_tvalid && !k_s_tready |=> $stable(k_s_tdata) && k_s_tvalid`; `tail − head ≤ NTAG`.

- [ ] **Step 4: тесты проходят, lint**

Run: `./run.sh rd_dma test_rd_dma "../rtl/kshell_pkg.sv ../rtl/rd_dma.sv" && make lint`
Expected: `PASS=3`, ассершены не срабатывают, `lint OK`.

- [ ] **Step 5: коммит**

```bash
git add rtl/rd_dma.sv rtl/rd_dma_sva.sv tb/test_rd_dma.py
git commit -m "rd_dma: окно-ROB, OoO LOAD → spm, STREAM по порядку, abort (+sva, тесты)"
```

---

### Task 4: wr_dma

**Files:**
- Create: `rtl/wr_dma.sv`, `rtl/wr_dma_sva.sv`
- Test: `tb/test_wr_dma.py`

**Interfaces:**

```systemverilog
module wr_dma import kshell_pkg::*; (
  input  logic clk, input logic rst_n, input logic soft_rst, input logic go, input logic abort,
  input  logic cmd_v, output logic cmd_rdy,
  input  logic [SEMW-1:0] cmd_sem, input logic [SPMAW-1:0] cmd_spm, input logic [AW-1:0] cmd_hbm,
  input  logic [NBW-1:0] cmd_nb, input logic [31:0] cmd_no,
  // чтение spm (порт A, неприоритетное)
  output logic spm_rd_req, output logic [SPMAW-1:0] spm_rd_addr, input logic spm_rd_gnt,
  input  logic [DW-1:0] spm_rd_data,
  // AXI4 AW/W/B
  output logic m_axi_awvalid, input logic m_axi_awready, output logic [AW-1:0] m_axi_awaddr,
  output logic [0:0] m_axi_awid, output logic [7:0] m_axi_awlen, output logic [2:0] m_axi_awsize,
  output logic [1:0] m_axi_awburst, output logic [3:0] m_axi_awcache,
  output logic m_axi_wvalid, input logic m_axi_wready, output logic [DW-1:0] m_axi_wdata,
  output logic [BB-1:0] m_axi_wstrb, output logic m_axi_wlast,
  input  logic m_axi_bvalid, output logic m_axi_bready, input logic [0:0] m_axi_bid,
  input  logic [1:0] m_axi_bresp,
  output logic sem_inc_v, output logic [SEMW-1:0] sem_inc_idx,
  output logic err_v, output logic [31:0] err_cmd,
  output logic axi_quiet, output logic idle, output logic [7:0] inflight, output logic pf_aw_hs);
```

Требования — §9: сначала полный burst в W-FIFO (`fifo_sync`, глубина 2×BURST), потом AW, потом W подряд; `AWID = 0`, `AWCACHE = 4'b0010`, `WSTRB` все единицы; FIFO `cmd_no` для burst'ов в полёте (глубина `WR_OUT`); `sem_inc_v` после последнего B команды без ошибки.

- [ ] **Step 1: тест (падает)**

```python
# tb/test_wr_dma.py
import random, cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly
from hbm_model import HbmModel

@cocotb.test()
async def test_store_and_spm_starvation(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    for s in ("soft_rst", "abort", "cmd_v"): getattr(dut, s).value = 0
    dut.go.value = 1; dut.rst_n.value = 0
    await RisingEdge(dut.clk); await RisingEdge(dut.clk); dut.rst_n.value = 1
    mem = HbmModel(dut, has_read=False, lat=(5, 50), seed=5); mem.start()
    rng = random.Random(4)
    spm = [rng.randrange(1 << 256) for _ in range(64)]
    async def spm_port():                         # порт A: gnt только в половине тактов
        pending = None
        while True:
            busy = rng.random() < 0.5            # имитация приоритетных R-записей
            dut.spm_rd_gnt.value = 0 if busy else int(dut.spm_rd_req.value)
            dut.spm_rd_data.value = spm[pending] if pending is not None else 0
            await ReadOnly()
            nxt = int(dut.spm_rd_addr.value) if int(dut.spm_rd_gnt.value) else None
            await RisingEdge(dut.clk)
            pending = nxt
    cocotb.start_soon(spm_port())
    sems = []
    async def watch():
        while True:
            await ReadOnly()
            if int(dut.sem_inc_v.value): sems.append(int(dut.sem_inc_idx.value))
            await RisingEdge(dut.clk)
    cocotb.start_soon(watch())
    dut.cmd_v.value = 1; dut.cmd_sem.value = 2; dut.cmd_spm.value = 0
    dut.cmd_hbm.value = 0x40000; dut.cmd_nb.value = 8; dut.cmd_no.value = 5
    await RisingEdge(dut.clk); dut.cmd_v.value = 0
    for _ in range(3000):
        await RisingEdge(dut.clk)
        if sems: break
    assert sems == [2]
    got = mem.read_bytes(0x40000, 64 * 32)
    assert [int.from_bytes(got[i*32:(i+1)*32], "little") for i in range(64)] == spm
    assert int(dut.axi_quiet.value) == 1
```

Run: `./run.sh wr_dma test_wr_dma "../rtl/kshell_pkg.sv ../rtl/fifo_sync.sv ../rtl/wr_dma.sv"`
Expected: ошибка сборки.

- [ ] **Step 2: написать `wr_dma.sv`** по §9 (FSM: IDLE → FILL (читать BURST слов в W-FIFO) → AW → W×BURST → следующий burst).
- [ ] **Step 3: `wr_dma_sva.sv`:** `awvalid` стабилен до handshake; `awlen == BURST-1`; после handshake AW все BURST beat'ов W выдаются без пауз со стороны wr_dma (`m_axi_wvalid` не падает до `wlast`); `wlast` на beat'е № BURST; `awcache == 4'b0010`.
- [ ] **Step 4:** `./run.sh wr_dma test_wr_dma "…" && make lint` → `PASS=1`, `lint OK`.
- [ ] **Step 5: коммит** `git commit -m "wr_dma: W-FIFO до AW, один AWID, счёт B, abort (+sva, тест)"`

---

### Task 5: cmd_seq

**Files:**
- Create: `rtl/cmd_seq.sv`
- Test: `tb/test_cmd_seq.py`

**Interfaces:**

```systemverilog
module cmd_seq import kshell_pkg::*; (
  input  logic clk, input logic rst_n, input logic soft_rst,
  input  logic enable, input logic error,          // error = липкий флаг ошибки из kshell
  // командная FIFO
  input  logic cq_v, output logic cq_pop, input logic [127:0] cq_d,
  // в очередь rd
  output logic rd_v, input logic rd_rdy, output logic rd_stream, output logic [SEMW-1:0] rd_sem,
  output logic [SPMAW-1:0] rd_spm, output logic [AW-1:0] rd_hbm, output logic [NBW-1:0] rd_nb,
  output logic [31:0] rd_no,
  // в очередь wr
  output logic wr_v, input logic wr_rdy, output logic [SEMW-1:0] wr_sem,
  output logic [SPMAW-1:0] wr_spm, output logic [AW-1:0] wr_hbm, output logic [NBW-1:0] wr_nb,
  output logic [31:0] wr_no,
  // в очередь ядра
  output logic krn_v, input logic krn_rdy, output logic [SEMW-1:0] krn_sem,
  output logic [3:0][31:0] krn_arg, output logic [31:0] krn_no,
  // семафоры
  input  logic [NSEM-1:0][31:0] sem_val,
  output logic sig_v, output logic [SEMW-1:0] sig_idx, output logic [31:0] sig_val,
  // ошибки декодирования (импульс) и статус
  output logic dec_err_v, output err_e dec_err_code, output logic [31:0] dec_err_cmd,
  output logic [31:0] cmd_no, output logic in_wait, output logic pf_wait);
```

Требования — §6 (поля, проверки `ALIGN`/`RANGE`/`BAD_OPCODE` с 33-битной суммой), §7. При `error`: текущий WAIT прерывается, `cq_pop` выкачивает и отбрасывает FIFO (очистка), ничего не выдаётся. `rd_nb = len >> 8`.

- [ ] **Step 1: тест (падает).** Тестбенч подаёт закодированные `isa.encode` команды через `cq_v/cq_d`, держит `rd_rdy/wr_rdy/krn_rdy` случайно, и проверяет:

```python
# tb/test_cmd_seq.py (основная часть)
import cocotb, random
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly
from isa import encode

def pack(c): return c[0] | (c[1] << 32) | (c[2] << 64) | (c[3] << 96)

@cocotb.test()
async def test_dispatch_wait_and_decode_errors(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst_n.value = 0; dut.soft_rst.value = 0; dut.enable.value = 1; dut.error.value = 0
    dut.cq_v.value = 0; dut.rd_rdy.value = 1; dut.wr_rdy.value = 1; dut.krn_rdy.value = 1
    dut.sem_val.value = 0
    await RisingEdge(dut.clk); await RisingEdge(dut.clk); dut.rst_n.value = 1
    cmds = [encode("LOAD", sem=1, spm=0, hbm=0x1000, length=512),
            encode("WAIT", sem=1, w1=1),
            encode("STORE", sem=2, spm=0, hbm=0x2000, length=512),
            encode("LOAD", sem=1, spm=0, hbm=0x1001, length=512)]   # ALIGN
    seen = []
    i = 0
    for cyc in range(200):
        dut.cq_v.value = 1 if i < len(cmds) else 0
        dut.cq_d.value = pack(cmds[i]) if i < len(cmds) else 0
        if cyc == 50: dut.sem_val.value = 1 << 32            # sem1 = 1 → WAIT проходит
        await ReadOnly()
        if int(dut.rd_v.value): seen.append(("rd", int(dut.rd_nb.value)))
        if int(dut.wr_v.value): seen.append(("wr", cyc))
        if int(dut.dec_err_v.value): seen.append(("err", int(dut.dec_err_code.value), int(dut.dec_err_cmd.value)))
        pop = int(dut.cq_pop.value)
        await RisingEdge(dut.clk)
        if pop: i += 1
    assert seen[0] == ("rd", 2)
    assert seen[1][0] == "wr" and seen[1][1] >= 50              # STORE только после WAIT
    assert seen[2] == ("err", 3, 3)                            # ALIGN, номер команды 3
```

Run: `./run.sh cmd_seq test_cmd_seq "../rtl/kshell_pkg.sv ../rtl/cmd_seq.sv"` → ошибка сборки.

- [ ] **Step 2: написать `cmd_seq.sv`** (FSM: FETCH → DISPATCH/WAIT; `cmd_no` с нуля, номер взятой команды начинается с 0).
- [ ] **Step 3:** тест проходит + `make lint`.
- [ ] **Step 4: коммит** `git commit -m "cmd_seq: декодирование, проверки, WAIT/SIGNAL, выдача в очереди (+тест)"`

---

### Task 6: регистры, perf, сборка `kshell`, тестовое ядро; системные T1, T2, T7

**Files:**
- Create: `rtl/axil_regs.sv`, `rtl/kshell_perf.sv`, `rtl/kshell.sv`, `rtl/kshell_sva.sv`, `tb/sink_kernel.sv`, `tb/kshell_sink_top.sv`, `tb/shell.py`
- Test: `tb/test_system.py` (тесты T1, T2, T7)

**Interfaces:**

```systemverilog
module kshell import kshell_pkg::*; #(parameter int NTAG = 16, parameter int WDOG_CYCLES = 1 << 20) (
  input logic clk, input logic rst_n,
  // AXI4-Lite slave (12 бит адреса, 32 бита данных): s_axil_awvalid/awready/awaddr,
  //   wvalid/wready/wdata/wstrb, bvalid/bready/bresp, arvalid/arready/araddr,
  //   rvalid/rready/rdata/rresp
  // AXI4 master: m_axi_* — объединение портов rd_dma (AR/R) и wr_dma (AW/W/B); ARID шириной log2(NTAG)
  // контракт ядра §5:
  output logic k_rst_n, output logic k_start, output logic [3:0][31:0] k_arg,
  input  logic k_done, input logic k_err,
  output logic k_s_tvalid, input logic k_s_tready, output logic [DW-1:0] k_s_tdata, output logic k_s_tlast,
  input  logic k_spm_en, input logic k_spm_we, input logic [SPMAW-1:0] k_spm_addr,
  input  logic [DW-1:0] k_spm_wdata, input logic [BB-1:0] k_spm_wstrb, output logic [DW-1:0] k_spm_rdata);

module axil_regs import kshell_pkg::*; (
  input logic clk, input logic rst_n, /* s_axil_* */,
  output logic enable, output logic soft_rst_req, output logic perf_snap,
  output logic cq_push, output logic [127:0] cq_d, input logic cq_full,
  output logic sem_we, output logic [SEMW-1:0] sem_idx, output logic [31:0] sem_wval,
  input logic [31:0] status, input logic [3:0] err_code, input logic [31:0] err_cmd,
  input logic [31:0] cmd_count, input logic [NSEM-1:0][31:0] sem_val,
  input logic [7:0][31:0] perf);

module kshell_perf (
  input logic clk, input logic rst_n, input logic clr, input logic enable, input logic snap,
  input logic ev_seq_wait, input logic ev_ar_hs, input logic ev_win_full, input logic ev_lat_v,
  input logic [31:0] lat, input logic ev_strm_bp, input logic ev_aw_hs, input logic ev_krn_busy,
  output logic [7:0][31:0] perf);         // порядок = таблица §14
```

`kshell` внутри: `fifo_sync` (командная, `CQ_DEPTH`), три `fifo_sync` очередей движков (`EQ_DEPTH`), `cmd_seq`, `rd_dma`, `wr_dma`, `spm`, `sem_file`, `kshell_perf`, `axil_regs`; локальная логика: липкий флаг ошибки и захват первой ошибки по приоритету §12.2; контроллер ядра (`krn_busy`, `k_start` из очереди ядра, `sem += 1` по `k_done` без ошибки); `AXI_QUIET`, `IDLE` (§12.1); `soft_rst` = `soft_rst_req && AXI_QUIET`, держит `k_rst_n = 0` один такт; watchdog (§12.5); `cq_ovf`.

`sink_kernel` (контракт §5): `arg0` = адрес spm, `arg1` = число beat'ов; пишет каждый принятый beat в spm[arg0 + i] через порт B; `k_done` после beat'а с `tlast`; `k_err`, если `tlast` не на beat'е `arg1−1`.

```python
# tb/shell.py
from axil import AxilMaster
REG = dict(CTRL=0x000, STATUS=0x004, ERR_CODE=0x008, ERR_CMD=0x00C, CMD0=0x010,
           DOORBELL=0x020, CMD_COUNT=0x024, SEM=0x040, PERF=0x080)
PERF_NAMES = ["CYCLES", "SEQ_WAIT", "RD_BURSTS", "RD_LAT_SUM", "RD_WIN_FULL",
              "STRM_BP", "WR_BURSTS", "KRN_BUSY"]
async def reset(dut): ...                               # rst_n; Clock запускается один раз (флаг на dut)
async def submit(axil, prog) -> None: ...               # проверяет cq_free (STATUS[15:8]) перед каждой командой
async def wait_sem(axil, idx, thr, timeout=100000) -> None: ...
async def status(axil) -> dict: ...                     # IDLE, AXI_QUIET, ERROR, cq_ovf, krn_busy, cq_free, …
async def perf(axil) -> dict: ...                       # PERF_SNAP + чтение 8 счётчиков
async def soft_reset(axil) -> None: ...                 # ждёт AXI_QUIET, пишет SOFT_RESET
```

- [ ] **Step 1: системные тесты T1, T2, T7 (падают)**

```python
# tb/test_system.py — часть 1
import os, random, cocotb, numpy as np
from cocotb.triggers import RisingEdge
from axil import AxilMaster
from hbm_model import HbmModel
from isa import Program, check_program
import shell

TOP = os.environ.get("TOPLEVEL")

async def bringup(dut, **mk):
    await shell.reset(dut)
    mem = HbmModel(dut, **mk); mem.start()
    ax = AxilMaster(dut); await ax.write(shell.REG["CTRL"], 1)   # ENABLE
    return ax, mem

@cocotb.test(skip=TOP != "kshell_sink_top")
async def t1_memcpy(dut):
    ax, mem = await bringup(dut, lat=(5, 90), seed=1)
    src = bytes(random.Random(1).randrange(256) for _ in range(256 * 24))
    mem.write_bytes(0x100000, src)
    p = Program(); p.load(0, 0, 0x100000, len(src)); p.wait(0, 1)
    p.store(1, 0, 0x200000, len(src)); p.wait(1, 1); p.signal(7, 1)
    check_program(p, run_regions=None)
    await shell.submit(ax, p); await shell.wait_sem(ax, 7, 1)
    assert mem.read_bytes(0x200000, len(src)) == src

@cocotb.test(skip=TOP != "kshell_sink_top")
async def t2_stream_order(dut):
    ax, mem = await bringup(dut, lat=(5, 90), seed=2, interleave=True)
    src = bytes(random.Random(2).randrange(256) for _ in range(256 * 20))
    mem.write_bytes(0x300000, src)
    p = Program(); p.stream(0x300000, len(src)); p.run(0, 0, len(src) // 32, 0, 0)
    p.wait(0, 1); p.store(1, 0, 0x400000, len(src)); p.wait(1, 1); p.signal(7, 1)
    await shell.submit(ax, p); await shell.wait_sem(ax, 7, 1)
    assert mem.read_bytes(0x400000, len(src)) == src

@cocotb.test(skip=TOP != "kshell_sink_top")
async def t7_decode_errors(dut):
    ax, mem = await bringup(dut)
    for prog_fn, code in ((lambda p: p.load(0, 0, 0x1001, 256), 3),        # ALIGN
                          (lambda p: p.load(0, 1020, 0x1000, 512), 4),     # RANGE
                          (lambda p: p.raw((0 << 28), 0, 0, 0), 5)):       # BAD_OPCODE
        p = Program(); prog_fn(p)
        await shell.submit(ax, p)
        for _ in range(200): await RisingEdge(dut.clk)
        st = await shell.status(ax)
        assert st["ERROR"] and await ax.read(shell.REG["ERR_CODE"]) == code
        await shell.soft_reset(ax); await ax.write(shell.REG["CTRL"], 1)
    # переполнение командной FIFO без ERROR
    await ax.write(shell.REG["CTRL"], 0)                                  # пауза: FIFO не убывает
    for _ in range(20): await ax.write(shell.REG["DOORBELL"], 0)
    st = await shell.status(ax)
    assert st["cq_ovf"] and not st["ERROR"]
```

(`Program.raw(c0, c1, c2, c3)` — добавить в `isa.py` для «сырых» дескрипторов; `check_program(..., run_regions=None)` — без правила о данных RUN.)

Run: `./run.sh kshell_sink_top test_system "<все rtl/*.sv кроме *_sva/*_cov/kshell_top/gemv_kernel> ../tb/sink_kernel.sv ../tb/kshell_sink_top.sv"` → ошибка сборки.

- [ ] **Step 2: написать `axil_regs.sv`, `kshell_perf.sv`, `kshell.sv`, `sink_kernel.sv`, `kshell_sink_top.sv`, `shell.py`.**
- [ ] **Step 3: `kshell_sva.sv`:** `soft_rst |-> axi_quiet`; первая ошибка не перезаписывается; нет двух записей в порт A spm в одном такте (`rd.spm_wr_v` и `wr.spm_rd_gnt` никогда не оба); нет одновременного доступа портов A и B к одному адресу.
- [ ] **Step 4:** T1, T2, T7 проходят; `make lint`.
- [ ] **Step 5: коммит** `git commit -m "kshell: регистры, perf, ошибки, контроллер ядра, AXI_QUIET/IDLE; T1 T2 T7"`

---

### Task 7: демо-ядро `gemv_kernel`

**Files:**
- Create: `rtl/gemv_kernel.sv`, `rtl/kshell_kif_sva.sv`
- Test: `tb/test_gemv.py` (ядро отдельно: тестбенч играет роль обёртки — поток + модель spm на порту B)

**Interfaces:** порты = контракт §5 (те же имена, что у `kshell`, со стороны ядра):

```systemverilog
module gemv_kernel import kshell_pkg::*; (
  input logic clk, input logic k_rst_n,
  input logic k_start, input logic [3:0][31:0] k_arg, output logic k_done, output logic k_err,
  input logic k_s_tvalid, output logic k_s_tready, input logic [DW-1:0] k_s_tdata, input logic k_s_tlast,
  output logic k_spm_en, output logic k_spm_we, output logic [SPMAW-1:0] k_spm_addr,
  output logic [DW-1:0] k_spm_wdata, output logic [BB-1:0] k_spm_wstrb, input logic [DW-1:0] k_spm_rdata);
```

`kshell_kif_sva.sv` — модуль с теми же портами (только входы) + `krn_busy` вычисляется внутри по `k_start`/`k_done`; `bind gemv_kernel kshell_kif_sva u_kif (.*)` и `bind sink_kernel …` — правила §16 «`kshell_kif_sva`».

- [ ] **Step 1: тест (падает)**

```python
# tb/test_gemv.py
import random, cocotb, numpy as np
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly

def pack_i8(v):  return int.from_bytes(np.asarray(v, np.int8).tobytes(), "little")

@cocotb.test()
async def test_gemv_vs_numpy(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    rng = np.random.default_rng(5)
    for M, K in ((8, 32), (16, 64), (64, 256)):
        W = rng.integers(-128, 128, (M, K), dtype=np.int8)
        x = rng.integers(-128, 128, K, dtype=np.int8)
        spm = {i: pack_i8(x[32*i:32*(i+1)]) for i in range(K // 32)}
        dut.k_rst_n.value = 0; dut.k_start.value = 0; dut.k_s_tvalid.value = 0
        await RisingEdge(dut.clk); dut.k_rst_n.value = 1
        X_AT, Y_AT = 0, 100
        dut.k_arg.value = (K << 96) | (M << 64) | (Y_AT << 32) | X_AT
        dut.k_start.value = 1; await RisingEdge(dut.clk); dut.k_start.value = 0
        beats = [pack_i8(W[m, 32*j:32*(j+1)]) for m in range(M) for j in range(K // 32)]
        prng = random.Random(M); i = 0; rd_pending = None; done = False
        while not done:
            dut.k_s_tvalid.value = 1 if (i < len(beats) and prng.random() < 0.8) else 0
            dut.k_s_tdata.value = beats[i] if i < len(beats) else 0
            dut.k_s_tlast.value = 1 if i == len(beats) - 1 else 0
            dut.k_spm_rdata.value = spm.get(rd_pending, 0) if rd_pending is not None else 0
            await ReadOnly()
            acc = int(dut.k_s_tvalid.value) and int(dut.k_s_tready.value)
            rd_pending = None
            if int(dut.k_spm_en.value):
                a = int(dut.k_spm_addr.value)
                if int(dut.k_spm_we.value):
                    old = spm.get(a, 0); st = int(dut.k_spm_wstrb.value); wd = int(dut.k_spm_wdata.value)
                    for b in range(32):
                        if st >> b & 1: old = (old & ~(0xFF << 8*b)) | (wd & (0xFF << 8*b))
                    spm[a] = old
                else:
                    rd_pending = a
            done = int(dut.k_done.value) == 1
            assert int(dut.k_err.value) == 0
            await RisingEdge(dut.clk)
            if acc: i += 1
        y = (W.astype(np.int64) @ x.astype(np.int64)).astype(np.int32)
        got = [np.int32(np.uint32((spm.get(Y_AT + m // 8, 0) >> (32 * (m % 8))) & 0xFFFFFFFF))
               for m in range(M)]
        assert list(got) == list(y), f"M={M} K={K}"
```

Run: `./run.sh gemv_kernel test_gemv "../rtl/kshell_pkg.sv ../rtl/gemv_kernel.sv"` → ошибка сборки.

- [ ] **Step 2: написать `gemv_kernel.sv`** по §15 (FSM IDLE → RUN → FLUSH → DONE; опережающее чтение `x`; дерево сумматоров 32 → 1 с конвейером; запись `y` с `wstrb` на 32-битной дорожке; пауза `tready` на такт записи).
- [ ] **Step 3: `kshell_kif_sva.sv`** и bind.
- [ ] **Step 4:** тест проходит (с `SVA_EXTRA=../rtl/kshell_kif_sva.sv`); `make lint`.
- [ ] **Step 5: коммит** `git commit -m "gemv_kernel + kshell_kif_sva (исполняемый контракт ядра)"`

---

### Task 8: `kshell_top`; системные T3 и T4

**Files:**
- Create: `rtl/kshell_top.sv` (`kshell` + `gemv_kernel`; параметры `NTAG` и `WDOG_CYCLES` пробрасываются в `kshell`; `tb/kshell_sink_top.sv` — так же)
- Modify: `tb/test_system.py` (добавить T3, T4)

- [ ] **Step 1: тесты T3, T4 (падают)**

```python
# tb/test_system.py — часть 2
def gemv_case(rng, M, K):
    W = rng.integers(-128, 128, (M, K), dtype=np.int8)
    x = rng.integers(-128, 128, K, dtype=np.int8)
    return W, x, (W.astype(np.int64) @ x.astype(np.int64)).astype(np.int32)

@cocotb.test(skip=TOP != "kshell_top")
async def t3_gemv(dut):
    ax, mem = await bringup(dut, lat=(10, 120), seed=3)
    rng = np.random.default_rng(3)
    for M, K in ((64, 256), (128, 512)):
        W, x, y = gemv_case(rng, M, K)
        mem.write_bytes(0x1000000, W.tobytes()); mem.write_bytes(0x2000000, x.tobytes())
        p = Program(); p.load(0, 0, 0x2000000, K); p.stream(0x1000000, M * K); p.wait(0, 1)
        p.run(1, 0, 64, M, K); p.wait(1, 1); p.store(2, 64, 0x3000000, 4 * M); p.wait(2, 1)
        p.signal(7, 1); check_program(p)
        await shell.submit(ax, p); await shell.wait_sem(ax, 7, 1)
        got = np.frombuffer(mem.read_bytes(0x3000000, 4 * M), np.int32)
        assert (got == y).all(), f"M={M} K={K}"
        await shell.soft_reset(ax); await ax.write(shell.REG["CTRL"], 1)

def overlapped(M, K):           # §6.2
    p = Program()
    p.load(0, 0, 0x2000000, K); p.stream(0x1000000, M * K); p.wait(0, 1); p.run(1, 0, 64, M, K)
    p.load(0, 16, 0x2100000, K); p.stream(0x1000000, M * K); p.wait(0, 2); p.run(1, 16, 72, M, K)
    p.wait(1, 1); p.store(2, 64, 0x3000000, 4 * M)
    p.wait(1, 2); p.store(2, 72, 0x3100000, 4 * M); p.wait(2, 2); p.signal(7, 1)
    return p

def sequential(M, K):           # базовая линия: WAIT после каждой команды
    p = Program()
    p.load(0, 0, 0x2000000, K); p.wait(0, 1); p.stream(0x1000000, M * K); p.run(1, 0, 64, M, K); p.wait(1, 1)
    p.load(0, 16, 0x2100000, K); p.wait(0, 2); p.stream(0x1000000, M * K); p.run(1, 16, 72, M, K); p.wait(1, 2)
    p.store(2, 64, 0x3000000, 4 * M); p.wait(2, 1); p.store(2, 72, 0x3100000, 4 * M); p.wait(2, 2)
    p.signal(7, 1)
    return p

@cocotb.test(skip=TOP != "kshell_top")
async def t4_overlap(dut):
    ax, mem = await bringup(dut, lat=(40, 160), seed=4)
    rng = np.random.default_rng(4); M, K = 64, 512
    W, x0, y0 = gemv_case(rng, M, K); _, x1, _ = gemv_case(rng, M, K)
    y1 = (W.astype(np.int64) @ x1.astype(np.int64)).astype(np.int32)
    mem.write_bytes(0x1000000, W.tobytes()); mem.write_bytes(0x2000000, x0.tobytes())
    mem.write_bytes(0x2100000, x1.tobytes())
    res = {}
    for name, fn in (("seq", sequential), ("ovl", overlapped)):
        p = fn(M, K); check_program(p)
        await shell.submit(ax, p); await shell.wait_sem(ax, 7, 1)
        res[name] = await shell.perf(ax)
        assert (np.frombuffer(mem.read_bytes(0x3000000, 4 * M), np.int32) == y0).all()
        assert (np.frombuffer(mem.read_bytes(0x3100000, 4 * M), np.int32) == y1).all()
        await shell.soft_reset(ax); await ax.write(shell.REG["CTRL"], 1)
    assert res["ovl"]["CYCLES"] < res["seq"]["CYCLES"]
    assert res["ovl"]["KRN_BUSY"] / res["ovl"]["CYCLES"] > res["seq"]["KRN_BUSY"] / res["seq"]["CYCLES"]
    dut._log.info("seq=%s ovl=%s", res["seq"], res["ovl"])
```

- [ ] **Step 2: написать `kshell_top.sv`.**
- [ ] **Step 3:** `./run.sh kshell_top test_system "<rtl без *_sva/*_cov>"` → T3, T4 PASS; `make lint`.
- [ ] **Step 4: коммит** `git commit -m "kshell_top; T3 GEMV против numpy, T4 перекрытие против базовой линии"`

---

### Task 9: ошибки, пауза, watchdog — T5, T6, T8

**Files:**
- Modify: `tb/test_system.py`; при необходимости — исправления в `rtl/*`
- Modify: `rtl/kshell.sv` (только если тесты выявят расхождение со §12)

- [ ] **Step 1: тесты (часть могут сразу пройти — это нормально; падающие фиксируем)**

```python
# tb/test_system.py — часть 3
@cocotb.test(skip=TOP != "kshell_top")
async def t5_slverr_in_flight(dut):
    for target in ("load", "stream", "store"):
        ax, mem = await bringup(dut, lat=(10, 80), seed=5,
                                slverr={dict(load=0x2000000, stream=0x1000000 + 256 * 20,
                                             store=0x3000000)[target]})
        rng = np.random.default_rng(5); M, K = 64, 512
        W, x, y = gemv_case(rng, M, K)
        mem.write_bytes(0x1000000, W.tobytes()); mem.write_bytes(0x2000000, x.tobytes())
        p = Program(); p.load(0, 0, 0x2000000, K); p.stream(0x1000000, M * K); p.wait(0, 1)
        p.run(1, 0, 64, M, K); p.wait(1, 1); p.store(2, 64, 0x3000000, 4 * M); p.wait(2, 1); p.signal(7, 1)
        await shell.submit(ax, p)
        for _ in range(20000):
            await RisingEdge(dut.clk)
            if (await shell.status(ax))["ERROR"]: break
        assert await ax.read(shell.REG["ERR_CODE"]) == (2 if target == "store" else 1)
        assert await ax.read(shell.REG["SEM"] + 4 * 7) == 0           # программа не «прошла»
        await shell.soft_reset(ax)                                     # ждёт AXI_QUIET
        st = await shell.status(ax); assert st["IDLE"] and not st["ERROR"]
        mem.stop(); await shell.reset(dut)                             # следующий цикл с новой моделью

@cocotb.test(skip=TOP != "kshell_top")
async def t6_pause_resume(dut):
    ax, mem = await bringup(dut, lat=(10, 80), seed=6)
    rng = np.random.default_rng(6); M, K = 64, 512
    W, x0, y0 = gemv_case(rng, M, K); _, x1, _ = gemv_case(rng, M, K)
    y1 = (W.astype(np.int64) @ x1.astype(np.int64)).astype(np.int32)
    for a, d in ((0x1000000, W), (0x2000000, x0), (0x2100000, x1)): mem.write_bytes(a, d.tobytes())
    await shell.submit(ax, overlapped(M, K))
    for _ in range(300): await RisingEdge(dut.clk)
    await ax.write(shell.REG["CTRL"], 0)                               # пауза
    for _ in range(5000):
        await RisingEdge(dut.clk)
        if (await shell.status(ax))["AXI_QUIET"]: break
    assert (await shell.status(ax))["AXI_QUIET"]
    await ax.write(shell.REG["CTRL"], 1)                               # продолжение
    await shell.wait_sem(ax, 7, 1)
    assert (np.frombuffer(mem.read_bytes(0x3000000, 4 * M), np.int32) == y0).all()
    assert (np.frombuffer(mem.read_bytes(0x3100000, 4 * M), np.int32) == y1).all()

@cocotb.test(skip=TOP != "kshell_top")
async def t8_watchdog(dut):
    # сборка с -GWDOG_CYCLES=2000 (см. команду ниже)
    ax, mem = await bringup(dut, lat=(10, 40), seed=8)
    p = Program(); p.wait(3, 1)                                        # долгий WAIT — не неисправность
    await shell.submit(ax, p)
    for _ in range(5000): await RisingEdge(dut.clk)
    assert not (await shell.status(ax))["ERROR"]
    await ax.write(shell.REG["SEM"] + 4 * 3, 1)
    mem.write_bytes(0x2000000, bytes(256 * 8))
    p = Program(); p.load(0, 0, 0x2000000, 256 * 8); await shell.submit(ax, p)
    for _ in range(30): await RisingEdge(dut.clk)
    mem.freeze()
    for _ in range(5000): await RisingEdge(dut.clk)
    assert await ax.read(shell.REG["ERR_CODE"]) == 7
```

Run: `./run.sh kshell_top test_system "<rtl>"` (для T8 — `EXTRA_ARGS="-GWDOG_CYCLES=2000"` через `make … EXTRA_ARGS+=…`, `kshell_top` пробрасывает `WDOG_CYCLES`).

- [ ] **Step 2: исправить RTL по найденным расхождениям со §12** (каждое исправление — со ссылкой на пункт спецификации в сообщении коммита).
- [ ] **Step 3:** весь `test_system` на `kshell_top` проходит; `make lint`.
- [ ] **Step 4: коммит** `git commit -m "T5 SLVERR в полёте, T6 пауза/продолжение, T8 watchdog"`

---

### Task 10: случайные программы (T9) и функциональное покрытие

**Files:**
- Modify: `tb/isa.py` (генератор `random_program(rng, n_cmds, spm_words, hbm_base)`)
- Modify: `tb/test_system.py` (T9)
- Create: `rtl/rd_dma_cov.sv`, `rtl/cmd_seq_cov.sv`, `rtl/kshell_cov.sv`

**Генератор (§16):** только LOAD/STORE/SIGNAL/WAIT; `WAIT sem ≥ N`, где N ≤ уже выданных приращений; трекинг областей spm и памяти; перед пересекающейся командой — барьер `WAIT sem ≥ всего выданного` на семафор предыдущей команды; в конце — общий барьер и `SIGNAL sem7`. Эталон — Python-исполнение программы над копией памяти и spm.

- [ ] **Step 1: тест T9**

```python
# tb/test_system.py — часть 4
from isa import random_program, execute_reference

@cocotb.test(skip=TOP != "kshell_sink_top")
async def t9_random_programs(dut):
    for seed in range(int(os.environ.get("T9_SEEDS", 20))):
        ax, mem = await bringup(dut, lat=(5, 120), seed=seed, interleave=bool(seed % 2))
        rng = random.Random(seed)
        init = bytes(rng.randrange(256) for _ in range(1 << 16))
        mem.write_bytes(0x500000, init)
        p = random_program(rng, n_cmds=40, spm_words=1024, hbm_base=0x500000, hbm_size=1 << 16)
        check_program(p, run_regions=None)
        ref_mem, ref_sem = execute_reference(p, init, base=0x500000)
        await shell.submit(ax, p); await shell.wait_sem(ax, 7, 1)
        assert mem.read_bytes(0x500000, 1 << 16) == ref_mem
        for i in range(7): assert await ax.read(shell.REG["SEM"] + 4 * i) == ref_sem[i]
        mem.stop(); await shell.reset(dut)
```

- [ ] **Step 2: реализовать `random_program` и `execute_reference` в `isa.py`; прогнать T9 (20 сидов).**
- [ ] **Step 3: covergroup'ы (bind, `_r`-снимки как в tick_to_trade, явный `sample()`):**
  - `rd_dma_cov`: окно полно; расстояние OoO `(rid − head) mod NTAG` — бины 0, 1–3, 4–7, 8+; чередование beat'ов разных ID; head-of-line (голова STREAM стоит при `done` у следующего LOAD);
  - `cmd_seq_cov`: все коды команд; стоянка на WAIT; WAIT прерван ошибкой;
  - `kshell_cov`: все коды ошибок; пауза во время трафика; W-FIFO полна; ≥ 2 приращения одного семафора за такт.
- [ ] **Step 4: прогон покрытия на всех системных тестах**

Run: `make cov TOP=kshell_top MODULE=test_system SRCS="<rtl>"` (и для `kshell_sink_top`). Expected: все бины covergroup'ов > 0; дыры — добавить стимул в T9 или отдельный короткий тест.

- [ ] **Step 5: коммит** `git commit -m "T9 случайные программы против эталона; функциональное покрытие"`

---

### Task 11: демо и свип NTAG × задержка

**Files:**
- Create: `scripts/demo.sh`, `tb/test_demo.py`

`test_demo.py`:
- `demo_overlap` — программа §6.2 и последовательная, печать таблицы perf (средняя задержка `RD_LAT_SUM/RD_BURSTS`, `KRN_BUSY/CYCLES`, `STRM_BP`, `SEQ_WAIT`, время) в `results/demo.md`;
- `demo_sweep` — для фиксированной задержки модели (`lat=(L, L)`, L из `DEMO_LAT`) один STREAM 64 KB в `sink_kernel` с `tready = 1`; полоса = `beat'ов / тактов` от первого AR до последнего beat'а; дописывает строку в `results/sweep.csv`.

`demo.sh`: для `NTAG in 4 8 16` и `L in 20 50 100 150 200 300` — пересборка `kshell_sink_top` с `-GNTAG=$NTAG`, прогон `demo_sweep`; затем `demo_overlap` на `kshell_top`; итог — `results/demo.md` с таблицами.

- [ ] **Step 1:** написать `test_demo.py` и `demo.sh`.
- [ ] **Step 2: прогон**

Run: `scripts/demo.sh`
Expected: в `results/sweep.csv` полоса ≈ 1.0 при `L ≲ NTAG × BURST` и падает примерно как `NTAG × BURST / L` дальше; в `results/demo.md` время с перекрытием меньше последовательного.

- [ ] **Step 3: коммит** `git commit -m "демо: отчёт perf и свип NTAG × задержка"`

---

### Task 12: README и публикация

**Files:**
- Create: `README.md` (английский — для публичного репозитория; дизайн-документ остаётся на русском)

Содержание README: что это и зачем (обёртка ядра для inference-ускорителя); схема §2; контракт ядра §5; таблица команд §6 и пример программы §6.2; ключевые решения §17 (сокращённо); как запустить (`scripts/run.sh`, `scripts/demo.sh`); **результаты из `results/demo.md`** — таблица перекрытия и график/таблица свипа; честная пометка «simulation with a memory model, not silicon»; «written with AI assistance; architecture and verification decisions are mine».

- [ ] **Step 1:** написать README по `results/*`.
- [ ] **Step 2:** финальный прогон всего: `make lint`, все тесты (`test_prims` × 3, `test_isa`, `test_rd_dma`, `test_wr_dma`, `test_cmd_seq`, `test_gemv`, `test_system` на обоих top), `scripts/demo.sh`.
- [ ] **Step 3: коммит** `git commit -m "README с результатами демо"`
- [ ] **Step 4:** публикация на GitHub — **только по явной команде пользователя** (`gh repo create … --public`).

---

## Самопроверка плана (выполнена)

- **Покрытие спецификации:** §5 → Task 6/7 (контракт, `kshell_kif_sva`); §6/§6.1 → Task 2 (`isa`, `check_program`), Task 5 (декодирование); §7 → Task 5; §8 → Task 3; §9 → Task 4; §10–§11 → Task 1; §12 → Task 6 (логика), Task 9 (тесты); §13–§14 → Task 6; §15 → Task 7; §16 T1–T9, SVA, покрытие, демо → Tasks 3–11; README → Task 12.
- **Согласованность имён:** порты AXI `m_axi_*`, поток `k_s_*`, spm порт A `spm_wr_*`/`spm_rd_*` (в `spm` — `a_wr_*`/`a_rd_*`, соединяются в `kshell`), Python-API `Program`, `check_program(prog, ntag, run_regions)`, `shell.REG`, `shell.PERF_NAMES` — одинаковы во всех задачах.
- **Известное допущение:** RTL-модули в плане заданы портами и требованиями со ссылками на спецификацию, а не полным кодом — код пишется при выполнении задачи под гейтами lint + тесты + SVA.
