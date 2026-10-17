# kshell: a universal kernel shell for an FPGA inference accelerator

`kshell` is the glue between a compute kernel and the top level of an FPGA accelerator.
The kernel gets a **narrow contract**: an AXI4-Stream in, a scratchpad port, `start`/`done`/`err`, and a reset.
The shell owns everything else: AXI4 to HBM, outstanding transactions, out-of-order read responses restored to in-order, semaphore-based synchronization between engines, errors, safe pause/reset, and performance counters.
A new kernel is a black box that is checked against an executable contract (`kshell_kif_sva`), not a block that re-implements memory plumbing.

**Status: simulation only** (Verilator + cocotb, Python AXI memory model). No synthesis, no timing closure, no silicon. All numbers below are simulation clock counts. See [Limitations and next steps](#limitations-and-next-steps).

Spec (Russian): [`docs/design.md`](docs/design.md). RTL style: [`docs/style_notes.md`](docs/style_notes.md).

## Architecture

```
                              kshell_top
 +--------------------------------------------------------------------------+
 |  kshell                                                                  |
 |  +----------+ cmd +---------+  rd-q  +------------------------+          |
 |  | axil_regs|---->| cmd_seq |------->| rd_dma                 |<-- R ----+-- m_axi
 |  | CTRL/STAT|     |         |  wr-q  |  slicer -> AR (ID=slot)|-- AR --->|   (AXI4, 256b)
 |  | CMD/DB   |     | WAIT <--+--sem-- |  NTAG-slot window      |          |
 |  | SEM/PERF |     |         |---+    |  LOAD   -> spm port A  |          |
 |  +----^-----+     +----+----+   |    |  STREAM -> ROB -> AXIS-+--+       |
 |       | s_axil         | krn-q  v    +------------------------+  |       |
 |       |                |    +----->  +------------------------+  |       |
 |  +----+-----+          |    |        | wr_dma  spm A -> AW/W  |--+-AW/W->|
 |  | sem_file |<--- +1 --+----+        |         B counting     |<-- B ----|
 |  +----------+          |             +------------------------+  |       |
 |  +----------+          |             +------------------------+  |       |
 |  |kshell_   |          |             | spm: 2-port SRAM       |  |       |
 |  |perf      |          |             | A = DMA, B = kernel    |  |       |
 |  +----------+          |             +----------^-------------+  |       |
 +------------------------+------------------------+----------------+-------+
    k_rst_n / k_start /   | k_done / k_err         | port B         | AXIS
    k_arg                 v                        |                v
                  +--------------------------------+---------------------+
                  | gemv_kernel (any kernel that obeys the same contract) |
                  +------------------------------------------------------+
```

| Module | Responsibility |
|---|---|
| `axil_regs` | AXI4-Lite slave, 128-bit descriptors into the command FIFO, doorbell |
| `cmd_seq` | decode and check commands, dispatch to engine queues in order, WAIT, stop on error |
| `rd_dma` | LOAD and STREAM: burst slicing, AR with `ARID` = window slot, in-order commit, ROB |
| `wr_dma` | STORE: scratchpad to W-FIFO first, then AW/W, count B responses |
| `spm` | true dual-port scratchpad written as the vendor BRAM template (one address and one `always_ff` per port): A = DMA, full-word writes; B = kernel, byte write strobes |
| `sem_file` | semaphore counters, same-cycle increments are summed |
| `kshell_perf` | 8 perf counters with an atomic snapshot |
| `kshell_kif_sva` | the kernel contract as bindable assertions |

Defaults: `DW=256`, `AW=40`, `NTAG=16` outstanding read bursts, `BURST=8` beats (256 B), `WR_OUT=8`, 32 KB scratchpad, 8 semaphores.

## Kernel contract

| Group | Signals | Rule |
|---|---|---|
| Reset | `k_rst_n` | synchronous, driven by the shell (`rst_n && !soft_reset`) |
| Control | `k_start` (1-cycle pulse), `k_arg[4][31:0]`, `k_done`, `k_err` | exactly one `k_done` per `k_start`; no new `k_start` before it; after `k_err` the kernel may stay silent, recovery is `SOFT_RESET` |
| Stream | `k_s_tdata/tvalid/tready/tlast` (AXI4-Stream, shell to kernel) | one RUN consumes exactly one STREAM command, in order; `tlast` on the last beat; `tready = 0` outside `k_start..k_done` |
| Scratchpad | `k_spm_en/we/addr/wdata/wstrb`, `k_spm_rdata` | port B belongs to the kernel only; read data one cycle after the request |

The kernel does not see semaphores, AXI IDs, or addresses in memory; argument meaning is the kernel's business. `kshell_kif_sva`, bound onto any kernel, checks the safety part of these rules: at most one `k_done` per `k_start` (and only for a started RUN); `k_done` only after the RUN's `tlast` was accepted (or after `k_err`); `tready` only inside the RUN window and not after `tlast`; no port-B access in a reset cycle. Liveness (that `k_done` eventually comes) is not an assertion; test timeouts catch it.

## Programming model

The host writes four 32-bit words `CMD0..CMD3` (one 128-bit descriptor) and rings `DOORBELL`. Engines run in parallel; the only ordering between them is what the program states with semaphores.

| op | Command | Semantics | `sem += 1` when |
|---|---|---|---|
| 1 | `LOAD sem, spm, hbm, len` | memory to scratchpad | all bursts arrived and committed in order, no error |
| 2 | `STREAM hbm, len` | memory to the kernel's AXIS | never (no semaphore; completes when the kernel accepts the last beat) |
| 3 | `STORE sem, spm, hbm, len` | scratchpad to memory | all B responses received, no error |
| 4 | `WAIT sem, thr` | sequencer stalls until `$signed(sem - thr) >= 0` | n/a |
| 5 | `SIGNAL sem, inc` | `sem += inc`, not a barrier | immediately |
| 6 | `RUN sem, a0..a3` | start the kernel | `k_done` |

Decode checks: `len` and `hbm` multiples of 256 B (`ALIGN`), scratchpad range (`RANGE`), opcode (`BAD_OPCODE`). The first error aborts: the sequencer stops, queues are flushed, in-flight AXI transactions are *completed* (none is abandoned), and the host does `SOFT_RESET` once `AXI_QUIET=1`.

Program correctness rules (no head-of-line deadlock, STREAM/RUN pairing, WAIT between conflicting commands) are checked statically by `tb/isa.py: check_program`. All test programs go through it except the deliberate negative tests (decode errors, the watchdog tests that issue a RUN before its STREAM on purpose, error-priority and kernel-error programs).

`CTRL` is written as a whole register, not bit-set/clear: `ENABLE` takes bit 0 of every write. So `CTRL = 0x2` (`SOFT_RESET`) or `0x4` (`PERF_SNAP`) also clears `ENABLE`; write `ENABLE | SOFT_RESET` (`0x3`) or `ENABLE | PERF_SNAP` (`0x5`) to keep running.

### Example: two GEMV tasks with overlap

`W` is shared, `x0`/`x1` differ. While RUN1 computes, `x1` loads and the weights for RUN2 already stream; STORE of `y0` overlaps RUN2.

```
LOAD   sem0, spm=XA, hbm=x0, len=K
STREAM hbm=W, len=M*K                ; weights run ahead, filling the window and ROB
WAIT   sem0 >= 1
RUN    sem1, x=XA, y=YA, M, K
LOAD   sem0, spm=XB, hbm=x1, len=K   ; loads while RUN1 computes
STREAM hbm=W, len=M*K
WAIT   sem0 >= 2                     ; RUN1 already issued, so STREAM1 can advance
RUN    sem1, x=XB, y=YB, M, K
WAIT   sem1 >= 1
STORE  sem2, spm=YA, hbm=y0, len=4*M ; stores while RUN2 computes
WAIT   sem1 >= 2
STORE  sem2, spm=YB, hbm=y1, len=4*M
WAIT   sem2 >= 2
SIGNAL sem7, 1                       ; host polls SEM[7]
```

## Key design decisions

| Decision | Why | Cost of the alternative |
|---|---|---|
| Kernel has no AXI: stream + scratchpad + start/done + reset | memory complexity lives once in the shell; kernels can be delegated as black boxes and checked by `kshell_kif_sva` | AXI in every kernel: repeated tag/OoO/error logic |
| Tag = slot number of the window | in-order commit comes for free, like a ROB; no tag storage | free-list tags need a separate reorder buffer |
| LOAD writes out of order, its semaphore commits in order | no data reordering memory; the consumer only sees a contiguous ready prefix | reordering LOAD data costs memory |
| `RREADY = 1` always | room is reserved up front (spm region or ROB row), memory is never stalled by the shell | R backpressure wastes memory bandwidth |
| W-FIFO fills before AW | the interconnect write channel is never held by the shell | AW first: lower latency, but W can stall behind an accepted AW |
| One `AWID`, `AWCACHE=0010` | writes are not reordered; B means "in memory", so a semaphore never lies | bufferable writes: faster, but B can precede the data |
| One window shared by LOAD and STREAM | simpler, one tag pool | separate windows remove head-of-line blocking at twice the state |
| Synchronization only via semaphores + WAIT, statically checked | explicit dependencies, parallel engines (NPU/TPU command-stream style) | implicit hardware dependencies: more complex, less flexible |
| Abort drains AXI; reset only at `AXI_QUIET` | nothing is abandoned; no epoch bit in IDs; the kernel resets through its own `k_rst_n` | reset at any time needs a decoupler or "zombie slots" |

The head-of-line cost of the shared window is real and deliberate: a stalled STREAM at the head delays the commit of LOADs behind it (visible as `STRM_BP`; functional-coverage bin `hol`).

## Verification

Toolchain: Verilator 5 (`--assert`, and `--coverage` in separate builds) and cocotb 2. Lint is Verilator `-Wall` plus Verible.

- **Python models** (`tb/`): an AXI4 slave memory model with 8 pseudo-channels, random per-channel latency (so responses are **out of order by ID**), random `ARREADY/AWREADY/WREADY` stalls, optional beat interleaving between IDs, `SLVERR` injection, a "stop responding" mode, and protocol checks (`WLAST`, `AxLEN`, 4 KB boundary). An AXI4-Lite driver. A program builder and a **static program checker**. A NumPy GEMV reference.
- **System tests T1 to T9**: memcpy through the scratchpad; STREAM into a sink kernel (OoO order, `tlast`, skid at `tready=0`); GEMV against NumPy; the overlap program against its sequential baseline; `SLVERR` injected into LOAD, STREAM and STORE with the kernel mid-RUN (error code, `ERR_CMD`, WAIT aborted, semaphores frozen, `AXI_QUIET`, `SOFT_RESET`, next program correct), plus `SLVERR` on a LOAD and on a STORE as the first error on the sink-kernel top; pause and resume mid-program; decode errors and command-FIFO overflow; watchdog on a dead memory (and no false trigger on pause or a long WAIT); **random LOAD/STORE/SIGNAL/WAIT programs compared against a sequential Python reference** (memory, scratchpad and semaphores, 20 seeds). Plus module tests for the primitives (including the scratchpad's BRAM-template read-hold and byte strobes), `rd_dma`, `wr_dma`, `cmd_seq`, `gemv_kernel`, and 37 pytest cases for the program builder, checker, generator and memory model.
- **SVA** in bind files: AXI master protocol (valid/payload stability, `AxLEN`, W-beat count; the 4 KB rule follows from 256 B-aligned bursts, which the decoder enforces, and the memory model checks it on every AR/AW), the rd window (R only for in-flight slots, `RLAST` consistency, `tail-head <= NTAG`), AXIS stability, scratchpad port conflicts (only when a side writes; A-read vs B-read is legal), `SOFT_RESET` only at `AXI_QUIET`, and **`kshell_kif_sva`, the executable kernel contract** (the four checks listed under [Kernel contract](#kernel-contract)), bound onto both `gemv_kernel` and the test sink kernel. A mutated sink kernel that raises `k_done` one beat before `tlast` used to pass every SVA (the program even completed, and only the end-to-end data compare noticed); the `k_done`-after-`tlast` assertion now fires at that `k_done`.
- **Functional coverage**: 30 bins: window full, OoO distance bins (0, 1-3, 4-7, 8+), beat interleaving, head-of-line blocking, all opcodes, WAIT stall and WAIT aborted by error, all seven error codes, pause under traffic, W-FIFO full, simultaneous increments on one semaphore. **`kshell_sink_top` alone hits 30/30**; `kshell_top` (GEMV) hits 17/30, since the decode-error, interleaving, `hol` and same-semaphore bins are driven by sink-top tests; the union is 30/30. Three directed tests exist only to close bins that random traffic did not reach (`hol`, same-semaphore collisions, and `SLVERR` as the first error on the sink top). Code coverage, sink top / GEMV top / union: line 91.1 / 90.6 / 94.2 %, branch 97.4 / 93.9 / 95.9 %. Also checked by hand-applied mutations (for example, committing a LOAD before its last beat makes the random-program test fail).

## Results

**Simulation with a memory model, not silicon.** Numbers are simulator clock counts (1 unit = 1 cycle, no frequency) and describe the structure of the shell, not measured speed on a board. Full tables: [`results/demo.md`](results/demo.md), raw data: [`results/sweep.csv`](results/sweep.csv), [`results/overlap.csv`](results/overlap.csv).

### Command overlap (section 6.2 program vs fully sequential)

GEMV M=64, K=512, two tasks, memory latency 40 to 160 cycles, 20 % random ready stalls (one fixed seed).

| program | CYCLES | KRN_BUSY | avg read latency | STRM_BP | SEQ_WAIT |
|---|---:|---:|---:|---:|---:|
| sequential (WAIT after every command) | 3378 | 87.4 % | 114.4 | 108 | 3307 |
| overlapped (section 6.2) | 3064 | 91.0 % | 113.0 | 121 | 2937 |

Speedup 1.10x (about 10 %). The gain is modest because of the workload: a 64x512 GEMV is bound by the W stream (32 KB per task), the kernel is already busy most of the time, and overlap only hides the `x` load and the `y` store. This is a single seed, not a distribution.

### STREAM bandwidth vs window size and memory latency

One 32 KB STREAM (1024 beats) into a sink kernel that is always ready; memory latency fixed at `L` cycles, no stalls. Steady-state bandwidth in beats per cycle (ceiling 1.0), measured over the middle 512 beats.

| NTAG (window) \ L, cycles | 20 | 50 | 100 | 150 | 200 | 300 |
|---|---:|---:|---:|---:|---:|---:|
| 4 (32 beats) | 0.889 | 0.485 | 0.276 | 0.193 | 0.148 | 0.101 |
| 8 (64 beats) | 1.000 | 0.970 | 0.552 | 0.386 | 0.296 | 0.203 |
| 16 (128 beats) | 1.000 | 1.000 | 1.000 | 0.771 | 0.593 | 0.405 |

This is Little's law: the window holds `NTAG x BURST` beats, so bandwidth is `NTAG x BURST / (slot hold time)`, capped at 1 beat per cycle. The knee is visible per row: NTAG=16 stays at 1.0 up to L=100 and drops at L=150.

The slot hold time follows from the RTL (spec section 8.5). A STREAM head becomes ready only after its `RLAST` (`h_ready = w_done_q[hslot]` in `rtl/rd_dma.sv`), and the slot retires only when the last of its BURST beats has left the ROB for the stream (`rd_end`). With the first beat `L` cycles after AR: AR to `RLAST` is `L + BURST - 1`, plus 1 cycle to set `done`, plus `BURST` cycles to drain. So the head advances `L + 2 x BURST` cycles after its AR, which gives `min(1, NTAG x BURST / (L + 2 x BURST))` with the knee at `L + 2 x BURST <= NTAG x BURST`. The measurement matches this derived curve at all 18 points (largest deviation 0.03 %; the test allows 3 %). The naive bound `NTAG x BURST / (L + BURST)`, which assumes the slot frees at `RLAST`, is a ceiling that the measurement never exceeds (also asserted).

Next step (not implemented): cut-through drain of the head burst, sending each beat to the stream as soon as it lands in the ROB instead of waiting for `RLAST`. That would cut the slot hold time to about `L + BURST`, i.e. up to the naive bound.

## How to run

Requirements: Verilator 5, a Python venv with `cocotb` 2, `numpy`, `pytest`, and Verible for lint. `scripts/run.sh` activates the venv at `$FPGA_VENV` (default `~/fpga-venv`) if it exists and wraps the run in a watchdog (`SIM_TIMEOUT`, default 600 s), because a failed SVA can leave the simulator hanging.

```bash
cd scripts
make lint                                   # Verible + Verilator -Wall on the design files

# Python-only tests: program builder, static checker, random-program generator, memory model
~/fpga-venv/bin/python -m pytest ../tb/test_isa.py ../tb/test_hbm_model.py -q

# Everything: lint, pytest, all module tests, both system tops with all SVA binds
# (T9 with 20 seeds). Exact per-step commands are in the script; logs go to regress_<step>.log
./regress.sh
./regress.sh cov          # + coverage builds of both tops and a per-top / union covergroup summary

# One module test by hand (run.sh <TOP> <test module> "<sources>")
./run.sh rd_dma test_rd_dma "../rtl/kshell_pkg.sv ../rtl/fifo_sync.sv ../rtl/rd_dma.sv"

# One system top by hand: kshell_top (GEMV), all SVA binds, short watchdog
COMMON="../rtl/kshell_pkg.sv ../rtl/fifo_sync.sv ../rtl/sem_file.sv ../rtl/spm.sv ../rtl/rd_dma.sv ../rtl/wr_dma.sv ../rtl/cmd_seq.sv ../rtl/axil_regs.sv ../rtl/kshell_perf.sv ../rtl/kshell.sv"
SVA="../rtl/rd_dma_sva.sv ../rtl/wr_dma_sva.sv ../rtl/cmd_seq_sva.sv ../rtl/kshell_sva.sv ../rtl/kshell_kif_sva.sv"
SVA_EXTRA="$SVA ../rtl/gemv_kernel_sva.sv" EXTRA_ARGS="-GWDOG_CYCLES=2000" SIM_TIMEOUT=900 \
  ./run.sh kshell_top test_system "$COMMON ../rtl/gemv_kernel.sv ../rtl/kshell_top.sv"

# The sink-kernel top, with 20 random-program seeds
T9_SEEDS=20 SVA_EXTRA="$SVA ../tb/sink_kernel_sva.sv" EXTRA_ARGS="-GWDOG_CYCLES=2000" SIM_TIMEOUT=900 \
  ./run.sh kshell_sink_top test_system "$COMMON ../tb/sink_kernel.sv ../tb/kshell_sink_top.sv"

# Demo: NTAG x latency sweep + overlap comparison, writes ../results/ (a few minutes)
./demo.sh

# Functional + code coverage of one top (separate build from the assertion runs)
COV_EXTRA="../rtl/rd_dma_cov.sv ../rtl/cmd_seq_cov.sv ../rtl/kshell_cov.sv" EXTRA_ARGS="-GWDOG_CYCLES=2000" \
  make cov TOP=kshell_top MODULE=test_system SRCS="$COMMON ../rtl/gemv_kernel.sv ../rtl/kshell_top.sv"
```

Note: cocotb runs a test that matches `COCOTB_TEST_FILTER` even when the test is marked `skip` for the other top, so filter only by tests that belong to the top you build.

Assertion runs and coverage runs are separate builds because Verilator 5.x hits an internal error with `--assert` and `--coverage` together.

## Limitations and next steps

Deliberately out of scope: synthesis and timing closure, clock-domain crossing (single clock), multiple kernels, interconnect, credit-based links, interrupts (the host polls), kernel-side access by computed addresses, overlap *inside* a kernel (the kernel does not see semaphores), and reset with unfinished AXI transactions (a real system needs a decoupler for that).

Also worth saying plainly:
- The memory model is a Python model with random latency, not an HBM controller; absolute numbers say nothing about a real device.
- The kernel is a small INT8 GEMV used to exercise the contract, not an optimized accelerator.
- The program correctness rules (section 6.1 of the spec) are the host's responsibility; the checker catches violations in tests but hardware does not enforce them. A broken program deadlocks, and the watchdog only covers a silent memory.
- The overlap speedup is one seed on one workload.

Known timing weak spots (found in review; the RTL is deliberately left as is, the project is simulation-only). These are the first things to fix before timing closure:
- **AXI R feeds the scratchpad combinationally.** `rvalid`/`rid` drive the spm write address (a 16:1 mux by slot plus an adder) and the port-A grant to `wr_dma` in the same cycle. Next step: one register stage on R inside `rd_dma`. Since `RREADY = 1`, no skid buffer is needed; the cost is 1 cycle of latency.
- **`soft_rst` / `k_rst_n` are combinational** (the AXI-Lite write pulse AND `AXI_QUIET`). Next step: register `soft_rst` and hold `go` low for the pending cycle; latch a request made while not quiet instead of ignoring it.
- **Kernel-to-shell signals** (`k_done`, `k_err`, `k_s_tready`) enter shell logic unregistered, so the path length depends on the kernel. Next step: require flop-driven kernel outputs in the contract (spec section 5), or register them at the boundary (a skid buffer for `tready`).
- **STREAM slot hold** is `L + 2 x BURST` (see Results). Next step: cut-through drain of the head burst.

## Layout

```
rtl/        shell modules, gemv_kernel, kshell_top, *_sva.sv and *_cov.sv bind files
tb/         cocotb tests, hbm_model.py, axil.py, isa.py (builder + static checker), test kernel
scripts/    Makefile, run.sh (build + run one test), regress.sh (full regression), demo.sh, demo_report.py
results/    demo.md, sweep.csv, overlap.csv
docs/       design.md (specification, Russian), style_notes.md
```

## Authorship

Written with AI assistance; architecture, specification and verification decisions are mine.
