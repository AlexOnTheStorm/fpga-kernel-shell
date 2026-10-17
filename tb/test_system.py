# Системные тесты kshell (docs/design.md §16): хост — AxilMaster + shell.py, память — HbmModel
# (OoO по ID). Тест выбирается по имени верхнего уровня (kshell_sink_top / kshell_top).
#   T1 memcpy LOAD → WAIT → STORE; T2 STREAM в sink_kernel (порядок при OoO, tlast, skid);
#   T7 ошибки декодирования + переполнение командной FIFO;
#   T3 GEMV, T4 перекрытие; T5 SLVERR в полёте, T6 пауза/продолжение, T8 watchdog (kshell_top);
#   T9 случайные программы LOAD/STORE/SIGNAL/WAIT против Python-эталона (kshell_sink_top);
#   t_slverr_sink — SLVERR первой ошибкой на LOAD и STORE (kshell_sink_top, бины RRESP/BRESP).
# Дополнительно к брифу (Task 6): IDLE/perf/SOFT_RESET после программы, порядок AW/W в
# AXI-Lite, KERNEL_ERR от sink_kernel, watchdog, приоритет ошибок в одном такте нет (§12.2
# покрыт kshell_sva). Сборка: -GWDOG_CYCLES=2000 (R5), все module SVA через SVA_EXTRA (R4).
import os
import random

import cocotb
import numpy as np  # noqa: F401  (части 2–4 плана: эталон GEMV)
from cocotb.triggers import ReadOnly, RisingEdge

from axil import AxilMaster
from hbm_model import HbmModel
from isa import Program, check_program, random_program, execute_reference
import shell

TOP = os.environ.get("COCOTB_TOPLEVEL") or os.environ.get("TOPLEVEL")


async def bringup(dut, **mk):
    await shell.reset(dut)
    mem = HbmModel(dut, **mk); mem.start()
    ax = AxilMaster(dut); await ax.write(shell.REG["CTRL"], 1)   # ENABLE
    return ax, mem


async def sem(ax, i):
    return await ax.read(shell.REG["SEM"] + 4 * i)


async def settle(dut, ax, n=2000):
    """Ждать IDLE (вся работа выполнена) после того, как программа «прошла» семафор."""
    for _ in range(n // 16):
        st = await shell.status(ax)
        if st["IDLE"]:
            return st
        for _ in range(16):
            await RisingEdge(dut.clk)
    raise AssertionError(f"нет IDLE за {n} тактов: {await shell.status(ax)}")


class StallDriver:
    """Случайный backpressure sink_kernel (вход k_stall верхнего уровня): tready = 0 в доле тактов."""

    def __init__(self, dut, p, seed):
        self.dut, self.p, self.rng = dut, p, random.Random(seed)
        self.bp = 0
        self._task = cocotb.start_soon(self._run())

    async def _run(self):
        while True:
            self.dut.k_stall.value = int(self.rng.random() < self.p)
            await ReadOnly()
            if int(self.dut.k_s_tvalid.value) and not int(self.dut.k_s_tready.value):
                self.bp += 1
            await RisingEdge(self.dut.clk)

    def stop(self):
        self._task.cancel()
        self.dut.k_stall.value = 0


# --------------------------------------------------------------------------- T1, T2, T7 (бриф)

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
    # сверх брифа: OoO реально было; IDLE; семафоры; perf; SOFT_RESET всё обнуляет
    assert mem.stats["ooo"] > 0, mem.stats
    st = await settle(dut, ax)
    assert st["AXI_QUIET"] and not st["ERROR"] and not st["cq_ovf"] and not st["krn_busy"], st
    assert st["cq_free"] == 16 and st["rd_inflight"] == 0 and st["wr_inflight"] == 0, st
    assert [await sem(ax, i) for i in (0, 1, 7)] == [1, 1, 1]
    assert await ax.read(shell.REG["CMD_COUNT"]) == 5
    pf = await shell.perf(ax)
    dut._log.info("T1 perf %s", pf)
    assert pf["RD_BURSTS"] == 24 and pf["WR_BURSTS"] == 24, pf
    assert 5 * 24 <= pf["RD_LAT_SUM"] <= 400 * 24, pf          # задержка ≥ min модели
    assert pf["SEQ_WAIT"] > 0 and pf["CYCLES"] > pf["SEQ_WAIT"], pf
    assert pf["STRM_BP"] == 0 and pf["KRN_BUSY"] == 0 and pf["RD_WIN_FULL"] <= pf["CYCLES"], pf
    pf2 = await shell.perf(ax)                                  # снимок атомарный и растёт
    assert pf2["CYCLES"] > pf["CYCLES"] and pf2["RD_BURSTS"] == 24, pf2
    await shell.soft_reset(ax)
    st = await shell.status(ax)
    assert st["IDLE"] and not st["ERROR"], st
    assert [await sem(ax, i) for i in range(8)] == [0] * 8
    assert await ax.read(shell.REG["CMD_COUNT"]) == 0
    pf3 = await shell.perf(ax)
    assert all(v == 0 for v in pf3.values()), pf3              # ENABLE = 0 после soft_reset
    # после SOFT_RESET программа снова верна (без нового reset)
    await ax.write(shell.REG["CTRL"], 1)
    src2 = bytes(random.Random(11).randrange(256) for _ in range(256 * 4))
    mem.write_bytes(0x110000, src2)
    p = Program(); p.load(2, 100, 0x110000, len(src2)); p.wait(2, 1)
    p.store(3, 100, 0x210000, len(src2)); p.wait(3, 1); p.signal(7, 5)
    check_program(p, run_regions=None)
    await shell.submit(ax, p); await shell.wait_sem(ax, 7, 5)
    assert mem.read_bytes(0x210000, len(src2)) == src2


@cocotb.test(skip=TOP != "kshell_sink_top")
async def t2_stream_order(dut):
    ax, mem = await bringup(dut, lat=(5, 90), seed=2, interleave=True)
    stall = StallDriver(dut, 0.3, seed=2)             # сверх брифа: tready = 0 → skid
    src = bytes(random.Random(2).randrange(256) for _ in range(256 * 20))
    mem.write_bytes(0x300000, src)
    p = Program(); p.stream(0x300000, len(src)); p.run(0, 0, len(src) // 32, 0, 0)
    p.wait(0, 1); p.store(1, 0, 0x400000, len(src)); p.wait(1, 1); p.signal(7, 1)
    # sink_kernel пишет поток в spm[a0, a0 + a1) (a1 = число beat'ов)
    check_program(p, run_regions=lambda a: ([], [(a[0], a[0] + a[1])]))
    await shell.submit(ax, p); await shell.wait_sem(ax, 7, 1)
    assert mem.read_bytes(0x400000, len(src)) == src
    stall.stop()
    assert mem.stats["ooo"] > 0 and mem.stats["interleaved"] > 0, mem.stats
    assert stall.bp > 0
    st = await settle(dut, ax)
    assert not st["ERROR"] and not st["krn_busy"], st
    pf = await shell.perf(ax)
    dut._log.info("T2 perf %s (bp seen by tb %d)", pf, stall.bp)
    assert pf["STRM_BP"] == stall.bp, pf               # STRM_BP = tvalid && !tready
    assert pf["RD_BURSTS"] == 20 and pf["WR_BURSTS"] == 20, pf
    assert 160 <= pf["KRN_BUSY"] < pf["CYCLES"], pf    # ≥ 160 beat'ов приёма


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
        assert await ax.read(shell.REG["ERR_CMD"]) == 0          # первая (и единственная) команда
        await shell.soft_reset(ax); await ax.write(shell.REG["CTRL"], 1)
        st = await shell.status(ax)
        assert not st["ERROR"] and await ax.read(shell.REG["ERR_CODE"]) == 0, st
    # переполнение командной FIFO без ERROR
    await ax.write(shell.REG["CTRL"], 0)                                  # пауза: FIFO не убывает
    for _ in range(20): await ax.write(shell.REG["DOORBELL"], 0)
    st = await shell.status(ax)
    assert st["cq_ovf"] and not st["ERROR"]
    assert st["cq_free"] == 0, st
    await shell.soft_reset(ax)
    st = await shell.status(ax)
    assert not st["cq_ovf"] and st["cq_free"] == 16 and st["IDLE"], st


# --------------------------------------------------------------------------- сверх брифа

@cocotb.test(skip=TOP != "kshell_sink_top")
async def t_err_first_wins(dut):
    """Первая ошибка не перезаписывается: ALIGN (#2) фиксируется, а пришедший позже SLVERR
    уже выданного LOAD (#1) — нет. Следующие команды не исполняются, FIFO вычищена,
    семафор LOAD не растёт (abort), SIGNAL до ошибки учтён."""
    bad_burst = 0x1000 + 256 * 7
    ax, mem = await bringup(dut, lat=(150, 200), seed=21, slverr={bad_burst})
    rd_err = []

    async def mon():
        while True:
            await ReadOnly()
            if int(dut.u_shell.rd_err_v.value):
                rd_err.append(int(dut.u_shell.error_q.value))
            await RisingEdge(dut.clk)
    cocotb.start_soon(mon())
    p = Program()
    p.signal(2, 1)                          # #0
    p.load(0, 0, 0x1000, 256 * 8)           # #1, последний burst — SLVERR (придёт поздно)
    p.load(0, 0, 0x1001, 256)               # #2 ALIGN
    p.signal(7, 1)                          # #3 выброшена
    await shell.submit(ax, p)
    for _ in range(600): await RisingEdge(dut.clk)
    st = await shell.status(ax)
    assert st["ERROR"] and st["AXI_QUIET"] and st["cq_free"] == 16, st
    assert rd_err == [1], rd_err                            # RRESP пришёл, когда ERROR уже стоял
    assert await ax.read(shell.REG["ERR_CODE"]) == 3
    assert await ax.read(shell.REG["ERR_CMD"]) == 2
    assert await ax.read(shell.REG["CMD_COUNT"]) == 3       # #3 выброшена
    assert await sem(ax, 2) == 1 and await sem(ax, 0) == 0 and await sem(ax, 7) == 0
    await shell.soft_reset(ax)
    assert [await sem(ax, i) for i in range(8)] == [0] * 8


@cocotb.test(skip=TOP != "kshell_sink_top")
async def t_kernel_err(dut):
    """sink_kernel: tlast не на beat'е arg1−1 → KERNEL_ERR с номером RUN; ядро зависает в RUN,
    SOFT_RESET (k_rst_n) его сбрасывает; следующая программа верна."""
    ax, mem = await bringup(dut, lat=(5, 40), seed=22)
    src = bytes(random.Random(22).randrange(256) for _ in range(256 * 2))
    mem.write_bytes(0x300000, src)
    p = Program(); p.signal(1, 1); p.stream(0x300000, len(src)); p.run(0, 0, 8, 0, 0)  # 16 ≠ 8
    p.wait(0, 1); p.signal(7, 1)
    await shell.submit(ax, p)
    for _ in range(600): await RisingEdge(dut.clk)
    st = await shell.status(ax)
    assert st["ERROR"] and st["AXI_QUIET"] and not st["IDLE"], st
    assert await ax.read(shell.REG["ERR_CODE"]) == 6
    assert await ax.read(shell.REG["ERR_CMD"]) == 2
    assert await sem(ax, 0) == 0 and await sem(ax, 7) == 0
    await shell.soft_reset(ax)
    st = await shell.status(ax)
    assert st["IDLE"] and not st["krn_busy"] and not st["ERROR"], st
    await ax.write(shell.REG["CTRL"], 1)
    p = Program(); p.stream(0x300000, len(src)); p.run(0, 0, 16, 0, 0); p.wait(0, 1)
    p.store(1, 0, 0x400000, len(src)); p.wait(1, 1); p.signal(7, 1)
    check_program(p, run_regions=lambda a: ([], [(a[0], a[0] + a[1])]))
    await shell.submit(ax, p); await shell.wait_sem(ax, 7, 1)
    assert mem.read_bytes(0x400000, len(src)) == src


@cocotb.test(skip=TOP != "kshell_sink_top")
async def t_slverr_sink(dut):
    """SLVERR как первая ошибка на sink-топе: RRESP на LOAD (#0) и BRESP на STORE (#2).
    Код и номер команды, семафор сбойной команды не растёт, выданное доводится до AXI_QUIET,
    SOFT_RESET, затем та же программа без SLVERR проходит (замыкает бины err.rresp/bresp)."""
    src = bytes(random.Random(24).randrange(256) for _ in range(256 * 4))
    cases = (("load", 0x500000 + 256 * 2, 1, 0, (0, 0)),
             ("store", 0x600000 + 256 * 1, 2, 2, (1, 0)))
    for target, bad, code, cmd, exp_sem in cases:
        ax, mem = await bringup(dut, lat=(10, 80), seed=24, interleave=True, slverr={bad})
        mem.write_bytes(0x500000, src)
        p = Program(); p.load(0, 0, 0x500000, len(src)); p.wait(0, 1)
        p.store(1, 0, 0x600000, len(src)); p.wait(1, 1); p.signal(7, 1)
        check_program(p, run_regions=None)
        await shell.submit(ax, p)
        for _ in range(5000):
            await RisingEdge(dut.clk)
            if (st := await shell.status(ax))["ERROR"]: break
        assert st["ERROR"], f"{target}: нет ERROR"
        assert await ax.read(shell.REG["ERR_CODE"]) == code, target
        assert await ax.read(shell.REG["ERR_CMD"]) == cmd, target
        for _ in range(2000 // 16):
            if (st := await shell.status(ax))["AXI_QUIET"]: break
            for _ in range(16): await RisingEdge(dut.clk)
        assert st["AXI_QUIET"] and st["cq_free"] == 16, (target, st)
        assert (await sem(ax, 0), await sem(ax, 1), await sem(ax, 7)) == exp_sem + (0,), target
        await shell.soft_reset(ax)
        st = await shell.status(ax)
        assert st["IDLE"] and not st["ERROR"] and await ax.read(shell.REG["ERR_CODE"]) == 0, st
        mem.slverr.clear(); await ax.write(shell.REG["CTRL"], 1)
        await shell.submit(ax, p); await shell.wait_sem(ax, 7, 1)
        assert mem.read_bytes(0x600000, len(src)) == src, target
        mem.stop(); await shell.reset(dut)


@cocotb.test(skip=TOP != "kshell_sink_top")
async def t_watchdog(dut):
    """WDOG_CYCLES = 2000: долгий WAIT — не неисправность (и не IDLE); замёрзшая память на
    чтении и на записи → TIMEOUT, ERR_CMD = последняя взятая команда; поля inflight STATUS;
    SOFT_RESET при незатихшем AXI игнорируется; после разморозки — восстановление
    (полная проверка — T8 на kshell_top)."""
    ax, mem = await bringup(dut, lat=(10, 40), seed=23)
    p = Program(); p.wait(3, 1); await shell.submit(ax, p)
    for _ in range(3000): await RisingEdge(dut.clk)
    st = await shell.status(ax)
    assert not st["ERROR"] and not st["IDLE"] and st["AXI_QUIET"], st
    await ax.write(shell.REG["SEM"] + 4 * 3, 1)
    # чтение: память замерзает с AR в полёте
    mem.lat = (100, 200)
    p = Program(); p.load(0, 0, 0x2000000, 256 * 16); await shell.submit(ax, p)
    for _ in range(30): await RisingEdge(dut.clk)
    mem.freeze()
    st = await shell.status(ax)
    assert st["rd_inflight"] > 0 and st["wr_inflight"] == 0 and not st["AXI_QUIET"], st
    for _ in range(2500): await RisingEdge(dut.clk)
    st = await shell.status(ax)
    assert st["ERROR"] and not st["AXI_QUIET"], st
    assert await ax.read(shell.REG["ERR_CODE"]) == 7
    assert await ax.read(shell.REG["ERR_CMD"]) == 1
    await ax.write(shell.REG["CTRL"], shell.CTRL_SOFT_RESET)            # не тихо → игнор
    st = await shell.status(ax)
    assert st["ERROR"] and await ax.read(shell.REG["ERR_CODE"]) == 7, st
    assert await sem(ax, 3) == 1                                        # семафоры не сброшены
    mem.frozen = False                                                  # ответы доходят
    await shell.soft_reset(ax)
    st = await shell.status(ax)
    assert st["IDLE"] and not st["ERROR"], st
    # запись: память замерзает с AW в полёте (ждём B)
    await ax.write(shell.REG["CTRL"], 1)
    p = Program(); p.store(0, 0, 0x3000000, 256 * 16); await shell.submit(ax, p)
    for _ in range(40): await RisingEdge(dut.clk)
    mem.freeze()
    st = await shell.status(ax)
    assert st["wr_inflight"] > 0 and st["rd_inflight"] == 0 and not st["AXI_QUIET"], st
    for _ in range(2500): await RisingEdge(dut.clk)
    assert (await shell.status(ax))["ERROR"]
    assert await ax.read(shell.REG["ERR_CODE"]) == 7
    assert await ax.read(shell.REG["ERR_CMD"]) == 0


@cocotb.test(skip=TOP != "kshell_sink_top")
async def t_axil_regs(dut):
    """AXI-Lite: W раньше AW и AW раньше W, wstrb, незанятые адреса читаются 0, SEM RW, CTRL."""
    await shell.reset(dut)
    ax = AxilMaster(dut)
    s = lambda n: getattr(dut, f"s_axil_{n}")

    async def write_split(addr, data, w_first, gap=3, strb=0xF):
        await RisingEdge(dut.clk)
        first, second = (("w", "aw") if w_first else ("aw", "w"))
        for ch in (first, second):
            if ch == "aw":
                s("awaddr").value = addr; s("awvalid").value = 1
            else:
                s("wdata").value = data; s("wstrb").value = strb; s("wvalid").value = 1
            while True:
                await ReadOnly()
                hs = int(s(f"{ch}ready").value)
                await RisingEdge(dut.clk)
                if hs:
                    s(f"{ch}valid").value = 0
                    break
            for _ in range(gap): await RisingEdge(dut.clk)
        s("bready").value = 1
        while True:
            await ReadOnly()
            hs = int(s("bvalid").value)
            resp = int(s("bresp").value)
            await RisingEdge(dut.clk)
            if hs:
                break
        s("bready").value = 0
        assert resp == 0

    sem_a = lambda i: shell.REG["SEM"] + 4 * i
    await write_split(sem_a(3), 0xDEADBEEF, w_first=True)
    assert await ax.read(sem_a(3)) == 0xDEADBEEF
    await write_split(sem_a(4), 0x12345678, w_first=False)
    assert await ax.read(sem_a(4)) == 0x12345678
    await write_split(sem_a(4), 0x0000AA00, w_first=True, strb=0b0010)  # только байт 1
    assert await ax.read(sem_a(4)) == 0x1234AA78
    await ax.write(sem_a(7), 0xFFFFFFFF)
    assert await ax.read(sem_a(7)) == 0xFFFFFFFF
    for a in (0x010, 0x014, 0x020, 0x028, 0x03C, 0x060, 0x0A0, 0x100, 0xFFC):
        assert await ax.read(a) == 0, f"{a:#x}"                 # WO и незанятые → 0
    await ax.write(shell.REG["CTRL"], 0x7)                      # SOFT_RESET и PERF_SNAP самосброс
    assert await ax.read(shell.REG["CTRL"]) == 1
    assert await ax.read(sem_a(3)) == 0                         # SOFT_RESET (AXI тихо) → sem = 0
    st = await shell.status(ax)
    assert st["IDLE"] and st["AXI_QUIET"] and st["cq_free"] == 16, st


# --------------------------------------------------------------------------- T3, T4 (бриф, часть 2)

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
        for a in (0x3000000, 0x3100000):                    # R15b: старый y не «пройдёт» проверку
            mem.write_bytes(a, b"\xAA" * (4 * M))
        p = fn(M, K); check_program(p)
        await shell.submit(ax, p); await shell.wait_sem(ax, 7, 1)
        res[name] = await shell.perf(ax)
        assert (np.frombuffer(mem.read_bytes(0x3000000, 4 * M), np.int32) == y0).all()
        assert (np.frombuffer(mem.read_bytes(0x3100000, 4 * M), np.int32) == y1).all()
        await shell.soft_reset(ax); await ax.write(shell.REG["CTRL"], 1)
    assert res["ovl"]["CYCLES"] < res["seq"]["CYCLES"]
    assert res["ovl"]["KRN_BUSY"] / res["ovl"]["CYCLES"] > res["seq"]["KRN_BUSY"] / res["seq"]["CYCLES"]
    dut._log.info("seq=%s ovl=%s", res["seq"], res["ovl"])


# --------------------------------------------------------------------------- T5, T6, T8 (бриф, часть 3)

def gemv_prog(M, K):
    """#0 LOAD x, #1 STREAM W, #2 WAIT, #3 RUN, #4 WAIT, #5 STORE y, #6 WAIT, #7 SIGNAL."""
    p = Program(); p.load(0, 0, 0x2000000, K); p.stream(0x1000000, M * K); p.wait(0, 1)
    p.run(1, 0, 64, M, K); p.wait(1, 1); p.store(2, 64, 0x3000000, 4 * M); p.wait(2, 1); p.signal(7, 1)
    return p

@cocotb.test(skip=TOP != "kshell_top")
async def t5_slverr_in_flight(dut):
    """SLVERR на LOAD x (#0), на burst'е 20 STREAM W (#1, ядро в середине RUN) и на STORE y (#5):
    код и номер команды (§12.2), семафоры не растут, AXI_QUIET без SOFT_RESET (ничего не
    бросается), SOFT_RESET сбрасывает и зависшее ядро (k_rst_n); затем та же программа верна."""
    exp_cmd = dict(load=0, stream=1, store=5)
    exp_sem = dict(load=(0, 0), stream=(1, 0), store=(1, 1))          # (sem0, sem1)
    for target in ("load", "stream", "store"):
        ax, mem = await bringup(dut, lat=(10, 80), seed=5,
                                slverr={dict(load=0x2000000, stream=0x1000000 + 256 * 20,
                                             store=0x3000000)[target]})
        rng = np.random.default_rng(5); M, K = 64, 512
        W, x, y = gemv_case(rng, M, K)
        mem.write_bytes(0x1000000, W.tobytes()); mem.write_bytes(0x2000000, x.tobytes())
        p = gemv_prog(M, K); check_program(p)
        await shell.submit(ax, p)
        for _ in range(20000):
            await RisingEdge(dut.clk)
            if (st := await shell.status(ax))["ERROR"]: break
        assert st["ERROR"], f"{target}: нет ERROR"
        assert await ax.read(shell.REG["ERR_CODE"]) == (2 if target == "store" else 1)
        assert await ax.read(shell.REG["ERR_CMD"]) == exp_cmd[target], target      # R15a
        assert await ax.read(shell.REG["SEM"] + 4 * 7) == 0           # программа не «прошла»
        # сверх брифа: выданное доводится до конца само (ENABLE = 1, без SOFT_RESET)
        for _ in range(2000 // 16):
            if (st := await shell.status(ax))["AXI_QUIET"]: break
            for _ in range(16): await RisingEdge(dut.clk)
        assert st["AXI_QUIET"] and st["cq_free"] == 16, (target, st)
        # ядро: на STREAM застряло в середине RUN (данные ROB отброшены, §12.2.4) — не IDLE
        stuck = target == "stream"
        assert bool(st["krn_busy"]) == stuck and bool(st["IDLE"]) == (not stuck), (target, st)
        assert (await sem(ax, 0), await sem(ax, 1), await sem(ax, 2)) == exp_sem[target] + (0,), target
        await shell.soft_reset(ax)                                     # ждёт AXI_QUIET
        st = await shell.status(ax); assert st["IDLE"] and not st["ERROR"]
        assert not st["krn_busy"] and await ax.read(shell.REG["ERR_CODE"]) == 0, (target, st)
        # восстановление: та же программа без SLVERR проходит (ядро сброшено k_rst_n)
        mem.slverr.clear(); mem.write_bytes(0x3000000, b"\xAA" * (4 * M))
        await ax.write(shell.REG["CTRL"], 1)
        await shell.submit(ax, p); await shell.wait_sem(ax, 7, 1)
        assert (np.frombuffer(mem.read_bytes(0x3000000, 4 * M), np.int32) == y).all(), target
        mem.stop(); await shell.reset(dut)                             # следующий цикл с новой моделью

@cocotb.test(skip=TOP != "kshell_top")
async def t6_pause_resume(dut):
    ax, mem = await bringup(dut, lat=(10, 80), seed=6)
    rng = np.random.default_rng(6); M, K = 64, 512
    W, x0, y0 = gemv_case(rng, M, K); _, x1, _ = gemv_case(rng, M, K)
    y1 = (W.astype(np.int64) @ x1.astype(np.int64)).astype(np.int32)
    for a, d in ((0x1000000, W), (0x2000000, x0), (0x2100000, x1)): mem.write_bytes(a, d.tobytes())
    p = overlapped(M, K); check_program(p)
    await shell.submit(ax, p)
    for _ in range(300): await RisingEdge(dut.clk)
    st0 = await shell.status(ax)
    assert not st0["IDLE"] and await sem(ax, 7) == 0, st0              # пауза посреди работы
    await ax.write(shell.REG["CTRL"], 0)                               # пауза
    for _ in range(5000):
        await RisingEdge(dut.clk)
        if (await shell.status(ax))["AXI_QUIET"]: break
    assert (await shell.status(ax))["AXI_QUIET"]
    # сверх брифа (§12.3, §12.5, §14): тишина держится, новых AR/AW нет, watchdog на паузе
    # молчит (пауза > WDOG_CYCLES = 2000), perf не считает
    ar, aw = mem.stats["ar"], mem.stats["aw"]
    assert ar < 2 * (K // 256) + 2 * (M * K // 256), mem.stats       # работа реально не вся
    pf0 = await shell.perf(ax)
    for _ in range(3000): await RisingEdge(dut.clk)
    st = await shell.status(ax)
    assert st["AXI_QUIET"] and not st["ERROR"] and not st["IDLE"], st
    assert (mem.stats["ar"], mem.stats["aw"]) == (ar, aw), mem.stats
    pf1 = await shell.perf(ax)
    assert pf1["CYCLES"] == pf0["CYCLES"] and pf1["RD_BURSTS"] == pf0["RD_BURSTS"], (pf0, pf1)
    assert await sem(ax, 7) == 0
    await ax.write(shell.REG["CTRL"], 1)                               # продолжение
    await shell.wait_sem(ax, 7, 1)
    assert (np.frombuffer(mem.read_bytes(0x3000000, 4 * M), np.int32) == y0).all()
    assert (np.frombuffer(mem.read_bytes(0x3100000, 4 * M), np.int32) == y1).all()
    st = await settle(dut, ax)
    assert not st["ERROR"], st
    assert mem.stats["ar"] == 2 * (K // 256) + 2 * (M * K // 256), mem.stats   # ни один burst не повторён

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
    st = await shell.status(ax)
    assert st["rd_inflight"] > 0 and not st["AXI_QUIET"], st          # замёрзло с AR в полёте
    for _ in range(5000): await RisingEdge(dut.clk)
    assert await ax.read(shell.REG["ERR_CODE"]) == 7
    assert await ax.read(shell.REG["ERR_CMD"]) == 1                   # последняя взятая (LOAD, #1)
    assert await sem(ax, 0) == 0
    # сверх брифа: после разморозки — тишина, SOFT_RESET, работа снова идёт
    mem.frozen = False
    await shell.soft_reset(ax)
    st = await shell.status(ax)
    assert st["IDLE"] and not st["ERROR"], st
    # медленное ядро — не неисправность (§12.5): RUN без своей STREAM ждёт поток > WDOG_CYCLES
    # при AXI_QUIET; STREAM подаётся позже (сознательно нарушает правило 6.1.1 — без check_program)
    await ax.write(shell.REG["CTRL"], 1)
    rng = np.random.default_rng(8); M, K = 64, 256
    W, x, y = gemv_case(rng, M, K)
    mem.write_bytes(0x1000000, W.tobytes()); mem.write_bytes(0x2000000, x.tobytes())
    p = Program(); p.load(0, 0, 0x2000000, K); p.wait(0, 1); p.run(1, 0, 64, M, K)
    await shell.submit(ax, p)
    for _ in range(5000): await RisingEdge(dut.clk)
    st = await shell.status(ax)
    assert st["krn_busy"] and st["AXI_QUIET"] and not st["ERROR"] and not st["IDLE"], st
    p = Program(); p.stream(0x1000000, M * K); p.wait(1, 1); p.store(2, 64, 0x3000000, 4 * M)
    p.wait(2, 1); p.signal(7, 1)
    await shell.submit(ax, p); await shell.wait_sem(ax, 7, 1)
    assert (np.frombuffer(mem.read_bytes(0x3000000, 4 * M), np.int32) == y).all()
    assert await ax.read(shell.REG["ERR_CODE"]) == 0


@cocotb.test(skip=TOP != "kshell_top")
async def t_kdone_after_error(dut):
    """R10/R15c: k_done, пришедший ПОСЛЕ ошибки, не увеличивает семафор RUN (§7, §11, §12.2.3).
    Ядро уже приняло весь свой поток (tlast), до k_done gemv ещё ~6 тактов; в этот промежуток
    подаётся SLVERR несвязанного LOAD (#4): модель держит его ответ (hold) и отпускает в такте
    handshake'а tlast. Тест проверяет своё предусловие: ошибка — строго после tlast и не позже k_done."""
    poison = 0x2200000
    ax, mem = await bringup(dut, lat=(10, 80), seed=9, slverr={poison}, hold={poison})
    rng = np.random.default_rng(9); M, K = 64, 256
    W, x, _ = gemv_case(rng, M, K)
    mem.write_bytes(0x1000000, W.tobytes()); mem.write_bytes(0x2000000, x.tobytes())
    ev = dict(last=[], err=[], done=[])
    u = dut.u_shell

    async def mon():
        cyc = 0
        while True:
            await ReadOnly()
            if int(dut.k_s_tvalid.value) and int(dut.k_s_tready.value) and int(dut.k_s_tlast.value):
                ev["last"].append(cyc); mem.release(poison)
            if int(u.rd_err_v.value) or int(u.wr_err_v.value) or int(u.dec_err_v.value) \
                    or int(u.k_err_q.value) or int(u.wdog_hit_q.value):
                ev["err"].append(cyc)
            if int(dut.k_done.value):
                ev["done"].append((cyc, int(u.error_q.value), int(u.krn_busy_q.value)))
            await RisingEdge(dut.clk); cyc += 1
    cocotb.start_soon(mon())
    p = Program()
    p.load(0, 0, 0x2000000, K)              # #0
    p.stream(0x1000000, M * K)              # #1
    p.wait(0, 1)                            # #2
    p.run(1, 0, 64, M, K)                   # #3
    p.load(4, 200, poison, 256)             # #4 SLVERR, ответ задержан до tlast
    p.wait(1, 1)                            # #5
    p.signal(7, 1)                          # #6
    check_program(p)
    await shell.submit(ax, p)
    for _ in range(20000 // 16):
        if ev["done"]: break
        for _ in range(16): await RisingEdge(dut.clk)
    for _ in range(20): await RisingEdge(dut.clk)
    dut._log.info("tlast/err/k_done: %s", ev)
    assert len(ev["last"]) == 1 and len(ev["done"]) == 1, ev
    t_last, (t_done, _, busy) = ev["last"][0], ev["done"][0]
    assert ev["err"] and t_last < ev["err"][0] <= t_done and busy, ev   # предусловие теста
    assert await ax.read(shell.REG["ERR_CODE"]) == 1
    assert await ax.read(shell.REG["ERR_CMD"]) == 4
    assert await sem(ax, 1) == 0, "k_done после ошибки увеличил семафор RUN"
    assert await sem(ax, 0) == 1 and await sem(ax, 4) == 0 and await sem(ax, 7) == 0
    st = await shell.status(ax)
    assert st["ERROR"] and not st["krn_busy"], st                     # k_done снял krn_busy
    await shell.soft_reset(ax)
    st = await shell.status(ax)
    assert st["IDLE"] and not st["ERROR"], st


# --------------------------------------------------------------------------- T9 (бриф, часть 4)

def spm_words(dut, words):
    """Слова spm (bytes, little-endian по байтам beat'а) прямым чтением массива памяти."""
    arr = dut.u_shell.u_spm.mem_q
    return {w: int(arr[w].value).to_bytes(32, "little") for w in words}


@cocotb.test(skip=TOP != "kshell_sink_top")
async def t9_random_programs(dut):
    """§16: случайные программы против последовательного эталона. Сверх брифа: каждая
    программа проверяется до конца (IDLE, нет ERROR), сравнивается и spm (слова, записанные
    программой), sem7; OoO/чередование модели памяти реально были."""
    seeds = int(os.environ.get("T9_SEEDS", 20))
    ooo = inter = 0
    for seed in range(seeds):
        ax, mem = await bringup(dut, lat=(5, 120), seed=seed, interleave=bool(seed % 2))
        rng = random.Random(seed)
        init = bytes(rng.randrange(256) for _ in range(1 << 16))
        mem.write_bytes(0x500000, init)
        p = random_program(rng, n_cmds=40, spm_words=1024, hbm_base=0x500000, hbm_size=1 << 16)
        check_program(p, run_regions=None)
        ref_mem, ref_sem, ref_spm = execute_reference(p, init, base=0x500000, with_spm=True)
        await shell.submit(ax, p); await shell.wait_sem(ax, 7, 1)
        got_mem = mem.read_bytes(0x500000, 1 << 16)
        if got_mem != ref_mem:
            off = next(i for i in range(len(ref_mem)) if got_mem[i] != ref_mem[i])
            raise AssertionError(f"seed {seed}: память расходится с эталоном с 0x{0x500000 + off:x} "
                                 f"(burst {off // 256}, программа {len(p)} команд)")
        for i in range(7): assert await ax.read(shell.REG["SEM"] + 4 * i) == ref_sem[i], (seed, i)
        st = await settle(dut, ax)
        assert not st["ERROR"] and st["cq_free"] == 16, (seed, st)
        assert await ax.read(shell.REG["SEM"] + 4 * 7) == 1, seed
        want = {w: v for w, v in enumerate(ref_spm) if v is not None}
        got = spm_words(dut, want)
        bad = [w for w in want if got[w] != want[w]]
        assert not bad, f"seed {seed}: spm слова {bad[:8]}"
        assert await ax.read(shell.REG["CMD_COUNT"]) == len(p), seed
        ooo += mem.stats["ooo"]; inter += mem.stats["interleaved"]
        dut._log.info("T9 seed %d: %d команд, %s", seed, len(p), mem.stats)
        mem.stop(); await shell.reset(dut)
    assert ooo > 0 and (seeds < 2 or inter > 0), (ooo, inter)


# --------------------------------------------------------------------------- closure покрытия (§16)

@cocotb.test(skip=TOP != "kshell_sink_top")
async def t_hol_blocking(dut):
    """§8.5 head-of-line (сознательно): голова STREAM стоит, пока ядро не берёт поток, и
    готовый LOAD за ней не коммитится — его данные уже в spm, слот done, но sem не растёт.
    После снятия backpressure поток и LOAD завершаются по порядку, данные верны.
    Closure бина rd_dma_cov.cp_hol."""
    ax, mem = await bringup(dut, lat=(5, 20), seed=31)
    rng = random.Random(31)
    src_s = bytes(rng.randrange(256) for _ in range(256))
    src_l = bytes(rng.randrange(256) for _ in range(256))
    mem.write_bytes(0x600000, src_s); mem.write_bytes(0x610000, src_l)
    dut.k_stall.value = 1                                   # ядро не берёт поток
    p = Program()
    p.stream(0x600000, 256)                                 # #0 1 burst: голова окна
    p.load(0, 100, 0x610000, 256)                           # #1 следующий слот
    p.run(1, 0, 8, 0, 0)                                    # #2 sink: spm[0..8) ← поток
    p.wait(0, 1); p.wait(1, 1)                              # правило 2: RUN уже выдан
    p.store(2, 0, 0x620000, 256); p.store(2, 100, 0x630000, 256); p.wait(2, 2); p.signal(7, 1)
    check_program(p, run_regions=lambda a: ([], [(a[0], a[0] + a[1])]))
    await shell.submit(ax, p)
    for _ in range(300): await RisingEdge(dut.clk)
    u = dut.u_shell.u_rd
    st = await shell.status(ax)
    assert st["rd_inflight"] == 2 and st["krn_busy"], st              # STREAM + LOAD в окне
    assert int(dut.k_s_tvalid.value) and not int(dut.k_s_tready.value)
    hs, done = int(u.head_q.value) % 16, int(u.w_done_q.value)
    assert all(done >> ((hs + k) % 16) & 1 for k in range(2)), f"{done:#x}"   # оба слота done
    assert int(u.w_stream_q[hs].value) == 1 and int(u.w_stream_q[(hs + 1) % 16].value) == 0
    assert await sem(ax, 0) == 0                                      # LOAD готов, но не снят
    got = spm_words(dut, range(100, 108))
    assert b"".join(got[w] for w in range(100, 108)) == src_l         # данные LOAD уже в spm
    dut.k_stall.value = 0
    await shell.wait_sem(ax, 7, 1)
    assert mem.read_bytes(0x620000, 256) == src_s and mem.read_bytes(0x630000, 256) == src_l
    pf = await shell.perf(ax)
    assert pf["STRM_BP"] > 250, pf


@cocotb.test(skip=TOP != "kshell_sink_top")
async def t_sem_collide(dut):
    """§11: приращения одного семафора в одном такте суммируются. Командная FIFO
    заполняется на паузе (ENABLE = 0): STORE s, LOAD v, LOAD s (nb burst'ов), WAIT v ≥ 1,
    12 × SIGNAL s. После ENABLE = 1 WAIT пропускает поезд SIGNAL'ов (по одному за такт)
    примерно тогда, когда коммитится LOAD s и приходит B STORE s. Сиды перебираются, пока
    монитор не увидит пары rd+SIGNAL, wr+SIGNAL, rd+wr и тройку rd+wr+SIGNAL в одном такте на
    один семафор (сиды фиксированы → детерминировано); каждый раз SEM[s] сверяется с суммой. Closure бинов kshell_cov.cp_same_inc / cp_pair_*."""
    ax, mem = await bringup(dut, lat=(2, 10), seed=41)
    u = dut.u_shell
    seen = dict(rd_sig=0, wr_sig=0, rd_wr=0, three=0)

    async def mon():
        while True:
            await ReadOnly()
            src = []
            if int(u.inc_rd_v.value): src.append(("rd", int(u.rd_sem_inc_idx.value)))
            if int(u.inc_wr_v.value): src.append(("wr", int(u.wr_sem_inc_idx.value)))
            if int(u.sig_v.value): src.append(("sig", int(u.sig_idx.value)))
            for a, b in (("rd", "sig"), ("wr", "sig"), ("rd", "wr")):
                if {i for n, i in src if n == a} & {i for n, i in src if n == b}:
                    seen[f"{a}_{b}"] += 1
            if len(src) == 3 and len({s for _, s in src}) == 1:
                seen["three"] += 1
            await RisingEdge(dut.clk)
    task = cocotb.start_soon(mon())
    it = 0
    for it in range(400):
        rng = random.Random(1000 + it)
        s, v = rng.sample(range(7), 2)
        p = Program()
        p.store(s, 0, 0x700000, 256)
        p.load(v, 100, 0x710000, 256)
        p.load(s, 200, 0x720000, 256 * rng.randint(1, 3))
        p.wait(v, 1)
        for _ in range(12): p.signal(s, 1)
        check_program(p, run_regions=None)
        await ax.write(shell.REG["CTRL"], 0)                # пауза: FIFO копит программу
        await shell.submit(ax, p)
        await ax.write(shell.REG["CTRL"], 1)
        await shell.wait_sem(ax, s, 14, timeout=5000)
        await settle(dut, ax)
        assert await sem(ax, s) == 14 and await sem(ax, v) == 1, (it, s, v)
        await shell.soft_reset(ax)
        if all(seen.values()):
            break
    task.cancel()
    dut._log.info("t_sem_collide: %d итераций, совпадения %s", it + 1, seen)
    assert all(seen.values()), seen
