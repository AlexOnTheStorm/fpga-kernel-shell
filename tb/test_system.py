# Системные тесты kshell (docs/design.md §16): хост — AxilMaster + shell.py, память — HbmModel
# (OoO по ID). Тест выбирается по имени верхнего уровня (kshell_sink_top / kshell_top).
#   T1 memcpy LOAD → WAIT → STORE; T2 STREAM в sink_kernel (порядок при OoO, tlast, skid);
#   T7 ошибки декодирования + переполнение командной FIFO.
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
from isa import Program, check_program
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
    check_program(p, run_regions=None)
    await shell.submit(ax, p); await shell.wait_sem(ax, 7, 1)
    assert mem.read_bytes(0x400000, len(src)) == src


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
        p = fn(M, K); check_program(p)
        await shell.submit(ax, p); await shell.wait_sem(ax, 7, 1)
        res[name] = await shell.perf(ax)
        assert (np.frombuffer(mem.read_bytes(0x3000000, 4 * M), np.int32) == y0).all()
        assert (np.frombuffer(mem.read_bytes(0x3100000, 4 * M), np.int32) == y1).all()
        await shell.soft_reset(ax); await ax.write(shell.REG["CTRL"], 1)
    assert res["ovl"]["CYCLES"] < res["seq"]["CYCLES"]
    assert res["ovl"]["KRN_BUSY"] / res["ovl"]["CYCLES"] > res["seq"]["KRN_BUSY"] / res["seq"]["CYCLES"]
    dut._log.info("seq=%s ovl=%s", res["seq"], res["ovl"])
