# Модульные тесты wr_dma (docs/design.md §9): STORE spm → AXI4 write поверх HbmModel
# (только каналы записи). Первый тест — из брифа Task 4; остальные закрывают требования,
# которые бриф не проверяет: несколько команд подряд, лимит WR_OUT, BRESP ≠ OKAY (нет sem,
# err_cmd, накопитель ошибки команды), пауза go=0, abort с W в полёте + soft_rst.
import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ReadOnly, RisingEdge

from hbm_model import HbmModel

BURST = 8
BB = 32
WR_OUT = 8
BURST_BYTES = BURST * BB


def words(data):
    return [int.from_bytes(data[i * BB:(i + 1) * BB], "little") for i in range(len(data) // BB)]


async def setup(dut, **mk):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    for s in ("soft_rst", "abort", "cmd_v"):
        getattr(dut, s).value = 0
    dut.go.value = 1
    dut.rst_n.value = 0
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    mk.setdefault("lat", (5, 50))
    mem = HbmModel(dut, has_read=False, **mk)
    mem.start()
    return mem


def start_spm(dut, spm, rng, p_busy=0.5):
    """Порт A spm: gnt в доле тактов (остальные — «приоритетные R-записи»), данные через такт."""
    async def run():
        pending = None
        while True:
            busy = rng.random() < p_busy
            dut.spm_rd_gnt.value = 0 if busy else int(dut.spm_rd_req.value)
            dut.spm_rd_data.value = spm[pending] if pending is not None else 0
            await ReadOnly()
            nxt = int(dut.spm_rd_addr.value) if int(dut.spm_rd_gnt.value) else None
            await RisingEdge(dut.clk)
            pending = nxt
    cocotb.start_soon(run())


async def push(dut, sem, spm, hbm, nb, no):
    dut.cmd_v.value = 1
    dut.cmd_sem.value = sem
    dut.cmd_spm.value = spm
    dut.cmd_hbm.value = hbm
    dut.cmd_nb.value = nb
    dut.cmd_no.value = no
    while True:
        await ReadOnly()
        ok = int(dut.cmd_rdy.value)
        await RisingEdge(dut.clk)
        if ok:
            break
    dut.cmd_v.value = 0


class Mon:
    """Пассивный монитор: sem, ошибки, handshake'и AW, окно в полёте, спец-условия."""

    def __init__(self, dut):
        self.dut = dut
        self.sems, self.errs = [], []
        self.aw_hs = self.pf_aw = self.w_hs = 0
        self.max_inflight = 0
        self.cycles = 0
        cocotb.start_soon(self._run())

    async def _run(self):
        d = self.dut
        while True:
            await ReadOnly()
            self.cycles += 1
            if int(d.sem_inc_v.value):
                self.sems.append(int(d.sem_inc_idx.value))
            if int(d.err_v.value):
                self.errs.append(int(d.err_cmd.value))
            if int(d.m_axi_awvalid.value) and int(d.m_axi_awready.value):
                self.aw_hs += 1
            if int(d.m_axi_wvalid.value) and int(d.m_axi_wready.value):
                self.w_hs += 1
            self.pf_aw += int(d.pf_aw_hs.value)
            self.max_inflight = max(self.max_inflight, int(d.inflight.value))
            await RisingEdge(d.clk)


async def wait_for(cond, dut, limit, what):
    for _ in range(limit):
        await RisingEdge(dut.clk)
        if cond():
            return
    raise AssertionError(f"таймаут ({limit} тактов): {what}")


async def wait_quiet(dut, limit=3000):
    async def poll():
        await ReadOnly()
        return int(dut.axi_quiet.value)
    for _ in range(limit):
        await RisingEdge(dut.clk)
        if await poll():
            return
    raise AssertionError("axi_quiet не наступил")


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


@cocotb.test()
async def test_multi_command_chain(dut):
    """Три команды подряд (1, 3, 5 burst'ов): данные, порядок sem, perf, idle."""
    mem = await setup(dut, seed=11)
    rng = random.Random(21)
    spm = [rng.randrange(1 << 256) for _ in range(1024)]
    start_spm(dut, spm, rng, 0.3)
    mon = Mon(dut)
    await RisingEdge(dut.clk)
    await ReadOnly()
    assert int(dut.idle.value) == 1 and int(dut.axi_quiet.value) == 1
    assert int(dut.inflight.value) == 0
    await RisingEdge(dut.clk)
    cmds = [(1, 0, 0x10000, 3, 11), (4, 100, 0x20000, 1, 12), (7, 200, 0x30000, 5, 13)]
    for c in cmds:
        await push(dut, *c)
    await ReadOnly()
    assert int(dut.idle.value) == 0
    await wait_for(lambda: len(mon.sems) == 3, dut, 6000, "три sem")
    assert mon.sems == [1, 4, 7]
    for sem, sp, hbm, nb, no in cmds:
        got = words(mem.read_bytes(hbm, nb * BURST_BYTES))
        assert got == spm[sp:sp + nb * BURST], f"данные команды {no}"
    assert mon.aw_hs == mon.pf_aw == 9 and mem.stats["aw"] == 9
    assert mem.stats["w_beats"] == 9 * BURST
    await wait_for(lambda: int(dut.idle.value) == 1, dut, 200, "idle")
    assert int(dut.axi_quiet.value) == 1 and int(dut.inflight.value) == 0
    assert not mon.errs


@cocotb.test()
async def test_wr_out_limit(dut):
    """Длинная B-латентность: в полёте ровно WR_OUT burst'ов, не больше; всё доходит."""
    mem = await setup(dut, lat=(150, 150), seed=3, p_stall=0.0)
    rng = random.Random(8)
    spm = [rng.randrange(1 << 256) for _ in range(1024)]
    start_spm(dut, spm, rng, 0.0)
    mon = Mon(dut)
    nb = 24
    await RisingEdge(dut.clk)
    await push(dut, 6, 0, 0x50000, nb, 31)
    await wait_for(lambda: mon.sems, dut, 8000, "sem")
    assert mon.sems == [6]
    assert mon.max_inflight == WR_OUT, mon.max_inflight
    assert words(mem.read_bytes(0x50000, nb * BURST_BYTES)) == spm[:nb * BURST]
    assert mon.aw_hs == nb


@cocotb.test()
async def test_bresp_error(dut):
    """SLVERR на одном burst'е: нет sem этой команды, err_cmd; следующая команда чиста.

    cmd 21: ошибка в середине (накопитель держится до последнего B); cmd 22 — чистая;
    cmd 23: ошибка только на последнем burst'е."""
    base1, base2, base3 = 0x10000, 0x20000, 0x30000
    mem = await setup(dut, seed=7, slverr=[base1 + 2 * BURST_BYTES, base3 + 2 * BURST_BYTES])
    rng = random.Random(5)
    spm = [rng.randrange(1 << 256) for _ in range(1024)]
    start_spm(dut, spm, rng, 0.2)
    mon = Mon(dut)
    await RisingEdge(dut.clk)
    await push(dut, 3, 0, base1, 4, 21)
    await push(dut, 5, 40, base2, 2, 22)
    await push(dut, 6, 80, base3, 3, 23)
    await wait_for(lambda: len(mon.errs) == 2 and int(dut.idle.value), dut, 6000, "две ошибки и idle")
    assert mon.sems == [5], mon.sems                  # sem только у чистой команды
    assert mon.errs == [21, 23], mon.errs
    assert words(mem.read_bytes(base2, 2 * BURST_BYTES)) == spm[40:56]
    assert words(mem.read_bytes(base1, 2 * BURST_BYTES)) == spm[0:16]       # burst'ы до ошибки целы
    assert words(mem.read_bytes(base1 + 3 * BURST_BYTES, BURST_BYTES)) == spm[24:32]


@cocotb.test()
async def test_go_pause(dut):
    """go=0: новые AW не выдаются, выданные доводятся до конца (quiet); go=1 — продолжение."""
    mem = await setup(dut, seed=13, lat=(30, 40))
    rng = random.Random(6)
    spm = [rng.randrange(1 << 256) for _ in range(1024)]
    start_spm(dut, spm, rng, 0.2)
    mon = Mon(dut)
    await RisingEdge(dut.clk)
    nb = 10
    await push(dut, 2, 0, 0x60000, nb, 41)
    await wait_for(lambda: mon.aw_hs >= 2, dut, 2000, "первые AW")
    dut.go.value = 0
    await wait_quiet(dut)
    frozen = mon.aw_hs
    assert 2 <= frozen < nb
    assert mon.w_hs == frozen * BURST                  # W выданных AW дописаны
    for _ in range(150):
        await RisingEdge(dut.clk)
    assert mon.aw_hs == frozen and not mon.sems
    dut.go.value = 1
    await wait_for(lambda: mon.sems, dut, 4000, "sem после go=1")
    assert mon.sems == [2]
    assert words(mem.read_bytes(0x60000, nb * BURST_BYTES)) == spm[:nb * BURST]


@cocotb.test()
async def test_abort_with_w_in_flight(dut):
    """abort при W/B в полёте: W выданных AW дописан, B приняты, sem нет, cmd_rdy=0;
    после soft_rst (при quiet) новая команда проходит."""
    mem = await setup(dut, seed=17, lat=(20, 40), p_stall=0.6)
    rng = random.Random(9)
    spm = [rng.randrange(1 << 256) for _ in range(1024)]
    start_spm(dut, spm, rng, 0.1)
    mon = Mon(dut)
    await RisingEdge(dut.clk)
    nb = 12
    await push(dut, 3, 0, 0x70000, nb, 51)
    await wait_for(lambda: mon.aw_hs >= 1, dut, 2000, "первый AW")
    dut.abort.value = 1
    await ReadOnly()
    assert int(dut.cmd_rdy.value) == 0
    await wait_quiet(dut)
    n_aw = mon.aw_hs
    assert 1 <= n_aw < nb, n_aw
    assert mem.stats["aw"] == n_aw == mem.stats["b"] and mem.stats["w_beats"] == n_aw * BURST
    assert words(mem.read_bytes(0x70000, n_aw * BURST_BYTES)) == spm[:n_aw * BURST]
    for _ in range(100):                              # abort держится: ни AW, ни sem, ни команд
        await RisingEdge(dut.clk)
        await ReadOnly()
        assert int(dut.cmd_rdy.value) == 0
    assert mon.aw_hs == n_aw and not mon.sems
    # soft_rst при quiet, затем abort снят — модуль в исходном состоянии
    await RisingEdge(dut.clk)
    dut.soft_rst.value = 1
    await RisingEdge(dut.clk)
    dut.soft_rst.value = 0
    dut.abort.value = 0
    await RisingEdge(dut.clk)
    await ReadOnly()
    assert int(dut.idle.value) == 1 and int(dut.inflight.value) == 0
    await RisingEdge(dut.clk)
    await push(dut, 5, 200, 0x80000, 2, 52)
    await wait_for(lambda: mon.sems, dut, 3000, "sem после soft_rst")
    assert mon.sems == [5]
    assert words(mem.read_bytes(0x80000, 2 * BURST_BYTES)) == spm[200:216]
