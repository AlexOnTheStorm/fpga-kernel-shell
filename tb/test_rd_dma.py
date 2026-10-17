# Модульные тесты rd_dma (docs/design.md §8): окно завершений + ROB поверх OoO-ответов
# HbmModel. Первые три теста — из брифа Task 3; остальные закрывают требования, которые
# бриф не проверяет: пауза go=0, очередь LOAD→STREAM→LOAD с head-of-line, заполнение
# окна, abort посреди STREAM и восстановление soft_rst, perf-выходы.
import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ReadOnly, RisingEdge

from hbm_model import HbmModel

BURST = 8
BB = 32


def ntag(dut):
    return 1 << len(dut.m_axi_arid)          # параметр NTAG сборки (ARID = log2 NTAG бит)


def words(data):
    return [int.from_bytes(data[i * BB:(i + 1) * BB], "little") for i in range(len(data) // BB)]


async def setup(dut, **mk):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    for s in ("soft_rst", "abort", "cmd_v", "k_s_tready"):
        getattr(dut, s).value = 0
    dut.go.value = 1
    dut.now.value = 0
    dut.rst_n.value = 0
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    mem = HbmModel(dut, has_write=False, **mk)
    mem.start()

    async def tick():
        n = 0
        while True:
            await RisingEdge(dut.clk)
            n += 1
            dut.now.value = n
    cocotb.start_soon(tick())
    return mem


async def push(dut, stream, sem, spm, hbm, nb, no):
    dut.cmd_v.value = 1
    dut.cmd_stream.value = stream
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
    """Пассивный монитор выходов: spm-записи, семафоры, ошибки, AXIS, perf, окно."""

    def __init__(self, dut):
        self.dut = dut
        self.spm, self.sems, self.errs = {}, [], []
        self.axis, self.lasts = [], []
        self.ar_hs = self.lat_v = self.win_full = 0
        self.lats = []
        self.max_inflight = 0
        cocotb.start_soon(self._run())

    async def _run(self):
        d = self.dut
        while True:
            await ReadOnly()
            if int(d.spm_wr_v.value):
                self.spm[int(d.spm_wr_addr.value)] = int(d.spm_wr_data.value)
            if int(d.sem_inc_v.value):
                self.sems.append(int(d.sem_inc_idx.value))
            if int(d.err_v.value):
                self.errs.append(int(d.err_cmd.value))
            if int(d.k_s_tvalid.value) and int(d.k_s_tready.value):
                self.axis.append(int(d.k_s_tdata.value))
                self.lasts.append(int(d.k_s_tlast.value))
            self.ar_hs += int(d.pf_ar_hs.value)
            self.win_full += int(d.pf_win_full.value)
            if int(d.pf_lat_v.value):
                self.lat_v += 1
                self.lats.append(int(d.pf_lat.value))
            self.max_inflight = max(self.max_inflight, int(d.inflight.value))
            await RisingEdge(d.clk)


async def wait_for(dut, cond, n, what):
    for _ in range(n):
        await RisingEdge(dut.clk)
        await ReadOnly()
        if cond():
            await RisingEdge(dut.clk)
            return
    raise AssertionError(f"таймаут: {what}")


# ---------------------------------------------------------------- тесты брифа

@cocotb.test(timeout_time=200, timeout_unit="us")
async def test_load_ooo_commit_in_order(dut):
    NTAG = ntag(dut)
    mem = await setup(dut, lat=(5, 80), seed=7)
    data = bytes(random.Random(1).randrange(256) for _ in range(256 * 40))
    mem.write_bytes(0x10000, data)
    m = Mon(dut)
    await push(dut, 0, 3, 0, 0x10000, 40, 1)            # 40 burst'ов > NTAG
    for _ in range(4000):
        await RisingEdge(dut.clk)
        if m.sems:
            break
    assert m.sems == [3]
    assert [m.spm[i] for i in range(320)] == words(data)
    await ReadOnly()
    assert int(dut.axi_quiet.value) == 1
    assert mem.stats["ooo"] > 0, "модель не дала ни одного OoO-ответа — тест ничего не проверил"
    # perf: AR handshake'и и задержки — по одному на burst; задержка ≥ минимальной модели
    assert m.ar_hs == 40 and m.lat_v == 40
    assert min(m.lats) >= 5
    assert m.max_inflight <= NTAG
    await RisingEdge(dut.clk)
    await ReadOnly()
    assert int(dut.idle.value) == 1 and int(dut.inflight.value) == 0


@cocotb.test(timeout_time=200, timeout_unit="us")
async def test_stream_order_under_backpressure(dut):
    mem = await setup(dut, lat=(5, 80), seed=9, interleave=True)
    data = bytes(random.Random(2).randrange(256) for _ in range(256 * 24))
    mem.write_bytes(0x20000, data)
    got = []
    lasts = []
    rng = random.Random(3)

    async def sink():
        while len(got) < 24 * 8:
            dut.k_s_tready.value = 1 if rng.random() < 0.6 else 0
            await ReadOnly()
            if int(dut.k_s_tvalid.value) and int(dut.k_s_tready.value):
                got.append(int(dut.k_s_tdata.value))
                lasts.append(int(dut.k_s_tlast.value))
            await RisingEdge(dut.clk)
    s = cocotb.start_soon(sink())
    await push(dut, 1, 0, 0, 0x20000, 24, 2)
    await s
    assert got == words(data)
    assert lasts == [0] * 191 + [1]
    assert mem.stats["interleaved"] > 0 and mem.stats["ooo"] > 0


@cocotb.test(timeout_time=200, timeout_unit="us")
async def test_rresp_error_abort(dut):
    mem = await setup(dut, lat=(5, 40), seed=11, slverr={0x30000 + 256 * 3})
    m = Mon(dut)
    await push(dut, 0, 1, 0, 0x30000, 8, 42)
    # бриф ждал 50 тактов; ответ burst'а 3 может прийти позже (очередь R) — ждём до ошибки
    for _ in range(400):
        await RisingEdge(dut.clk)
        if m.errs:
            dut.abort.value = 1
            dut.go.value = 0
            break
    for _ in range(500):
        await RisingEdge(dut.clk)
        if int(dut.axi_quiet.value):
            break
    assert m.errs[:1] == [42] and m.sems == [] and int(dut.axi_quiet.value) == 1
    assert len(m.errs) == 1, f"ожидался один импульс err_v на burst с ошибкой: {m.errs}"
    assert mem.stats["ar"] == m.ar_hs and mem.stats["r_beats"] == 8 * m.ar_hs, \
        "каждый выданный AR обязан получить все свои R (RREADY = 1)"


# ---------------------------------------------------------------- дополнительные

@cocotb.test(timeout_time=200, timeout_unit="us")
async def test_pause_go0(dut):
    """go=0: резчик стоит на границе burst'а, выданные burst'ы и коммиты идут до конца,
    axi_quiet = 1; go=1 — продолжение с того же места."""
    mem = await setup(dut, lat=(20, 60), seed=21)
    data = bytes(random.Random(4).randrange(256) for _ in range(256 * 32))
    mem.write_bytes(0x40000, data)
    m = Mon(dut)
    await push(dut, 0, 5, 100, 0x40000, 32, 7)
    await wait_for(dut, lambda: m.ar_hs >= 6, 200, "6 AR")
    dut.go.value = 0
    n_at_pause = m.ar_hs
    await wait_for(dut, lambda: int(dut.axi_quiet.value) == 1, 1000, "axi_quiet на паузе")
    assert m.ar_hs - n_at_pause <= 1, "на паузе выдан новый AR (кроме удерживаемого arvalid)"
    for _ in range(100):                                   # долгая пауза: ничего не выдаётся
        await RisingEdge(dut.clk)
    await ReadOnly()
    paused = m.ar_hs
    assert paused < 32 and int(dut.inflight.value) == 0, "коммиты LOAD должны идти на паузе"
    assert m.sems == [] and int(dut.idle.value) == 0     # команда ещё в резчике
    assert int(dut.cmd_rdy.value) == 0
    assert [m.spm[100 + i] for i in range(paused * BURST)] == words(data)[:paused * BURST]
    await RisingEdge(dut.clk)
    dut.go.value = 1
    await wait_for(dut, lambda: bool(m.sems), 4000, "sem после продолжения")
    assert m.sems == [5] and m.ar_hs == 32
    assert [m.spm[100 + i] for i in range(256)] == words(data)


@cocotb.test(timeout_time=200, timeout_unit="us")
async def test_load_stream_load_hol_and_window_full(dut):
    """LOAD → STREAM (20 burst'ов > NTAG) → LOAD. Пока ядро не принимает поток: окно
    заполнено (≤ NTAG, pf_win_full), второй LOAD блокирован head-of-line. R9: команда
    снимается с очереди сразу, как резчик свободен."""
    NTAG = ntag(dut)
    mem = await setup(dut, lat=(5, 60), seed=31)
    rng = random.Random(5)
    d_l1 = bytes(rng.randrange(256) for _ in range(256 * 4))
    d_st = bytes(rng.randrange(256) for _ in range(256 * 20))
    d_l2 = bytes(rng.randrange(256) for _ in range(256 * 4))
    mem.write_bytes(0x50000, d_l1)
    mem.write_bytes(0x60000, d_st)
    mem.write_bytes(0x70000, d_l2)
    m = Mon(dut)
    pushed = []

    async def queue():
        for c in ((0, 1, 0, 0x50000, 4, 10), (1, 0, 0, 0x60000, 20, 11),
                  (0, 2, 64, 0x70000, 4, 12)):
            await push(dut, *c)
            pushed.append(c[-1])
    q = cocotb.start_soon(queue())
    for _ in range(400):                                    # ядро стоит (tready = 0)
        await RisingEdge(dut.clk)
    await ReadOnly()
    assert m.sems == [1], "первый LOAD обязан закоммититься"
    assert m.max_inflight == NTAG and int(dut.inflight.value) == NTAG
    assert m.win_full > 0, "pf_win_full не поднялся при полном окне"
    assert m.ar_hs == 4 + NTAG and mem.stats["ar"] == 4 + NTAG
    assert pushed == [10, 11], "STREAM должна быть снята в резчик, LOAD2 ждёт резчик"
    assert m.axis == []
    await RisingEdge(dut.clk)
    sink_rng = random.Random(6)

    async def sink():
        while True:
            dut.k_s_tready.value = 1 if sink_rng.random() < 0.7 else 0
            await RisingEdge(dut.clk)
    cocotb.start_soon(sink())
    await wait_for(dut, lambda: len(m.sems) == 2, 4000, "sem LOAD2")
    await q
    assert m.sems == [1, 2]
    assert m.axis == words(d_st) and m.lasts == [0] * 159 + [1]
    assert [m.spm[i] for i in range(32)] == words(d_l1)
    assert [m.spm[64 + i] for i in range(32)] == words(d_l2)
    await ReadOnly()
    assert int(dut.idle.value) == 1 and int(dut.axi_quiet.value) == 1


@cocotb.test(timeout_time=200, timeout_unit="us")
async def test_stream_error_abort_then_soft_rst(dut):
    """SLVERR в STREAM: burst с ошибкой и всё после него не попадают в AXIS (выдан только
    верный префикс), sem не растёт, axi_quiet; soft_rst → idle, следующая команда верна."""
    bad = 0x80000 + 256 * 5
    mem = await setup(dut, lat=(5, 40), seed=41, slverr={bad})
    rng = random.Random(7)
    d_st = bytes(rng.randrange(256) for _ in range(256 * 12))
    d_l = bytes(rng.randrange(256) for _ in range(256 * 3))
    mem.write_bytes(0x80000, d_st)
    mem.write_bytes(0x90000, d_l)
    m = Mon(dut)
    dut.k_s_tready.value = 1
    await push(dut, 1, 0, 0, 0x80000, 12, 99)
    await wait_for(dut, lambda: bool(m.errs), 1000, "err_v")
    # abort с запозданием: burst с SLVERR успевает стать done и дойти до головы —
    # голова обязана стоять на нём, а не выдавать его в поток
    for _ in range(150):
        await RisingEdge(dut.clk)
    await ReadOnly()
    assert m.axis == words(d_st)[:5 * BURST], "до abort должны выйти ровно burst'ы 0..4"
    assert int(dut.k_s_tvalid.value) == 0 and int(dut.idle.value) == 0
    await RisingEdge(dut.clk)
    dut.abort.value = 1
    dut.go.value = 0
    await wait_for(dut, lambda: int(dut.axi_quiet.value) == 1, 1000, "axi_quiet после abort")
    for _ in range(50):                                     # окно сливается без выдачи
        await RisingEdge(dut.clk)
    await ReadOnly()
    assert m.errs == [99]
    assert m.axis == words(d_st)[:5 * BURST], "после abort в AXIS ничего не выдано"
    assert 1 not in m.lasts
    assert int(dut.inflight.value) == 0 and int(dut.k_s_tvalid.value) == 0
    assert int(dut.idle.value) == 1, "после abort окно пусто и резчик сбросил команду"
    assert int(dut.cmd_rdy.value) == 0, "при abort команды не принимаются"
    assert mem.stats["r_beats"] == 8 * mem.stats["ar"]
    await RisingEdge(dut.clk)
    dut.soft_rst.value = 1
    dut.abort.value = 0
    await RisingEdge(dut.clk)
    dut.soft_rst.value = 0
    dut.go.value = 1
    await ReadOnly()
    assert int(dut.idle.value) == 1 and int(dut.cmd_rdy.value) == 1
    await RisingEdge(dut.clk)
    m.spm.clear()
    await push(dut, 0, 6, 200, 0x90000, 3, 100)
    await wait_for(dut, lambda: bool(m.sems), 1000, "sem после soft_rst")
    assert m.sems == [6] and m.errs == [99]
    assert [m.spm[200 + i] for i in range(24)] == words(d_l)


@cocotb.test(timeout_time=200, timeout_unit="us")
async def test_stream_full_rate(dut):
    """2-местный выходной буфер держит 1 beat/такт: окно, заполненное готовыми STREAM-
    burst'ами, выдаётся NTAG·BURST beat'ов подряд без пузырей при tready = 1."""
    NTAG = ntag(dut)
    mem = await setup(dut, lat=(5, 30), seed=51)
    data = bytes(random.Random(8).randrange(256) for _ in range(256 * NTAG))
    mem.write_bytes(0xA0000, data)
    m = Mon(dut)
    await push(dut, 1, 0, 0, 0xA0000, NTAG, 3)
    await wait_for(dut, lambda: int(dut.axi_quiet.value) == 1, 2000, "все R пришли")
    assert m.axis == [] and int(dut.inflight.value) == NTAG
    dut.k_s_tready.value = 1
    first = None
    cyc = 0
    while len(m.axis) < NTAG * BURST and cyc < 1000:
        await RisingEdge(dut.clk)
        await ReadOnly()
        cyc += 1
        if first is None and int(dut.k_s_tvalid.value):
            first = cyc
    assert m.axis == words(data) and m.lasts == [0] * (NTAG * BURST - 1) + [1]
    span = cyc - first + 1
    assert span == NTAG * BURST, f"выдача заняла {span} тактов вместо {NTAG * BURST}"
    await RisingEdge(dut.clk)
    await ReadOnly()
    assert int(dut.idle.value) == 1


@cocotb.test(timeout_time=200, timeout_unit="us")
async def test_load_error_suppresses_sem_without_abort(dut):
    """Накопитель ошибки команды (§8.2/§8.5) сам по себе, без abort: SLVERR в burst'е 3
    из 8 → после коммита последнего (хорошего) burst'а sem не растёт; накопитель
    сбрасывается на границе команды — следующий LOAD даёт sem."""
    mem = await setup(dut, lat=(5, 40), seed=61, slverr={0xB0000 + 256 * 3})
    data = bytes(random.Random(9).randrange(256) for _ in range(256 * 2))
    mem.write_bytes(0xC0000, data)
    m = Mon(dut)
    await push(dut, 0, 2, 0, 0xB0000, 8, 70)
    await push(dut, 0, 4, 500, 0xC0000, 2, 71)
    await wait_for(dut, lambda: bool(m.sems), 2000, "sem второго LOAD")
    await ReadOnly()
    assert m.sems == [4] and m.errs == [70]
    assert [m.spm[500 + i] for i in range(16)] == words(data)
    assert int(dut.idle.value) == 1
