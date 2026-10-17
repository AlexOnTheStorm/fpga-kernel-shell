# Модульные тесты cmd_seq (docs/design.md §6, §7, §11, §12.2).
# Первый тест — из брифа Task 5; остальные закрывают то, что бриф не проверяет:
# все шесть операций с точными полями (RUN arg3 из CMD0[15:0]), ALIGN/RANGE/BAD_OPCODE
# на границах (33-битная сумма), знаковое сравнение WAIT через переполнение и «следующий
# такт», очистка при error (FIFO выкачана, WAIT прерван, ничего не выдано), пауза
# enable=0, стоп на полной очереди (порядок строго сохраняется), случайный прогон
# против эталонной модели.
import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ReadOnly, RisingEdge

from isa import decode, encode

NSEM = 8
M32 = (1 << 32) - 1
E_ALIGN, E_RANGE, E_OPCODE = 3, 4, 5


def pack(c):
    if isinstance(c, int):  # уже упакован
        return c
    return c[0] | (c[1] << 32) | (c[2] << 64) | (c[3] << 96)


def raw(op, sem=0, lo16=0, c1=0, c2=0, c3=0):
    """Сырой дескриптор (в обход проверок isa.encode)."""
    return ((op << 28) | (sem << 24) | lo16, c1, c2, c3)


def ref_err(c):
    """Эталон проверок декодирования §6 на неограниченных целых."""
    c0, c1, c2, c3 = c
    code = c0 >> 28
    if code not in (1, 2, 3, 4, 5, 6):
        return E_OPCODE
    if code in (1, 2, 3):
        spm, hbm, ln = c0 & 0xFFFF, ((c2 & 0xFF) << 32) | c1, c3
        if ln == 0 or ln % 256 or hbm % 256 or hbm + ln > (1 << 40):
            return E_ALIGN
        if code != 2 and spm + ln // 32 > 1024:
            return E_RANGE
    return 0


def ref_event(c, no):
    """Событие, которое секвенсор обязан выдать для верной команды c под номером no."""
    d = decode(c)
    op = d["op"]
    if op in ("LOAD", "STREAM"):
        return ("rd", int(op == "STREAM"), d["sem"], None if op == "STREAM" else d["spm"] & 0x3FF,
                d["hbm"], d["len"] >> 8, no)
    if op == "STORE":
        return ("wr", d["sem"], d["spm"] & 0x3FF, d["hbm"], d["len"] >> 8, no)
    if op == "RUN":
        return ("krn", d["sem"], d["args"], no)
    if op == "SIGNAL":
        return ("sig", d["sem"], d["inc"])
    return None  # WAIT событий не даёт


def ref_events(cmds):
    """Ожидаемые события программы: верные команды по порядку, затем ошибка первой плохой."""
    ev = []
    for no, c in enumerate(cmds):
        code = ref_err(c)
        if code:
            ev.append(("err", code, no))
            break
        e = ref_event(c, no)
        if e:
            ev.append(e)
    return ev


class Env:
    """Модель окружения: командная FIFO, три очереди движков (rdy случайно), семафоры
    (sig_v виден со следующего такта), липкий error от kshell (с задержкой err_lat после
    dec_err_v; None — error ведёт тест). Такт = от фронта до фронта; выходы читаются в
    ReadOnly сразу после фронта, пока входы этого такта уже выставлены."""

    def __init__(self, dut, seed=1, err_lat=2):
        self.dut = dut
        self.rng = random.Random(seed)
        self.err_lat = err_lat
        self.cq = []
        self.sem = [0] * NSEM
        self.rdy = dict(rd=1.0, wr=1.0, krn=1.0)  # вероятность rdy в такте
        self.enable = 1
        self.p_enable = None  # если задано — enable случайно с этой вероятностью
        self.error = 0
        self.err_at = None
        self.soft_rst = 0
        self.cyc = 0
        self.events = []  # (cyc, event)
        self.trace = []  # снимки по тактам
        self.popped = 0

    async def start(self):
        cocotb.start_soon(Clock(self.dut.clk, 10, unit="ns").start())
        d = self.dut
        d.rst_n.value = 0
        d.soft_rst.value = 0
        d.enable.value = 1
        d.error.value = 0
        d.cq_v.value = 0
        d.cq_d.value = 0
        d.rd_rdy.value = 1
        d.wr_rdy.value = 1
        d.krn_rdy.value = 1
        d.sem_val.value = 0
        await RisingEdge(d.clk)
        await RisingEdge(d.clk)
        d.rst_n.value = 1

    async def step(self, record=True):
        d = self.dut
        if self.err_at is not None and self.cyc >= self.err_at:
            self.error = 1
        en = self.enable if self.p_enable is None else int(self.rng.random() < self.p_enable)
        head = self.cq[0] if self.cq else None
        rdy = {k: int(self.rng.random() < p) for k, p in self.rdy.items()}
        d.enable.value = en
        d.error.value = self.error
        d.soft_rst.value = self.soft_rst
        d.cq_v.value = int(head is not None)
        d.cq_d.value = pack(head) if head is not None else 0
        d.rd_rdy.value, d.wr_rdy.value, d.krn_rdy.value = rdy["rd"], rdy["wr"], rdy["krn"]
        d.sem_val.value = sum(v << (32 * i) for i, v in enumerate(self.sem))
        await ReadOnly()
        g = lambda n: int(getattr(d, n).value)
        s = dict(cyc=self.cyc, en=en, error=self.error, rdy=rdy, head=head)
        for n in ("rd_v", "wr_v", "krn_v", "sig_v", "dec_err_v", "cq_pop", "cmd_no", "in_wait",
                  "pf_wait", "rd_nb", "rd_no", "rd_hbm", "rd_spm", "rd_sem", "rd_stream"):
            s[n] = g(n)
        self.trace.append(s)
        valids = s["rd_v"] + s["wr_v"] + s["krn_v"] + s["sig_v"]
        assert valids <= 1, f"cyc {self.cyc}: больше одной команды за такт {s}"
        if s["cq_pop"]:
            assert head is not None, f"cyc {self.cyc}: cq_pop без cq_v"
            self.cq.pop(0)
            self.popped += 1
        inc = None
        if record:
            if (self.error or not en) and valids:
                raise AssertionError(f"cyc {self.cyc}: выдача при error={self.error} enable={en}")
            ev = None
            if s["rd_v"] and rdy["rd"]:
                ev = ("rd", s["rd_stream"], s["rd_sem"], None if s["rd_stream"] else s["rd_spm"],
                      s["rd_hbm"], s["rd_nb"], s["rd_no"])
            elif s["wr_v"] and rdy["wr"]:
                ev = ("wr", g("wr_sem"), g("wr_spm"), g("wr_hbm"), g("wr_nb"), g("wr_no"))
            elif s["krn_v"] and rdy["krn"]:
                arg = int(d.krn_arg.value)
                ev = ("krn", g("krn_sem"), tuple((arg >> (32 * i)) & M32 for i in range(4)),
                      g("krn_no"))
            elif s["sig_v"]:
                ev = ("sig", g("sig_idx"), g("sig_val"))
                inc = (ev[1], ev[2])
            if ev:
                self.events.append((self.cyc, ev))
            if s["dec_err_v"]:
                self.events.append((self.cyc, ("err", g("dec_err_code"), g("dec_err_cmd"))))
                if self.err_lat is not None and self.err_at is None:
                    self.err_at = self.cyc + self.err_lat
        await RisingEdge(d.clk)
        if inc:  # приращение семафора видно со следующего такта
            self.sem[inc[0]] = (self.sem[inc[0]] + inc[1]) & M32
        self.cyc += 1
        return s

    async def run(self, n):
        for _ in range(n):
            await self.step()

    async def run_until(self, pred, limit=3000):
        for _ in range(limit):
            await self.step()
            if pred():
                return
        raise AssertionError(f"таймаут {limit} тактов")

    async def soft_reset(self):
        """SOFT_RESET: сброс секвенсора и окружения (FIFO, error, семафоры)."""
        self.cq = []
        self.soft_rst = 1
        await self.step(record=False)
        self.soft_rst = 0
        self.error = 0
        self.err_at = None
        self.sem = [0] * NSEM
        self.enable = 1
        self.p_enable = None
        self.rdy = dict(rd=1.0, wr=1.0, krn=1.0)
        self.events = []

    def evs(self):
        return [e for _, e in self.events]


# --------------------------------------------------------------------------- тесты


@cocotb.test()
async def test_dispatch_wait_and_decode_errors(dut):
    """Тест из брифа: LOAD → WAIT → STORE только после семафора → ALIGN с номером 3."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst_n.value = 0
    dut.soft_rst.value = 0
    dut.enable.value = 1
    dut.error.value = 0
    dut.cq_v.value = 0
    dut.rd_rdy.value = 1
    dut.wr_rdy.value = 1
    dut.krn_rdy.value = 1
    dut.sem_val.value = 0
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    cmds = [encode("LOAD", sem=1, spm=0, hbm=0x1000, length=512),
            encode("WAIT", sem=1, w1=1),
            encode("STORE", sem=2, spm=0, hbm=0x2000, length=512),
            encode("LOAD", sem=1, spm=0, hbm=0x1001, length=512)]  # ALIGN
    seen = []
    i = 0
    for cyc in range(200):
        dut.cq_v.value = 1 if i < len(cmds) else 0
        dut.cq_d.value = pack(cmds[i]) if i < len(cmds) else 0
        if cyc == 50:
            dut.sem_val.value = 1 << 32  # sem1 = 1 → WAIT проходит
        await ReadOnly()
        if int(dut.rd_v.value):
            seen.append(("rd", int(dut.rd_nb.value)))
        if int(dut.wr_v.value):
            seen.append(("wr", cyc))
        if int(dut.dec_err_v.value):
            seen.append(("err", int(dut.dec_err_code.value), int(dut.dec_err_cmd.value)))
        pop = int(dut.cq_pop.value)
        await RisingEdge(dut.clk)
        if pop:
            i += 1
    assert seen[0] == ("rd", 2)
    assert seen[1][0] == "wr" and seen[1][1] >= 50  # STORE только после WAIT
    assert seen[2] == ("err", 3, 3)  # ALIGN, номер команды 3


@cocotb.test()
async def test_all_ops_fields(dut):
    """Все шесть операций: точные поля, номера команд с 0, RUN arg3 из CMD0[15:0]."""
    env = Env(dut)
    await env.start()
    cmds = [
        encode("LOAD", sem=5, spm=0x123, hbm=0xAB_CDEF_1200, length=3 * 256),
        encode("STREAM", sem=0, spm=0, hbm=0x01_0000_0000, length=0xFFFFFF00),  # nb = 0xFFFFFF
        encode("STORE", sem=7, spm=1000, hbm=0x1_0000_0100, length=768),  # 1000+24 = 1024
        encode("WAIT", sem=3, w1=0),
        encode("SIGNAL", sem=3, w1=0xDEADBEEF),
        encode("RUN", sem=6, args=(0x11111111, 0x22222222, 0x33333333, 0xBEEF)),
        encode("RUN", sem=0, args=(0xFFFFFFFF, 0, 0xFFFFFFFF, 0xFFFF)),
    ]
    env.cq = list(cmds)
    await env.run_until(lambda: not env.cq and len(env.events) == 6)
    await env.run(5)
    assert env.evs() == [
        ("rd", 0, 5, 0x123, 0xAB_CDEF_1200, 3, 0),
        ("rd", 1, 0, None, 0x01_0000_0000, 0xFFFFFF, 1),
        ("wr", 7, 1000, 0x1_0000_0100, 3, 2),
        ("sig", 3, 0xDEADBEEF),
        ("krn", 6, (0x11111111, 0x22222222, 0x33333333, 0xBEEF), 5),
        ("krn", 0, (0xFFFFFFFF, 0, 0xFFFFFFFF, 0xFFFF), 6),
    ], env.evs()
    assert env.trace[-1]["cmd_no"] == 7
    # без ожидания и стоянок команды идут по одной за такт подряд
    cycs = [c for c, _ in env.events]
    assert cycs[0:3] == [cycs[0], cycs[0] + 1, cycs[0] + 2]


@cocotb.test()
async def test_throughput_one_per_cycle(dut):
    """Back-to-back LOAD при rdy=1: одна выдача и один cq_pop за такт, без пузырей."""
    env = Env(dut)
    await env.start()
    env.cq = [pack(encode("LOAD", sem=1, spm=i, hbm=0x1000 * i, length=256)) for i in range(20)]
    await env.run_until(lambda: len(env.events) == 20)
    cycs = [c for c, _ in env.events]
    assert cycs == list(range(cycs[0], cycs[0] + 20)), cycs
    assert [e[6] for e in env.evs()] == list(range(20))


async def decode_case(dut, env, cmd):
    """Два верных SIGNAL, команда под проверкой, хвост из четырёх верных команд."""
    await env.soft_reset()
    tail = [encode("LOAD", sem=1, spm=0, hbm=0, length=256), encode("STORE", spm=0, hbm=0, length=256),
            encode("RUN", sem=1, args=(1, 2, 3, 4)), encode("SIGNAL", sem=1, w1=1)]
    pre = [encode("SIGNAL", sem=0, w1=1), encode("SIGNAL", sem=0, w1=2)]
    env.cq = [pack(c) for c in pre + [cmd] + tail]
    await env.run_until(lambda: not env.cq)
    await env.run(12)


@cocotb.test()
async def test_decode_errors_and_boundaries(dut):
    """ALIGN / RANGE / BAD_OPCODE на границах; ничего после плохой команды не выдаётся."""
    env = Env(dut)
    await env.start()
    top = (1 << 40) - 256
    cases = [
        # ---- ALIGN
        encode("LOAD", spm=0, hbm=0, length=0),
        encode("STREAM", hbm=0, length=0),
        encode("STORE", spm=0, hbm=0, length=0),
        encode("LOAD", spm=0, hbm=0, length=255),
        encode("LOAD", spm=0, hbm=0, length=300),
        encode("STREAM", hbm=0, length=257),
        encode("LOAD", spm=0, hbm=0x80, length=256),
        encode("STORE", spm=0, hbm=0x1, length=256),
        encode("STREAM", hbm=0xFF, length=256),
        encode("LOAD", spm=0, hbm=top, length=512),  # hbm+len = 2^40+256 > 2^40
        encode("STREAM", hbm=top, length=0xFFFFFF00),
        encode("STORE", spm=0, hbm=0xFF_FFFF_FF00, length=256 * 2),
        # ALIGN приоритетнее RANGE
        encode("LOAD", spm=2000, hbm=0, length=300),
        # ---- RANGE
        encode("LOAD", spm=1000, hbm=0, length=1024),  # 1000+32 > 1024
        encode("LOAD", spm=1023, hbm=0, length=256),  # 1023+8
        encode("STORE", spm=1017, hbm=0, length=256),  # 1017+8 = 1025
        encode("LOAD", spm=0, hbm=0, length=32768 + 256),  # 1024+8
        encode("LOAD", spm=0xFFFF, hbm=0, length=256),
        encode("LOAD", spm=0, hbm=0, length=0xFFFFFF00),  # len/32 ~ 2^27
        encode("STORE", spm=0x8000, hbm=0, length=0x100000),  # сумма 0x10000: 16 бит обнулили бы
        encode("LOAD", spm=1000, hbm=0, length=1048 * 32),  # 1000+1048 = 2048: 11 бит обнулили бы
        encode("LOAD", spm=1, hbm=0, length=0x1_0000_0000 - 256 * 4),  # много, но 32 бита len
        # ---- BAD_OPCODE
        raw(0, sem=1, lo16=0x1234, c1=0x1000, c3=256),
        raw(7), raw(8), raw(9), raw(10), raw(11), raw(12), raw(13), raw(14),
        raw(15, sem=7, lo16=0xFFFF, c1=M32, c2=M32, c3=M32),
        # ---- верные на границах (ошибки не должно быть)
        encode("LOAD", spm=1016, hbm=0, length=256),  # 1016+8 = 1024
        encode("STORE", spm=0, hbm=0, length=32768),  # ровно весь spm
        encode("LOAD", spm=0, hbm=top, length=256),  # последний burst памяти
        encode("STREAM", hbm=top, length=256),
        encode("STREAM", hbm=0, length=0xFFFFFF00),  # spm у STREAM не проверяется
        raw(2, lo16=0xFFFF, c1=0x100, c3=256),  # STREAM с мусором в spm_addr — верная
        raw(1, lo16=0, c1=0x100, c2=0xABCDEF00, c3=256),  # CMD2[31:8] игнорируются (hbm_hi = [7:0])
    ]
    for k, cmd in enumerate(cases):
        code = ref_err(cmd)
        await decode_case(dut, env, cmd)
        ev = env.evs()
        pre = [("sig", 0, 1), ("sig", 0, 2)]
        assert ev[:2] == pre, (k, cmd, ev)
        assert not env.cq
        if code:
            assert ev[2:] == [("err", code, 2)], (k, hex(cmd[0]), cmd, ref_err(cmd), ev)
            assert env.trace[-1]["cmd_no"] == 3, (k, "cmd_no должен остановиться на плохой")
        else:
            exp = [ref_event(cmd, 2)] + [ref_event(c, 3 + i) for i, c in enumerate([
                encode("LOAD", sem=1, spm=0, hbm=0, length=256), encode("STORE", spm=0, hbm=0, length=256),
                encode("RUN", sem=1, args=(1, 2, 3, 4)), encode("SIGNAL", sem=1, w1=1)])]
            assert ev[2:] == exp, (k, cmd, ev)


@cocotb.test()
async def test_decode_error_halts_without_kshell(dut):
    """Секвенсор останавливается сам на плохой команде, даже если error приходит поздно/никогда."""
    for lat in (None, 5):
        env = Env(dut, err_lat=lat)
        if lat is None:
            await env.start()
        await env.soft_reset()
        good = encode("SIGNAL", sem=0, w1=1)
        bad = encode("LOAD", spm=0, hbm=1, length=256)
        env.cq = [pack(good), pack(bad)] + [pack(good)] * 10
        await env.run_until(lambda: not env.cq)
        await env.run(10)
        assert env.evs() == [("sig", 0, 1), ("err", E_ALIGN, 1)], (lat, env.evs())


@cocotb.test()
async def test_wait_signed_compare(dut):
    """WAIT: $signed(sem − thr) >= 0, в том числе через переполнение 32 бит."""
    env = Env(dut)
    await env.start()
    # (sem, thr, проходит?)
    cases = [
        (0, 0, True), (1, 1, True), (0, 1, False), (5, 3, True), (3, 5, False),
        (M32, 0, False),  # sem = −1 < 0
        (0, M32, True),  # 0 − (−1) = 1
        (0x80000000, 0x7FFFFFFF, True),  # разность 1 через границу знака
        (0x7FFFFFFF, 0x80000000, False),  # разность −1
        (2, 0x80000003, True),  # thr «в прошлом» через переполнение
        (M32 - 1, 1, False),  # sem = −2, thr = 1
        (1, M32 - 1, True),  # 1 − (−2) = 3
        (0x00000001, 0x80000001, False),  # 1 − 0x80000001 = 0x80000000 = −2^31 (самая далёкая)
        (0x80000001, 0x00000001, False),  # 0x80000000 как знаковое = −2^31 < 0
        (0xFFFFFFF0, 0x00000010, False),  # −16 − 16 < 0
        (0x00000010, 0xFFFFFFF0, True),
    ]
    for k, (sem, thr, ok) in enumerate(cases):
        for idx in (0, 3, 7):
            await env.soft_reset()
            env.sem = [0x12345678] * NSEM  # чужие семафоры — мусор, не должны влиять
            env.sem[idx] = sem
            env.cq = [pack(encode("WAIT", sem=idx, w1=thr)), pack(encode("SIGNAL", sem=6, w1=1))]
            await env.run(8)
            if ok:
                assert env.evs() == [("sig", 6, 1)], (k, sem, thr, idx, env.evs())
                assert env.trace[-1]["in_wait"] == 0
            else:
                assert env.evs() == [], (k, sem, thr, idx)
                assert env.trace[-1]["in_wait"] == 1 and env.trace[-1]["pf_wait"] == 1
                assert len(env.cq) == 1  # SIGNAL за WAIT не взят
                env.sem[idx] = (thr + 1) & M32  # теперь проходит
                await env.run(4)
                assert env.evs() == [("sig", 6, 1)]


@cocotb.test()
async def test_wait_sees_sem_next_cycle(dut):
    """Изменение семафора видно WAIT со следующего такта; WAIT не барьер для SIGNAL перед ним."""
    env = Env(dut)
    await env.start()
    env.cq = [pack(encode("WAIT", sem=2, w1=1)), pack(encode("SIGNAL", sem=3, w1=9))]
    await env.run(10)
    assert env.evs() == [] and env.trace[-1]["in_wait"] == 1
    env.sem[2] = 1  # на входе с этого такта K → маркер SIGNAL виден в такте K+1
    k = env.cyc
    await env.run(5)
    assert env.events == [(k + 1, ("sig", 3, 9))], (k, env.events)
    assert env.trace[k]["in_wait"] == 1 and env.trace[k + 1]["in_wait"] == 0

    # собственный SIGNAL → WAIT на него: sig в такте c, sem виден в c+1, WAIT проходит в c+1,
    # маркер уходит в c+2
    await env.soft_reset()
    env.cq = [pack(encode("SIGNAL", sem=2, w1=1)), pack(encode("WAIT", sem=2, w1=1)),
              pack(encode("SIGNAL", sem=3, w1=7))]
    await env.run(10)
    (c0, e0), (c1, e1) = env.events
    assert (e0, e1) == (("sig", 2, 1), ("sig", 3, 7)) and c1 == c0 + 2, env.events


@cocotb.test()
async def test_pf_wait_counts_stalled_cycles(dut):
    env = Env(dut)
    await env.start()
    env.cq = [pack(encode("WAIT", sem=1, w1=1))]
    await env.run(20)
    stalled = sum(s["pf_wait"] for s in env.trace)
    assert 18 <= stalled <= 19, stalled  # пока WAIT в секвенсоре и условие ложно
    assert all(s["in_wait"] >= s["pf_wait"] for s in env.trace)
    env.sem[1] = 1
    await env.run(3)
    assert env.trace[-1]["pf_wait"] == 0 and env.trace[-1]["in_wait"] == 0


@cocotb.test()
async def test_error_flush(dut):
    """error: WAIT прерван, FIFO выкачана по одной команде за такт, ничего не выдано."""
    env = Env(dut, err_lat=None)
    await env.start()
    prog = [encode("WAIT", sem=2, w1=5), encode("LOAD", sem=1, spm=0, hbm=0, length=256),
            encode("STORE", spm=0, hbm=0, length=256), encode("RUN", sem=1, args=(1, 2, 3, 4)),
            encode("SIGNAL", sem=1, w1=1), encode("STREAM", hbm=0, length=256),
            encode("LOAD", sem=1, spm=0, hbm=0, length=256), encode("SIGNAL", sem=1, w1=1)]
    env.cq = [pack(c) for c in prog]
    await env.run(12)
    assert env.events == [] and len(env.cq) == len(prog) - 1
    assert env.trace[-1]["in_wait"] == 1 and env.trace[-1]["cmd_no"] == 1
    env.error = 1
    s = await env.step()
    assert s["in_wait"] == 0 and s["cq_pop"] == 1, s  # WAIT прерван сразу
    n = len(env.cq)
    for _ in range(n):
        s = await env.step()
        assert s["cq_pop"] == 1
    assert not env.cq
    assert env.events == []
    await env.run(3)
    assert env.trace[-1]["cmd_no"] == 1  # выброшенные команды не «взяты»
    # error липкий: новые команды тоже выкачиваются без выдачи, даже если WAIT бы прошёл
    env.sem[2] = 5
    env.cq = [pack(encode("LOAD", sem=1, spm=0, hbm=0, length=256)),
              pack(encode("SIGNAL", sem=1, w1=1))]
    await env.run(4)
    assert not env.cq and env.events == []
    # после SOFT_RESET всё заново: cmd_no = 0
    await env.soft_reset()
    env.cq = [pack(encode("LOAD", sem=1, spm=4, hbm=0x200, length=256))]
    await env.run_until(lambda: env.events)
    assert env.evs() == [("rd", 0, 1, 4, 0x200, 1, 0)]


@cocotb.test()
async def test_error_discards_held_command(dut):
    """Команда, застрявшая на полной очереди (взята, не выдана), при error не выдаётся никогда."""
    env = Env(dut, err_lat=None)
    await env.start()
    env.rdy["rd"] = 0.0
    env.cq = [pack(encode("LOAD", sem=1, spm=0, hbm=0, length=256)),
              pack(encode("SIGNAL", sem=1, w1=1))]
    await env.run(6)
    assert env.trace[-1]["rd_v"] == 1 and len(env.cq) == 1
    env.error = 1
    env.rdy["rd"] = 1.0
    s = await env.step()
    assert s["rd_v"] == 0
    await env.run(5)
    assert env.events == [] and not env.cq


@cocotb.test()
async def test_error_same_cycle_as_dispatch(dut):
    """error в тот же такт, когда команда выдавалась бы, подавляет выдачу."""
    env = Env(dut, err_lat=None)
    await env.start()
    env.cq = [pack(encode("LOAD", sem=1, spm=0, hbm=0, length=256))]
    s = await env.step()
    assert s["cq_pop"] == 1  # взята; в следующем такте уходила бы
    env.error = 1
    s = await env.step()
    assert s["rd_v"] == 0
    await env.run(3)
    assert env.events == []


@cocotb.test()
async def test_enable_pause(dut):
    """enable=0: новые команды не берутся, висящие не выдаются, WAIT стоит и перепроверяется."""
    env = Env(dut)
    await env.start()
    env.enable = 0
    env.cq = [pack(encode("SIGNAL", sem=1, w1=1)), pack(encode("LOAD", sem=1, spm=0, hbm=0, length=256))]
    await env.run(10)
    assert env.events == [] and env.popped == 0 and env.trace[-1]["cmd_no"] == 0
    env.enable = 1
    await env.run_until(lambda: len(env.events) == 2)
    assert env.evs()[0] == ("sig", 1, 1)

    # команда уже взята, но не выдана (очередь полна) → пауза → возобновление
    await env.soft_reset()
    env.rdy["rd"] = 0.0
    env.cq = [pack(encode("LOAD", sem=1, spm=0, hbm=0, length=256)),
              pack(encode("SIGNAL", sem=1, w1=1))]
    await env.run(4)
    assert env.trace[-1]["rd_v"] == 1
    env.enable = 0
    env.rdy["rd"] = 1.0
    await env.run(6)
    assert env.events == [] and len(env.cq) == 1
    assert env.trace[-1]["rd_v"] == 0
    env.enable = 1
    await env.run_until(lambda: len(env.events) == 2)
    assert env.evs() == [("rd", 0, 1, 0, 0, 1, 0), ("sig", 1, 1)]

    # WAIT в работе: пауза, условие стало истинным во время паузы — WAIT стоит; после
    # возобновления проходит
    await env.soft_reset()
    env.cq = [pack(encode("WAIT", sem=3, w1=1)), pack(encode("SIGNAL", sem=4, w1=1))]
    await env.run(5)
    assert env.trace[-1]["in_wait"] == 1
    env.enable = 0
    env.sem[3] = 1
    await env.run(6)
    assert env.events == [] and env.trace[-1]["in_wait"] == 1 and len(env.cq) == 1
    env.enable = 1
    k = env.cyc
    await env.run(4)
    assert env.events == [(k + 1, ("sig", 4, 1))], env.events
    # пауза посреди потока не теряет и не дублирует команды
    await env.soft_reset()
    env.cq = [pack(encode("SIGNAL", sem=0, w1=i)) for i in range(1, 30)]
    env.p_enable = 0.5
    await env.run_until(lambda: len(env.events) == 29)
    assert env.evs() == [("sig", 0, i) for i in range(1, 30)]


@cocotb.test()
async def test_queue_full_stall_and_order(dut):
    """Нужная очередь полна → секвенсор стоит; порядок команд между движками сохраняется."""
    env = Env(dut)
    await env.start()
    prog = [encode("LOAD", sem=1, spm=0, hbm=0, length=256),
            encode("STORE", sem=2, spm=0, hbm=0x100, length=256),
            encode("RUN", sem=3, args=(1, 2, 3, 4)),
            encode("SIGNAL", sem=4, w1=1),
            encode("LOAD", sem=1, spm=8, hbm=0x200, length=256)]
    for blocked, which in (("rd", "rd_v"), ("wr", "wr_v"), ("krn", "krn_v")):
        await env.soft_reset()
        env.rdy[blocked] = 0.0
        env.cq = [pack(c) for c in prog]
        await env.run(25)
        # команда движка blocked висит с rd_v/wr_v/krn_v и неизменными полями; дальше не идём
        first = {"rd": 0, "wr": 1, "krn": 2}[blocked]
        assert env.trace[-1][which] == 1
        assert len(env.cq) == len(prog) - (first + 1), (blocked, len(env.cq))
        held = [t for t in env.trace[-20:]]
        assert all(t[which] == 1 for t in held)
        assert all(t["rd_no"] == held[0]["rd_no"] and t["rd_hbm"] == held[0]["rd_hbm"] for t in held)
        # команды ДО заблокированной вышли, ПОСЛЕ — нет (даже в свободные очереди)
        assert len(env.events) == first, env.evs()
        env.rdy[blocked] = 1.0
        await env.run_until(lambda: len(env.events) == len(prog) - 0 and not env.cq)
        got = [e[0] for e in env.evs()]
        assert got == ["rd", "wr", "krn", "sig", "rd"], got
        assert [e[-1] for e in env.evs() if e[0] != "sig"] == [0, 1, 2, 4]


@cocotb.test()
async def test_random_against_reference(dut):
    """Случайные программы (верные и с плохой командой), случайные rdy/enable, WAIT по SIGNAL."""
    env = Env(dut, seed=7)
    await env.start()
    rng = random.Random(1234)
    for rnd in range(60):
        await env.soft_reset()
        init = [rng.choice([0, 0xFFFFFFF0, 0x7FFFFFF0, rng.getrandbits(32)]) for _ in range(NSEM)]
        env.sem = list(init)
        total = [0] * NSEM
        cmds = []
        bad_at = rng.randrange(1, 30) if rng.random() < 0.5 else None
        for i in range(rng.randint(1, 30)):
            if bad_at == i:
                cmds.append(rng.choice([
                    raw(rng.choice([0, 7, 8, 12, 15]), sem=rng.randrange(8), lo16=rng.getrandbits(16),
                        c1=rng.getrandbits(32), c3=rng.getrandbits(32)),
                    encode("LOAD", spm=0, hbm=rng.randrange(1, 256), length=256),
                    encode("STORE", spm=1020, hbm=0, length=256 * rng.randint(1, 8)),
                    encode("STREAM", hbm=0, length=rng.choice([0, 100, 257])),
                ]))
                continue
            kind = rng.choice(["LOAD", "STREAM", "STORE", "WAIT", "SIGNAL", "RUN", "SIGNAL", "WAIT"])
            sem = rng.randrange(NSEM)
            if kind == "SIGNAL":
                inc = rng.randint(0, 5)
                total[sem] += inc
                cmds.append(encode("SIGNAL", sem=sem, w1=inc))
            elif kind == "WAIT":
                if rng.random() < 0.3:
                    thr = (init[sem] - rng.randint(1, 100)) & M32  # уже прошёл
                else:
                    thr = (init[sem] + rng.randint(0, total[sem])) & M32
                cmds.append(encode("WAIT", sem=sem, w1=thr))
            elif kind == "RUN":
                cmds.append(encode("RUN", sem=sem, args=tuple(rng.getrandbits(32) for _ in range(3))
                                   + (rng.getrandbits(16),)))
            else:
                ln = 256 * rng.randint(1, 16)
                spm = rng.randrange(0, 1024 - ln // 32 + 1)
                hbm = 256 * rng.getrandbits(32) if rng.random() < 0.7 else (1 << 40) - ln
                c = encode(kind, sem=sem, spm=spm if kind != "STREAM" else 0, hbm=hbm, length=ln)
                if rng.random() < 0.3:
                    c = (c[0], c[1], c[2] | (rng.getrandbits(24) << 8), c[3])  # CMD2[31:8] игнорируются
                cmds.append(c)
        expect = ref_events(cmds)
        env.cq = [pack(c) for c in cmds]
        env.rdy = {k: rng.choice([0.2, 0.5, 1.0]) for k in ("rd", "wr", "krn")}
        env.p_enable = rng.choice([None, 0.7])
        await env.run_until(lambda: not env.cq and len(env.events) >= len(expect))
        await env.run(6)
        assert env.evs() == expect, (rnd, env.evs(), expect)
        taken = min(len(cmds), (bad_at + 1) if expect and expect[-1][0] == "err" else len(cmds))
        assert env.trace[-1]["cmd_no"] == taken, (rnd, env.trace[-1]["cmd_no"], taken)
