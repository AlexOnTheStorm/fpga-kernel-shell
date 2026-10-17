# Самопроверка HbmModel без симулятора: такт модели (_drive/_sample) гоняется на
# фейковых сигналах против фейкового AXI-мастера на Python (pytest).
import random
from collections import deque

import pytest
from hbm_model import HbmModel

READ_SIGS = dict(arvalid=1, arready=1, araddr=40, arid=4, arlen=8, arsize=3, arburst=2,
                 arcache=4, rvalid=1, rready=1, rdata=256, rid=4, rresp=2, rlast=1)
WRITE_SIGS = dict(awvalid=1, awready=1, awaddr=40, awid=2, awlen=8, awsize=3, awburst=2,
                  awcache=4, wvalid=1, wready=1, wdata=256, wstrb=32, wlast=1,
                  bvalid=1, bready=1, bid=2, bresp=2)


class Sig:
    def __init__(self, width):
        self.value = 0
        self._w = width

    def __len__(self):
        return self._w


class FakeDut:
    def __init__(self, read=True, write=True):
        self.clk = Sig(1)
        for group, on in ((READ_SIGS, read), (WRITE_SIGS, write)):
            if on:
                for n, w in group.items():
                    setattr(self, f"m_axi_{n}", Sig(w))

    def __getattr__(self, n):           # короткий доступ: d.arvalid → d.m_axi_arvalid
        try:
            return self.__dict__[f"m_axi_{n}"]
        except KeyError:
            raise AttributeError(n) from None


def step(model, masters):
    for m in masters:
        m.drive()
    model._drive()
    model._sample()
    for m in masters:
        m.sample()
    model.cycle += 1


class ReadMaster:
    """AR по списку адресов; ID из пула свободных (same_id=False) или все с ID 0
    (same_id=True: до max_out в полёте, ответы одного ID обязаны идти по порядку AR —
    beat приписывается самому старому burst'у этого ID). RREADY случайный."""

    def __init__(self, d, addrs, ids=16, p_rready=1.0, seed=1, ar=(7, 5, 1),
                 same_id=False, max_out=16):
        self.d, self.todo, self.rng, self.p_rready = d, list(addrs), random.Random(seed), p_rready
        self.free = [0] * max_out if same_id else list(range(ids))
        self.inflight = {}                 # id -> deque([addr, beats]) в порядке AR
        self.done = []                     # (addr, [(data, resp)]) в порядке завершения
        self.order = []                    # id каждого принятого R-beat'а
        self.ar = ar
        self.prev = None                   # (rvalid, rid, rdata, rlast, rresp) без handshake

    def drive(self):
        d = self.d
        if not int(d.arvalid.value) and self.todo and self.free:
            i = self.free.pop(0)
            a = self.todo.pop(0)
            self.inflight.setdefault(i, deque()).append([a, []])
            d.arvalid.value, d.araddr.value, d.arid.value = 1, a, i
            d.arlen.value, d.arsize.value, d.arburst.value = self.ar
        d.rready.value = int(self.rng.random() < self.p_rready)

    def sample(self):
        d = self.d
        if int(d.arvalid.value) and int(d.arready.value):
            d.arvalid.value = 0
        if self.prev is not None:          # выставленный и не принятый beat — стабилен
            assert self.prev == self._r(), "R изменился без handshake"
        self.prev = None
        if not int(d.rvalid.value):
            return
        if not int(d.rready.value):
            self.prev = self._r()
            return
        rid = int(d.rid.value)
        a, beats = self.inflight[rid][0]
        beats.append((int(d.rdata.value), int(d.rresp.value)))
        self.order.append(rid)
        assert int(d.rlast.value) == (len(beats) == 8)
        if len(beats) == 8:
            self.done.append((a, beats))
            self.inflight[rid].popleft()
            if not self.inflight[rid]:
                del self.inflight[rid]
            self.free.append(rid)

    def _r(self):
        d = self.d
        return tuple(int(getattr(d, n).value) for n in ("rvalid", "rid", "rdata", "rlast", "rresp"))

    def finished(self):
        return not self.todo and not self.inflight


class WriteMaster:
    """bursts: (addr, id, [(data, strb)]×8). w_first=True — W раньше AW."""

    def __init__(self, d, bursts, w_first=False, wlast_at=7, aw=(7, 5, 1)):
        self.d, self.aw_todo, self.w_todo, self.n = d, list(bursts), list(bursts), len(bursts)
        self.w_first, self.wlast_at, self.aw = w_first, wlast_at, aw
        self.w_beat = 0
        self.w_started = 0                 # burst'ов, чей первый W уже выставлен
        self.b = []                        # (bid, bresp)

    def drive(self):
        d = self.d
        d.bready.value = 1
        if not int(d.wvalid.value) and self.w_todo:
            if self.w_beat == 0:
                self.w_started += 1
            data, strb = self.w_todo[0][2][self.w_beat]
            d.wvalid.value, d.wdata.value, d.wstrb.value = 1, data, strb
            d.wlast.value = int(self.w_beat == self.wlast_at)
        aw_sent = self.n - len(self.aw_todo)
        if not int(d.awvalid.value) and self.aw_todo and (not self.w_first or self.w_started > aw_sent):
            a, i, _ = self.aw_todo.pop(0)
            d.awvalid.value, d.awaddr.value, d.awid.value = 1, a, i
            d.awlen.value, d.awsize.value, d.awburst.value = self.aw

    def sample(self):
        d = self.d
        if int(d.awvalid.value) and int(d.awready.value):
            d.awvalid.value = 0
        if int(d.wvalid.value) and int(d.wready.value):
            d.wvalid.value = 0
            self.w_beat += 1
            if self.w_beat == 8:
                self.w_beat = 0
                self.w_todo.pop(0)
        if int(d.bvalid.value) and int(d.bready.value):
            self.b.append((int(d.bid.value), int(d.bresp.value)))


def run(model, masters, done, limit=20000):
    for _ in range(limit):
        step(model, masters)
        if done():
            return
    raise AssertionError("не завершилось")


def mem_words(model, a):
    return [int.from_bytes(model.read_bytes(a + 32 * k, 32), "little") for k in range(8)]


@pytest.mark.parametrize("interleave", [False, True])
def test_read_ooo(interleave):
    d = FakeDut(write=False)
    m = HbmModel(d, has_write=False, lat=(5, 80), seed=3, interleave=interleave)
    data = bytes(random.Random(0).randrange(256) for _ in range(256 * 64))
    m.write_bytes(0x10000, data)
    addrs = [0x10000 + 256 * k for k in range(64)]
    rm = ReadMaster(d, addrs, p_rready=0.7)
    run(m, [rm], rm.finished)
    assert sorted(a for a, _ in rm.done) == addrs
    for a, beats in rm.done:
        assert [w for w, _ in beats] == mem_words(m, a) and all(r == 0 for _, r in beats)
    assert m.stats["ooo"] > 0
    if interleave:
        assert m.stats["interleaved"] > 0
    else:                                  # burst подряд: смена ID только на границе burst'а
        assert m.stats["interleaved"] == 0
        assert all(len(set(rm.order[k:k + 8])) == 1 for k in range(0, len(rm.order), 8))


@pytest.mark.parametrize("interleave", [False, True])
def test_read_same_id_in_order_and_slverr(interleave):
    d = FakeDut(write=False)
    m = HbmModel(d, has_write=False, lat=(1, 100), seed=9, slverr={0x2300}, interleave=interleave)
    data = bytes(random.Random(5).randrange(256) for _ in range(256 * 32))
    m.write_bytes(0x2000, data)
    addrs = [0x2000 + 256 * k for k in range(32)]
    rm = ReadMaster(d, addrs, same_id=True, max_out=16)   # один ID, 16 в полёте → по порядку
    run(m, [rm], rm.finished)
    assert [a for a, _ in rm.done] == addrs
    for a, beats in rm.done:
        assert [w for w, _ in beats] == mem_words(m, a)
        assert all(r == (2 if a == 0x2300 else 0) for _, r in beats)


def test_pc_serves_in_order():
    # один PC → готовность монотонна → без interleave ответы строго в порядке AR
    d = FakeDut(write=False)
    m = HbmModel(d, has_write=False, n_pc=1, lat=(1, 100), seed=11)
    addrs = [0x8000 + 256 * k for k in range(40)]
    rm = ReadMaster(d, addrs)
    run(m, [rm], rm.finished)
    assert [a for a, _ in rm.done] == addrs and m.stats["ooo"] == 0


@pytest.mark.parametrize("w_first", [False, True])
def test_write_strb_b_order(w_first):
    d = FakeDut(read=False)
    m = HbmModel(d, has_read=False, lat=(3, 40), seed=4, slverr={0x5100})
    rng = random.Random(2)
    m.write_bytes(0x5000, bytes([0xEE]) * 256 * 6)
    bursts = []
    for k in range(6):
        beats = [(rng.randrange(1 << 256), rng.choice([(1 << 32) - 1, 0x0000FFFF, 0xF0F0F0F0]))
                 for _ in range(8)]
        bursts.append((0x5000 + 256 * k, k % 2, beats))
    wm = WriteMaster(d, bursts, w_first=w_first)
    run(m, [wm], lambda: len(wm.b) == 6)
    for a, _, beats in bursts:
        got = m.read_bytes(a, 256)
        for j, (data, strb) in enumerate(beats):
            raw = data.to_bytes(32, "little")
            for i in range(32):
                exp = 0xEE if a == 0x5100 or not (strb >> i) & 1 else raw[i]
                assert got[32 * j + i] == exp
    assert [r for _, r in wm.b].count(2) == 1
    for i in (0, 1):                       # B одного ID — по порядку AW
        assert [b for b in wm.b if b[0] == i] == [(i, 2 if (i, a) == (1, 0x5100) else 0)
                                                  for a, ii, _ in bursts if ii == i]


def test_b_in_order_per_id():
    # SLVERR на каждом третьем burst'е — метка, по которой видна перестановка B внутри ID
    d = FakeDut(read=False)
    bad = {0x9000 + 256 * k for k in range(0, 24, 3)}
    m = HbmModel(d, has_read=False, lat=(1, 100), seed=6, slverr=bad)
    bursts = [(0x9000 + 256 * k, k % 2, [(k, (1 << 32) - 1)] * 8) for k in range(24)]
    wm = WriteMaster(d, bursts)
    run(m, [wm], lambda: len(wm.b) == 24)
    for i in (0, 1):
        assert [r for b, r in wm.b if b == i] == [2 if a in bad else 0 for a, ii, _ in bursts if ii == i]
    assert [b for b, _ in wm.b] != [k % 2 for k in range(24)]   # между ID — OoO


@pytest.mark.parametrize("ar,addr,msg", [((3, 5, 1), 0x1000, "ARLEN"), ((7, 4, 1), 0x1000, "ARSIZE"),
                                         ((7, 5, 0), 0x1000, "ARBURST"), ((7, 5, 1), 0x1F80, "4 KB")])
def test_ar_protocol_checks(ar, addr, msg):
    d = FakeDut(write=False)
    m = HbmModel(d, has_write=False, p_stall=0)
    rm = ReadMaster(d, [addr], ar=ar)
    with pytest.raises(AssertionError, match=msg):
        run(m, [rm], rm.finished, limit=10)


@pytest.mark.parametrize("wlast_at", [3, 8])
def test_wlast_check(wlast_at):
    d = FakeDut(read=False)
    m = HbmModel(d, has_read=False, p_stall=0)
    wm = WriteMaster(d, [(0, 0, [(0, (1 << 32) - 1)] * 8)], wlast_at=wlast_at)
    with pytest.raises(AssertionError, match="WLAST"):
        run(m, [wm], lambda: len(wm.b) == 1, limit=100)


def test_freeze():
    d = FakeDut()
    m = HbmModel(d, lat=(30, 30), seed=1)
    rm = ReadMaster(d, [0x0, 0x100])
    for _ in range(10):
        step(m, [rm])
    m.freeze()
    for _ in range(200):
        step(m, [rm])
        assert not int(d.arready.value) and not int(d.awready.value) and not int(d.wready.value)
        assert not int(d.rvalid.value)
    assert not rm.done
