# Модель памяти HBM: AXI4 slave с out-of-order ответами (§16).
#
# - память — страницы по 4 KB (bytearray), непрописанное читается нулями;
# - n_pc псевдоканалов, PC = (addr >> 8) % n_pc; у PC своя очередь: burst готов через
#   randint(*lat) тактов после приёма, но не раньше предыдущего burst'а этого PC
#   (PC отвечает по порядку, конвейерно — задержка без потолка полосы);
# - R: среди готовых burst'ов (по одному на ID — голова ID, AXI порядок по ID) —
#   в порядке готовности → OoO по ID; interleave=True — beat'ы разных готовых burst'ов
#   чередуются случайно; иначе burst идёт подряд;
# - B: по порядку для одного ID, между ID — в порядке готовности;
# - случайные паузы ARREADY/AWREADY/WREADY с вероятностью p_stall;
# - slverr: адреса начала burst'ов, на которые RRESP/BRESP = 2 (запись не выполняется);
# - freeze(): перестать отвечать (уже выставленный beat/B держится до handshake);
# - проверки протокола (AssertionError → тест падает): AxLEN = 7, AxSIZE = 5,
#   AxBURST = INCR, адрес кратен 32, burst не пересекает 4 KB, WLAST ровно на beat'е AxLEN.
#
# Такт: после фронта _drive() выставляет выходы модели, в ReadOnly _sample() снимает
# handshake'и. Логика такта не зависит от cocotb — её можно гонять на фейковых сигналах.
import random
from collections import deque

import cocotb
from cocotb.triggers import ReadOnly, RisingEdge

PAGE = 4096
BEATS = 8           # AxLEN + 1
BB = 32             # байт на beat (AxSIZE = 5)
INCR = 1
OKAY, SLVERR = 0, 2


class _Burst:
    __slots__ = ("id", "addr", "seq", "pc", "ready", "beat", "resp")

    def __init__(self, bid, addr, seq, pc, resp):
        self.id, self.addr, self.seq, self.pc, self.resp = bid, addr, seq, pc, resp
        self.ready = None
        self.beat = 0


class HbmModel:
    def __init__(self, dut, prefix="m_axi", n_pc=8, lat=(20, 60), seed=0,
                 p_stall=0.2, interleave=False, slverr=None, has_read=True, has_write=True):
        self.dut = dut
        self.n_pc, self.lat, self.p_stall, self.interleave = n_pc, lat, p_stall, interleave
        self.rng = random.Random(seed)
        self.slverr = set(slverr or ())
        self.has_read, self.has_write = has_read, has_write
        self.frozen = False
        self.cycle = 0
        self._task = None
        self._pages = {}
        self._seq = 0
        # чтение
        self._rd = []                    # принятые AR без последнего R, в порядке приёма
        self._r_cur = None               # burst, чей beat выставлен на R в этом такте
        self._r_hold = False             # beat выставлен и не принят (RREADY = 0) — держим
        self._r_active = None            # без interleave: burst в процессе выдачи
        self._r_pc_t = [0] * n_pc
        self._arready = False
        # запись
        self._aw_q = deque()             # принятые AW, W ещё не дописан
        self._w_buf = deque()            # W-beat'ы, ещё не сопоставленные с AW
        self._b = []                     # дописанные burst'ы, ждут B, в порядке AW
        self._b_cur = None
        self._b_hold = False
        self._w_pc_t = [0] * n_pc
        self._awready = self._wready = False
        # счётчики (для отладки и проверки, что OoO/чередование реально были)
        self.stats = dict(ar=0, aw=0, r_beats=0, w_beats=0, b=0, ooo=0, interleaved=0)
        self._r_last_id = None

        def sig(name, optional=False):
            full = f"{prefix}_{name}"
            if hasattr(dut, full):
                return getattr(dut, full)
            if optional:
                return None
            raise AttributeError(f"HbmModel: у DUT нет сигнала {full}")

        if has_read:
            self.arvalid, self.arready, self.araddr = sig("arvalid"), sig("arready"), sig("araddr")
            self.arlen, self.arsize, self.arburst = sig("arlen"), sig("arsize"), sig("arburst")
            self.arid = sig("arid", True)
            self.rvalid, self.rready, self.rdata = sig("rvalid"), sig("rready"), sig("rdata")
            self.rresp, self.rlast, self.rid = sig("rresp"), sig("rlast"), sig("rid", True)
        if has_write:
            self.awvalid, self.awready, self.awaddr = sig("awvalid"), sig("awready"), sig("awaddr")
            self.awlen, self.awsize, self.awburst = sig("awlen"), sig("awsize"), sig("awburst")
            self.awid = sig("awid", True)
            self.wvalid, self.wready, self.wdata = sig("wvalid"), sig("wready"), sig("wdata")
            self.wstrb, self.wlast = sig("wstrb", True), sig("wlast")
            self.bvalid, self.bready, self.bresp = sig("bvalid"), sig("bready"), sig("bresp")
            self.bid = sig("bid", True)
        self._drive_idle()

    # ------------------------------------------------------------------ память

    def write_bytes(self, addr, data):
        data = bytes(data)
        off = 0
        while off < len(data):
            pg, po = divmod(addr + off, PAGE)
            n = min(PAGE - po, len(data) - off)
            page = self._pages.setdefault(pg, bytearray(PAGE))
            page[po:po + n] = data[off:off + n]
            off += n

    def read_bytes(self, addr, n):
        out = bytearray()
        while len(out) < n:
            pg, po = divmod(addr + len(out), PAGE)
            k = min(PAGE - po, n - len(out))
            page = self._pages.get(pg)
            out += page[po:po + k] if page is not None else bytes(k)
        return bytes(out)

    # ------------------------------------------------------------------ управление

    def freeze(self):
        """Перестать отвечать: READY низкие, новых R/B нет (для watchdog)."""
        self.frozen = True

    def start(self):
        """Запустить корутину каналов (AR/R и/или AW/W/B)."""
        if self._task is None:
            self._task = cocotb.start_soon(self._run())

    def stop(self):
        """Убить корутину и отпустить выходы (перед повторным bringup в одном тесте)."""
        if self._task is not None:
            self._task.cancel()
            self._task = None
        try:
            self._drive_idle()
        except Exception:        # вызов в фазе ReadOnly — выходы перехватит новая модель
            pass

    async def _run(self):
        clk = self.dut.clk
        while True:
            self._drive()
            await ReadOnly()
            self._sample()
            await RisingEdge(clk)
            self.cycle += 1

    # ------------------------------------------------------------------ такт

    def _drive_idle(self):
        if self.has_read:
            self.arready.value = 0
            self.rvalid.value = 0
        if self.has_write:
            self.awready.value = 0
            self.wready.value = 0
            self.bvalid.value = 0

    def _go(self):
        return not self.frozen and self.rng.random() >= self.p_stall

    def _drive(self):
        if self.has_read:
            self._arready = self._go()
            self.arready.value = int(self._arready)
            if not self._r_hold:
                b = None if self.frozen else self._pick_r()
                self._r_cur = b
                self.rvalid.value = int(b is not None)
                if b is not None:
                    a = b.addr + b.beat * BB
                    self.rdata.value = int.from_bytes(self.read_bytes(a, BB), "little")
                    self.rresp.value = b.resp
                    self.rlast.value = int(b.beat == BEATS - 1)
                    if self.rid is not None:
                        self.rid.value = b.id
        if self.has_write:
            self._awready = self._go()
            self._wready = self._go()
            self.awready.value = int(self._awready)
            self.wready.value = int(self._wready)
            if not self._b_hold:
                b = None if self.frozen else self._pick(self._b, None)
                self._b_cur = b
                self.bvalid.value = int(b is not None)
                if b is not None:
                    self.bresp.value = b.resp
                    if self.bid is not None:
                        self.bid.value = b.id

    def _pick(self, pending, active):
        """Голова каждого ID (AXI: по ID по порядку); из готовых — раньше всех готовый."""
        if active is not None:
            return active
        heads = {}
        for b in pending:
            heads.setdefault(b.id, b)
        ready = [b for b in heads.values() if b.ready <= self.cycle]
        if not ready:
            return None
        return min(ready, key=lambda b: (b.ready, b.seq))

    def _pick_r(self):
        if not self.interleave:
            self._r_active = self._pick(self._rd, self._r_active)
            return self._r_active
        heads = {}
        for b in self._rd:
            heads.setdefault(b.id, b)
        ready = [b for b in heads.values() if b.ready <= self.cycle]
        return self.rng.choice(ready) if ready else None

    def _sample(self):
        if self.has_read:
            if self._arready and int(self.arvalid.value):
                self._accept_ar()
            if self._r_cur is not None:
                if int(self.rready.value):
                    self._r_beat_done(self._r_cur)
                    self._r_hold = False
                else:
                    self._r_hold = True
        if self.has_write:
            if self._awready and int(self.awvalid.value):
                self._accept_aw()
            if self._wready and int(self.wvalid.value):
                strb = int(self.wstrb.value) if self.wstrb is not None else (1 << BB) - 1
                self._w_buf.append((int(self.wdata.value), strb, int(self.wlast.value)))
                self.stats["w_beats"] += 1
            self._match_w()
            if self._b_cur is not None:
                if int(self.bready.value):
                    self._b.remove(self._b_cur)
                    self.stats["b"] += 1
                    self._b_hold = False
                else:
                    self._b_hold = True

    # ------------------------------------------------------------------ каналы

    def _check_ax(self, ch, addr, ln, size, burst):
        assert ln == BEATS - 1, f"{ch}LEN={ln} (ожидается {BEATS - 1}), addr={addr:#x}"
        assert size == 5, f"{ch}SIZE={size} (ожидается 5), addr={addr:#x}"
        assert burst == INCR, f"{ch}BURST={burst} (ожидается INCR), addr={addr:#x}"
        assert addr % BB == 0, f"{ch}ADDR={addr:#x} не кратен {BB}"
        assert (addr % PAGE) + BEATS * BB <= PAGE, f"{ch}: burst {addr:#x} пересекает 4 KB"

    def _new_burst(self, ch, addr_s, id_s, len_s, size_s, burst_s):
        addr = int(addr_s.value)
        self._check_ax(ch, addr, int(len_s.value), int(size_s.value), int(burst_s.value))
        bid = int(id_s.value) if id_s is not None else 0
        b = _Burst(bid, addr, self._seq, (addr >> 8) % self.n_pc,
                   SLVERR if addr in self.slverr else OKAY)
        self._seq += 1
        return b

    def _ready_time(self, pc_t, pc):
        t = max(self.cycle + self.rng.randint(*self.lat), pc_t[pc])
        pc_t[pc] = t
        return t

    def _accept_ar(self):
        b = self._new_burst("AR", self.araddr, self.arid, self.arlen, self.arsize, self.arburst)
        b.ready = self._ready_time(self._r_pc_t, b.pc)
        self._rd.append(b)
        self.stats["ar"] += 1

    def _r_beat_done(self, b):
        self.stats["r_beats"] += 1
        if self._r_last_id is not None and self._r_last_id != b.id and b.beat != 0:
            self.stats["interleaved"] += 1           # продолжили burst после чужого beat'а
        self._r_last_id = b.id
        b.beat += 1
        if b.beat == BEATS:
            if self._rd[0] is not b:
                self.stats["ooo"] += 1               # обогнал более ранний AR
            self._rd.remove(b)
            if self._r_active is b:
                self._r_active = None

    def _accept_aw(self):
        b = self._new_burst("AW", self.awaddr, self.awid, self.awlen, self.awsize, self.awburst)
        self._aw_q.append(b)
        self.stats["aw"] += 1

    def _match_w(self):
        """W идут по порядку AW (AXI4: без W-interleaving); W может прийти раньше AW."""
        while self._aw_q and self._w_buf:
            b = self._aw_q[0]
            data, strb, last = self._w_buf.popleft()
            final = b.beat == BEATS - 1
            assert bool(last) == final, \
                f"WLAST={last} на W-beat'е {b.beat} burst'а AW {b.addr:#x} (ожидается на {BEATS - 1})"
            if b.resp == OKAY:
                self._write_beat(b.addr + b.beat * BB, data, strb)
            b.beat += 1
            if final:
                self._aw_q.popleft()
                b.ready = self._ready_time(self._w_pc_t, b.pc)
                self._b.append(b)

    def _write_beat(self, addr, data, strb):
        raw = data.to_bytes(BB, "little")
        if strb == (1 << BB) - 1:
            self.write_bytes(addr, raw)
            return
        for i in range(BB):
            if (strb >> i) & 1:
                self.write_bytes(addr + i, raw[i:i + 1])
