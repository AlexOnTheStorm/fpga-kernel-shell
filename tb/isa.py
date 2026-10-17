# ISA kshell: кодировщик дескрипторов (§6), построитель программ и статический
# проверщик правил корректной программы (§6.1).
from collections import deque

OP = dict(LOAD=1, STREAM=2, STORE=3, WAIT=4, SIGNAL=5, RUN=6)
OP_NAME = {v: k for k, v in OP.items()}
BURST_BYTES = 256
BB = 32
SPM_WORDS = 1024
AW = 40
NSEM = 8
EQ_DEPTH = 4            # глубина очереди каждого движка (§3)

_DMA = ("LOAD", "STREAM", "STORE")
_M32 = (1 << 32) - 1


def _fit(name, val, bits):
    if not 0 <= val < (1 << bits):
        raise ValueError(f"{name}={val:#x} не влезает в {bits} бит")
    return val


def encode(op, sem=0, spm=0, hbm=0, length=0, w1=None, args=None):
    """Вернуть (CMD0, CMD1, CMD2, CMD3) по §6.
    LOAD/STREAM/STORE: sem, spm, hbm, length. WAIT/SIGNAL: w1 = thr/inc → CMD1.
    RUN: args = (a0, a1, a2, a3) → CMD1, CMD2, CMD3, CMD0[15:0] (удобнее через Program.run())."""
    name = op if isinstance(op, str) else OP_NAME[op]
    code = OP[name]
    c0 = (code << 28) | (_fit("sem", sem, 3) << 24)
    if name in _DMA:
        c0 |= _fit("spm", spm, 16)
        _fit("hbm", hbm, AW)
        return (c0, hbm & _M32, hbm >> 32, _fit("len", length, 32))
    if name in ("WAIT", "SIGNAL"):
        return (c0, _fit("w1", 0 if w1 is None else w1, 32), 0, 0)
    a0, a1, a2, a3 = args if args is not None else (0, 0, 0, 0)
    c0 |= _fit("arg3", a3, 16)
    return (c0, _fit("arg0", a0, 32), _fit("arg1", a1, 32), _fit("arg2", a2, 32))


def decode(c):
    """Обратное к encode: dict с полями по коду операции; op = None для BAD_OPCODE."""
    c0, c1, c2, c3 = c
    code = (c0 >> 28) & 0xF
    d = dict(op=OP_NAME.get(code), code=code, sem=(c0 >> 24) & 7, encoded=tuple(c))
    if d["op"] in _DMA:
        d.update(spm=c0 & 0xFFFF, hbm=((c2 & 0xFF) << 32) | c1, len=c3)
    elif d["op"] == "WAIT":
        d.update(thr=c1)
    elif d["op"] == "SIGNAL":
        d.update(inc=c1)
    elif d["op"] == "RUN":
        d.update(args=(c1, c2, c3, c0 & 0xFFFF))
    return d


class Program:
    """Построитель программы. cmds — list[dict]: op, поля команды, encoded."""

    def __init__(self):
        self.cmds = []

    def _add(self, c):
        self.cmds.append(decode(c))
        return self

    def load(self, sem, spm, hbm, length):
        return self._add(encode("LOAD", sem=sem, spm=spm, hbm=hbm, length=length))

    def stream(self, hbm, length):
        return self._add(encode("STREAM", hbm=hbm, length=length))

    def store(self, sem, spm, hbm, length):
        return self._add(encode("STORE", sem=sem, spm=spm, hbm=hbm, length=length))

    def wait(self, sem, thr):
        return self._add(encode("WAIT", sem=sem, w1=thr))

    def signal(self, sem, inc):
        return self._add(encode("SIGNAL", sem=sem, w1=inc))

    def run(self, sem, a0, a1, a2, a3):
        return self._add(encode("RUN", sem=sem, args=(a0, a1, a2, a3)))

    def raw(self, c0, c1, c2, c3):
        """Сырой дескриптор как есть (например BAD_OPCODE)."""
        return self._add(tuple(x & _M32 for x in (c0, c1, c2, c3)))

    def encoded(self):
        return [c["encoded"] for c in self.cmds]

    def increments(self, sem):
        """Сколько приращений выдано на sem: LOAD/STORE/RUN — по 1, SIGNAL — inc."""
        n = 0
        for c in self.cmds:
            if c["sem"] != sem:
                continue
            if c["op"] in ("LOAD", "STORE", "RUN"):
                n += 1
            elif c["op"] == "SIGNAL":
                n += c["inc"]
        return n

    def __len__(self):
        return len(self.cmds)


class ProgramError(Exception):
    pass


# --------------------------------------------------------------------------- проверщик


def _signed32(v):
    v &= _M32
    return v - (1 << 32) if v >> 31 else v


def _gemv_regions(a):
    a0, a1, a2, a3 = a
    return [(a0, a0 + a3 // BB)], [(a1, a1 + (a2 + 7) // 8)]


def _ranges(x):
    """Нормализовать (lo, hi) или список таких пар в список пар [lo, hi)."""
    if len(x) == 2 and all(isinstance(v, int) for v in x):
        x = [x]
    return [(lo, hi) for lo, hi in x if hi > lo]


def _overlap(ra, rb):
    for lo1, hi1 in ra:
        for lo2, hi2 in rb:
            if lo1 < hi2 and lo2 < hi1:
                return (max(lo1, lo2), min(hi1, hi2))
    return None


def _desc(i, c):
    return f"#{i} {c['op'] or 'BAD'}"


def _check_decode(cmds):
    for i, c in enumerate(cmds):
        if c["op"] is None:
            raise ProgramError(f"decode: {_desc(i, c)} — BAD_OPCODE {c['code']}")
        if c["op"] in _DMA:
            ln, hbm = c["len"], c["hbm"]
            if ln == 0 or ln % BURST_BYTES or hbm % BURST_BYTES or hbm + ln > (1 << AW):
                raise ProgramError(f"decode: {_desc(i, c)} — ALIGN (hbm={hbm:#x}, len={ln})")
            if c["op"] != "STREAM" and c["spm"] + ln // BB > SPM_WORDS:
                raise ProgramError(f"decode: {_desc(i, c)} — RANGE (spm={c['spm']}, len={ln})")


def _pair_stream_run(cmds):
    """Правило 1. Вернуть {индекс RUN: индекс STREAM} и обратное."""
    streams = [i for i, c in enumerate(cmds) if c["op"] == "STREAM"]
    runs = [i for i, c in enumerate(cmds) if c["op"] == "RUN"]
    if len(streams) != len(runs):
        raise ProgramError(f"rule 1: STREAM×{len(streams)} != RUN×{len(runs)}")
    for k, (s, r) in enumerate(zip(streams, runs)):
        if s > r:
            raise ProgramError(f"rule 1: STREAM #{s} (пара {k}) стоит после своего RUN #{r}")
    return dict(zip(runs, streams)), dict(zip(streams, runs))


def _execute(cmds, run_of_stream, ntag, eq_depth):
    """Правило 2: абстрактное исполнение «всё, что может завершиться, завершается».
    Память отвечает всегда; STORE завершается всегда; голова окна STREAM снимается,
    только пока ядро исполняет RUN этой STREAM; секвенсор стоит на WAIT и на полной
    очереди движка. Неподвижная точка с pc < len — тупик."""
    sem = [0] * NSEM
    rd_q, krn_q = deque(), deque()
    slicer = None            # [индекс команды, осталось burst'ов]
    window = deque()         # (индекс команды, последний burst команды)
    krn = None               # индекс исполняемого RUN
    done = set()
    pc, n = 0, len(cmds)
    cap = (lambda q: True) if eq_depth is None else (lambda q: len(q) < eq_depth)
    progress = True
    while progress:
        progress = False
        if pc < n:                                             # секвенсор
            c, ok = cmds[pc], True
            if c["op"] == "WAIT":
                ok = _signed32(sem[c["sem"]] - c["thr"]) >= 0
            elif c["op"] == "SIGNAL":
                sem[c["sem"]] += c["inc"]
            elif c["op"] in ("LOAD", "STREAM"):
                ok = cap(rd_q)
                if ok:
                    rd_q.append(pc)
            elif c["op"] == "STORE":
                sem[c["sem"]] += 1                             # wr-движок не блокируется
                done.add(pc)
            elif c["op"] == "RUN":
                ok = cap(krn_q)
                if ok:
                    krn_q.append(pc)
            if ok:
                pc += 1
                progress = True
        if slicer is None and rd_q:                            # резчик rd
            i = rd_q.popleft()
            slicer = [i, cmds[i]["len"] // BURST_BYTES]
            progress = True
        while slicer is not None and len(window) < ntag:
            slicer[1] -= 1
            window.append((slicer[0], slicer[1] == 0))
            if slicer[1] == 0:
                slicer = None
            progress = True
        if krn is None and krn_q:                              # ядро
            krn = krn_q.popleft()
            progress = True
        while window:                                          # коммит по порядку
            i, last = window[0]
            if cmds[i]["op"] == "STREAM" and run_of_stream[i] != krn:
                break
            window.popleft()
            progress = True
            if last:
                done.add(i)
                if cmds[i]["op"] == "LOAD":
                    sem[cmds[i]["sem"]] += 1
                else:                                          # поток RUN кончился → k_done
                    done.add(krn)
                    sem[cmds[krn]["sem"]] += 1
                    krn = None
    if pc == n and not (rd_q or slicer or window or krn_q or krn is not None):
        return
    where = f"{_desc(pc, cmds[pc])}" if pc < n else "конец программы"
    hol_possible = pc < n
    if pc < n and cmds[pc]["op"] == "WAIT":
        s = cmds[pc]["sem"]
        where += f" (sem{s} ≥ {cmds[pc]['thr']}, достижимо {sem[s]})"
        # head-of-line, только если WAIT ждёт ещё не завершённые команды
        hol_possible = any(i not in done and cmds[i]["sem"] == s
                           and cmds[i]["op"] in ("LOAD", "STORE", "RUN") for i in range(pc))
    elif pc < n:
        where += " (очередь движка полна)"
    stuck = sorted(i for i in run_of_stream if i not in done and run_of_stream[i] >= pc)
    if stuck and hol_possible:
        s = stuck[0]
        raise ProgramError(f"rule 2: head-of-line deadlock at {where}: STREAM #{s} держит "
                           f"rd-движок, а её RUN #{run_of_stream[s]} ещё не выдан")
    raise ProgramError(f"deadlock at {where}: условие никогда не выполнится")


def _regions(i, c, run_regions):
    """Области команды: dict space -> (reads, writes); space 'spm' (слова) / 'mem' (байты)."""
    op = c["op"]
    if op in _DMA:
        mem = [(c["hbm"], c["hbm"] + c["len"])]
        if op == "STREAM":
            return dict(mem=(mem, []))
        spm = [(c["spm"], c["spm"] + c["len"] // BB)]
        if op == "LOAD":
            return dict(mem=(mem, []), spm=([], spm))
        return dict(mem=([], mem), spm=(spm, []))
    if op == "RUN" and run_regions is not None:
        fn = _gemv_regions if run_regions == "gemv" else run_regions
        rd, wr = fn(c["args"])
        return dict(spm=(_ranges(rd), _ranges(wr)))
    return {}


def _check_data(cmds, stream_of_run, run_regions):
    """Правило 3. implied[x] — команды, завершённые к моменту завершения x
    (коммит rd по порядку; ядро по порядку и после своей STREAM; B STORE по порядку).
    WAIT sem ≥ thr гарантирует A, если без A (и без всего, что требует A) значение
    семафора не может дойти до thr."""
    implied = {}
    last = {}                  # последняя выданная команда движка
    issued = []                # выданные команды с семафорами и областями
    known = set()              # гарантированно завершены
    sig = [0] * NSEM           # сумма SIGNAL
    for i, c in enumerate(cmds):
        op = c["op"]
        if op == "WAIT":
            s, thr = c["sem"], _signed32(c["thr"])
            incs = [x for x in issued if cmds[x]["op"] != "STREAM" and cmds[x]["sem"] == s]
            for a in issued:
                if a in known:
                    continue
                vmax = sig[s] + sum(1 for x in incs if a not in implied[x])
                if vmax < thr:
                    known |= implied[a]
            continue
        if op == "SIGNAL":
            sig[c["sem"]] += c["inc"]
            continue
        eng = dict(LOAD="rd", STREAM="rd", STORE="wr", RUN="krn")[op]
        imp = {i} | implied.get(last.get(eng), set())
        if op == "RUN":
            imp |= implied[stream_of_run[i]]
        implied[i] = imp
        last[eng] = i
        reg = _regions(i, c, run_regions)
        for a in issued:
            if a in known or {op, cmds[a]["op"]} in ({"RUN"}, {"STORE"}):
                continue                     # ядро и B STORE — строго по порядку
            ra = _regions(a, cmds[a], run_regions)
            for space in ("spm", "mem"):
                if space not in reg or space not in ra:
                    continue
                (ar, aw), (br, bw) = ra[space], reg[space]
                hit = _overlap(aw, br) or _overlap(aw, bw) or _overlap(ar, bw)
                if hit:
                    raise ProgramError(
                        f"rule 3: {_desc(i, c)} пересекается с {_desc(a, cmds[a])} по {space} "
                        f"[{hit[0]:#x}, {hit[1]:#x}) без WAIT на семафор {_desc(a, cmds[a])}")
        issued.append(i)


def check_program(prog, ntag=16, run_regions="gemv", eq_depth=EQ_DEPTH):
    """Абстрактное исполнение §6.1; ProgramError при нарушении правил 1–3:
    1) пары STREAM–RUN: число STREAM == число RUN, k-я STREAM раньше k-го RUN;
    2) head-of-line: WAIT, условие которого зависит от команды rd, стоящей после
       STREAM, чей RUN ещё не выдан, — тупик (также тупик на полной очереди движка,
       eq_depth=None отключает модель очередей; WAIT, который не выполнится никогда);
    3) пересечение областей spm/памяти без WAIT на семафор предыдущей команды.
    Также проверяются ошибки декодирования (ALIGN, RANGE, BAD_OPCODE).
    run_regions: "gemv" — области RUN по правилу GEMV (читает spm[a0, a0+a3/32),
    пишет spm[a1, a1+ceil(a2/8))); None — RUN не трогает spm (программы без RUN
    или с sink_kernel); callable(args) -> (reads, writes), каждое — (lo, hi) или
    список таких пар в словах spm."""
    cmds = prog.cmds
    _check_decode(cmds)
    stream_of_run, run_of_stream = _pair_stream_run(cmds)
    _execute(cmds, run_of_stream, ntag, eq_depth)
    _check_data(cmds, stream_of_run, run_regions)
