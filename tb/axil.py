# Драйвер AXI4-Lite master (32 бита данных). Одна транзакция за раз (Lock):
# write — AW и W выставляются вместе, каждый снимается по своему handshake, затем B;
# read — AR, затем R. Handshake снимается в ReadOnly, сигналы меняются после фронта.
# Ответ обёртки всегда OKAY (§4) — иное считается ошибкой теста.
from cocotb.triggers import Lock, ReadOnly, RisingEdge


class AxilMaster:
    def __init__(self, dut, prefix="s_axil", timeout=10000):
        self.dut = dut
        self.clk = dut.clk
        self.timeout = timeout
        self._lock = Lock()
        self._p = prefix
        for n in ("awvalid", "wvalid", "bready", "arvalid", "rready"):
            self._s(n).value = 0
        for n in ("awprot", "arprot"):                 # необязательные
            if hasattr(dut, f"{prefix}_{n}"):
                self._s(n).value = 0

    def _s(self, name):
        return getattr(self.dut, f"{self._p}_{name}")

    async def _until(self, name, what):
        """Ждать <name> = 1 (при уже выставленном ready/valid). Возвращает после фронта
        такта handshake'а; ReadOnly-значения сигналов фазы handshake — в self._seen."""
        for _ in range(self.timeout):
            await ReadOnly()
            hit = int(self._s(name).value)
            if hit:
                self._seen = {n: int(self._s(n).value) for n in what}
            await RisingEdge(self.clk)
            if hit:
                return
        raise TimeoutError(f"AXI-Lite: нет {self._p}_{name} за {self.timeout} тактов")

    async def write(self, addr: int, data: int) -> None:
        async with self._lock:
            await RisingEdge(self.clk)
            s = self._s
            s("awaddr").value = addr
            s("awvalid").value = 1
            s("wdata").value = data & 0xFFFFFFFF
            s("wstrb").value = 0xF
            s("wvalid").value = 1
            aw = w = False
            for _ in range(self.timeout):
                await ReadOnly()
                aw_hs = not aw and int(s("awready").value)
                w_hs = not w and int(s("wready").value)
                await RisingEdge(self.clk)
                if aw_hs:
                    s("awvalid").value = 0
                    aw = True
                if w_hs:
                    s("wvalid").value = 0
                    w = True
                if aw and w:
                    break
            else:
                raise TimeoutError(f"AXI-Lite write {addr:#x}: нет AWREADY/WREADY")
            s("bready").value = 1
            await self._until("bvalid", ("bresp",))
            s("bready").value = 0
            assert self._seen["bresp"] == 0, f"AXI-Lite write {addr:#x}: BRESP={self._seen['bresp']}"

    async def read(self, addr: int) -> int:
        async with self._lock:
            await RisingEdge(self.clk)
            s = self._s
            s("araddr").value = addr
            s("arvalid").value = 1
            await self._until("arready", ())
            s("arvalid").value = 0
            s("rready").value = 1
            await self._until("rvalid", ("rdata", "rresp"))
            s("rready").value = 0
            assert self._seen["rresp"] == 0, f"AXI-Lite read {addr:#x}: RRESP={self._seen['rresp']}"
            return self._seen["rdata"]
