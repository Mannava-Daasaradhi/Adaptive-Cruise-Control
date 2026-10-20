"""Hardware-in-the-loop: the simulated platoon drives a controller on real
hardware (Raspberry Pi, Arduino, an ECU on a CAN bus).

The controller under test is :class:`HILController`, a plugin like any
other, so every scenario, plan and campaign can run against hardware::

    cacc hil --udp 192.168.1.20:5005 --maneuver hard-brake --realtime
    cacc stress -c cacc.adapters.hil:HILController --option address=192.168.1.20:5005

Every ``step`` seconds of simulated time it sends one car's measurements to
the device and holds the reply as the command until the next sample
(zero-order hold, as an ECU task would). ``cars`` picks which followers
are hardware; the others run the device's reference law in software
(:class:`cacc.adapters.hil_device.DeviceLaw`), so a single board can sit at
any place in the string. With ``realtime`` the simulation is paced to the
wall clock, so the device sees the real timing; every round trip is timed
and a reply later than ``timeout`` counts as a miss (the command is held,
or the car brakes at ``fail_brake`` with ``on_timeout='brake'``).

Transports: ``udp`` (address ``host:port``), ``serial`` (address = port,
needs pyserial), ``can`` (address = channel, needs python-can; use with
``realtime``: the CAN frames carry no time stamp) and ``loopback`` (the
reference law in-process through the full encode/decode path, for tests
and for the software baseline).
"""

from __future__ import annotations

import socket
import time

import numpy as np

from cacc.adapters import hil_device as dev

#: live controllers, for :func:`stats` after a run
_REGISTRY: list["HILController"] = []
_PACE: dict = {}


class _Loopback:
    def __init__(self, law):
        self.law = law

    def request(self, car, seq, m, timeout):
        return dev.unpack_reply(dev.handle(self.law, dev.pack_request(car, seq, m)))[2]


class _UDP:
    def __init__(self, address: str):
        host, port = address.rsplit(":", 1)
        self.peer = (host, int(port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def request(self, car, seq, m, timeout):
        self.sock.settimeout(timeout)
        self.sock.sendto(dev.pack_request(car, seq, m), self.peer)
        end = time.perf_counter() + timeout
        while True:
            try:
                data = self.sock.recv(64)
            except (socket.timeout, ConnectionResetError):
                return None
            c, s, u = dev.unpack_reply(data)
            if c == car and s == seq & 0xFFFFFFFF:
                return u
            left = end - time.perf_counter()  # a stale reply: keep waiting
            if left <= 0:
                return None
            self.sock.settimeout(left)


class _Serial:
    def __init__(self, port: str, baud: int = 115200):
        try:
            import serial
        except ImportError:
            raise ImportError("serial HIL needs pyserial: pip install pyserial") from None
        self.ser = serial.Serial(port, baud, timeout=0.1)

    def request(self, car, seq, m, timeout):
        body = dev.pack_request(car, seq, m)
        self.ser.timeout = timeout
        self.ser.write(bytes([dev.SER_REQ]) + body + bytes([dev.checksum(body)]))
        if self.ser.read(1) != bytes([dev.SER_REP]):
            return None
        rep = self.ser.read(dev.REP.size + 1)
        if len(rep) != dev.REP.size + 1 or dev.checksum(rep[:-1]) != rep[-1]:
            return None
        return dev.unpack_reply(rep[:-1])[2]


class _CAN:
    def __init__(self, channel: str, interface: str = "socketcan"):
        try:
            import can
        except ImportError:
            raise ImportError("CAN HIL needs python-can: pip install python-can") from None
        self.can = can
        self.bus = can.Bus(channel=channel, interface=interface)

    def request(self, car, seq, m, timeout):
        for aid, data in dev.can_request(car, seq, m):
            self.bus.send(self.can.Message(arbitration_id=aid, data=data,
                                           is_extended_id=False))
        end = time.perf_counter() + timeout
        while (left := end - time.perf_counter()) > 0:
            msg = self.bus.recv(left)
            if msg is not None and msg.arbitration_id == dev.CAN_REPLY + car:
                u_mm, s = __import__("struct").unpack("<hH", bytes(msg.data))
                if s == seq & 0xFFFF:
                    return u_mm / 1000.0
        return None


def make_transport(kind: str, address: str, law=None, baud: int = 115200,
                   can_interface: str = "socketcan"):
    if kind == "loopback":
        return _Loopback(law)
    if kind == "udp":
        return _UDP(address)
    if kind == "serial":
        return _Serial(address, baud)
    if kind == "can":
        return _CAN(address, can_interface)
    raise ValueError(f"unknown HIL transport {kind!r}; use udp, serial, can or loopback")


class HILController:
    """Plugin: one follower's controller on hardware (module docstring)."""

    n_states = 0
    raw_inputs = True
    uses_v2v = True
    ff_signal = "u"

    def __init__(self, params, transport: str = "udp", address: str = "127.0.0.1:5005",
                 cars=None, step: float = 0.02, timeout: float = 0.05,
                 on_timeout: str = "hold", fail_brake: float = -3.0,
                 realtime: bool = False, law: str = "cacc", baud: int = 115200,
                 can_interface: str = "socketcan"):
        if step <= 0 or timeout <= 0:
            raise ValueError("HIL step and timeout must be > 0")
        if on_timeout not in ("hold", "brake"):
            raise ValueError("on_timeout must be 'hold' or 'brake'")
        self.r, self.h = params.r, params.h
        self.law = dev.DeviceLaw(law, params.kp, params.kd, params.h)
        self.uses_v2v = law == "cacc"
        self.kind, self.address = transport, address
        self.cars = None if cars is None else {int(c) for c in cars}
        self.step, self.timeout = float(step), float(timeout)
        self.on_timeout, self.fail_brake = on_timeout, float(fail_brake)
        self.realtime = bool(realtime)
        self._link_args = (baud, can_interface)
        self.car = 1
        self.link = None
        self._t_next, self._u, self._seq = 0.0, 0.0, 0
        self.rtt: list[float] = []
        self.misses = 0
        _REGISTRY.append(self)

    def bind_car(self, car: int) -> None:
        """Called by the simulator with this follower's number (1-based)."""
        self.car = car
        if self.hardware and self.link is None:
            self.link = make_transport(self.kind, self.address, self.law, *self._link_args)

    @property
    def hardware(self) -> bool:
        return self.cars is None or self.car in self.cars

    def equilibrium_gap(self, v):
        return self.r + self.h * v

    def _pace(self, t: float) -> None:
        if not self.realtime:
            return
        if t == 0.0 or "wall0" not in _PACE:
            _PACE["wall0"] = time.perf_counter() - t
        lag = _PACE["wall0"] + t - time.perf_counter()
        if lag > 0:
            time.sleep(lag)

    def output(self, xc, e, e_dot, u_ff, dv=0.0, *, gap, v, a, t):
        if t < self._t_next - 1e-9:
            return self._u
        self._t_next = t + self.step
        m = {"t": t, "e": e, "e_dot": e_dot, "dv": dv, "u_ff": u_ff,
             "gap": gap, "v": v, "a": a}
        if not self.hardware:
            self._u = self.law.step(self.car, m)
            return self._u
        if self.car == min(self.cars or {1}):
            self._pace(t)
        self._seq += 1
        t0 = time.perf_counter()
        u = self.link.request(self.car, self._seq, m, self.timeout)
        self.rtt.append(time.perf_counter() - t0)
        if u is None or not np.isfinite(u):
            self.misses += 1
            if self.on_timeout == "brake":
                self._u = self.fail_brake
        else:
            self._u = float(u)
        return self._u

    def deriv(self, xc, e, e_dot, u_ff, dv=0.0, **raw):
        return np.empty(0)


def reset() -> None:
    _REGISTRY.clear()
    _PACE.clear()


def stats(step: float | None = None) -> list[dict]:
    """Round-trip statistics of every hardware car since :func:`reset`."""
    out = []
    for c in _REGISTRY:
        if not c.hardware or not c.rtt:
            continue
        r = np.array(c.rtt) * 1e3
        out.append({"car": c.car, "samples": int(r.size), "misses": c.misses,
                    "rtt_ms_p50": float(np.percentile(r, 50)),
                    "rtt_ms_p99": float(np.percentile(r, 99)),
                    "rtt_ms_max": float(r.max()),
                    "late": int((r > 1e3 * (step or c.step)).sum())})
    return out


def run(maneuver: str = "hard-brake", condition: str = "nominal",
        transport: str = "udp", address: str = "127.0.0.1:5005", cars=None,
        step: float = 0.02, timeout: float = 0.05, realtime: bool = False,
        law: str = "cacc", spawn_device: bool = False, seed: int = 1) -> dict:
    """One campaign cell with hardware in the loop, next to the same law in
    pure software: how far the hardware run drifts, and the timing."""
    import threading

    import yaml

    from cacc import campaign as cp
    from cacc.criteria import METRICS

    stop = threading.Event()
    if spawn_device:  # a device on this machine, for a demo without hardware
        got = {}
        ready = threading.Event()
        th = threading.Thread(target=dev.serve_udp, daemon=True, args=(
            "127.0.0.1:0", dev.DeviceLaw(law)), kwargs={
            "stop": stop, "ready": lambda a: (got.update(a=a), ready.set())})
        th.start()
        ready.wait(5.0)
        transport, address = "udp", f"127.0.0.1:{got['a'][1]}"
    base = yaml.safe_load(cp.BASE.read_text(encoding="utf-8"))
    tree = cp.compose(base, maneuver, condition, "cacc")
    case = {"scenario": tree, "name": f"{maneuver} | {condition}"}
    ref = "cacc.adapters.hil:HILController"
    opts = {"step": step, "timeout": timeout, "law": law,
            "cars": None if cars is None else list(cars)}
    try:
        reset()
        hw = cp.replay(case, ref, seed, {**opts, "transport": transport,
                                         "address": address, "realtime": realtime})
        timing = stats(step)
        reset()
        sw = cp.replay(case, ref, seed, {**opts, "transport": "loopback"})
    finally:
        stop.set()
        reset()
    mg = METRICS["min_gap"].fn
    return {"cell": case["name"], "transport": transport, "address": address,
            "step": step, "realtime": realtime, "timing": timing,
            "min_gap_hw": mg(hw), "min_gap_sw": mg(sw),
            "max_du": float(np.abs(hw.u - sw.u).max()),
            "max_dgap": float(np.abs(np.diff(hw.pos, axis=1) - np.diff(sw.pos, axis=1)).max()),
            "hw": hw, "sw": sw}


def format_text(r: dict) -> str:
    lines = [f"HIL {r['cell']} over {r['transport']} {r['address']} "
             f"(sample {r['step'] * 1e3:.0f} ms{', real time' if r['realtime'] else ''})",
             f"  min gap: hardware {r['min_gap_hw']:.2f} m · software "
             f"{r['min_gap_sw']:.2f} m · largest command gap {r['max_du']:.3f} m/s² "
             f"· largest position gap {r['max_dgap']:.3f} m"]
    for s in r["timing"]:
        lines.append(f"  car {s['car']}: {s['samples']} samples · round trip "
                     f"p50 {s['rtt_ms_p50']:.2f} ms, p99 {s['rtt_ms_p99']:.2f} ms, "
                     f"max {s['rtt_ms_max']:.2f} ms · {s['late']} late · "
                     f"{s['misses']} missed")
    if not r["timing"]:
        lines.append("  no hardware car answered")
    return "\n".join(lines)
