"""Controllers built from transfer-function blocks (Controller Lab).

A design is a sum of linear blocks, one per measured signal:

    u(s) = K_e(s) E(s) + K_edot(s) E'(s) + K_dv(s) DV(s) + K_ff(s) U_ff(s)

with ``E = p_{i-1} - p_i - L - (r + h v_i)`` (spacing error), ``E' = sE``,
``DV = v_{i-1} - v_i`` and ``U_ff`` the predecessor's V2V value
(``ff_signal`` 'u' commanded or 'a' realized acceleration). Each block is a
rational expression in ``s`` written as text, e.g. ``0.2/(h*s + 1)``; the
names ``h r kp kd kv ka`` read the scenario's controller block and
``lag(T)``, ``lead(Tz, Tp)``, ``pi(kp, ki)``, ``lowpass2(wn, zeta)`` are
built in. The three built-in laws are exactly expressible this way
(:data:`PRESETS`), so a design starts from a known-good point.

Everything the Lab shows comes from the same blocks:

* :class:`TFController` realizes them as one state-space controller the
  simulator integrates (a regular plugin: ``cacc.tfctrl:TFController``);
* :func:`gamma` is the analytic string-stability transfer for a general
  block design, with the vehicle ``G(s) = e^{-theta_a s}/(s^2 (tau s + 1))``
  and V2V delay ``e^{-theta s}``::

      Gamma = G (K_E + s K_dv + K_ff e^{-theta s} phi)
              / (1 + G (K_E (1 + h s) + s K_dv)),   K_E = K_e + s K_edot

  with ``phi = s^2`` for 'a' and ``1/G`` for 'u' (reduces to the formulas in
  :mod:`cacc.analysis` for ACC, CACC and CTHP);
* :func:`closed_loop_poles` are the roots of the follower's loop (without
  dead times) plus the feedforward filter poles;
* :func:`contributions` splits a simulated command into what each block
  added, so saturation and feedforward effects are visible.
"""

from __future__ import annotations

import ast
from dataclasses import dataclass, field

import numpy as np
import yaml

SIGNALS = ("e", "e_dot", "dv", "u_ff")
SIGNAL_LABELS = {"e": "spacing error e", "e_dot": "error rate ė",
                 "dv": "relative speed Δv", "u_ff": "V2V feedforward u_ff"}
PARAM_NAMES = ("h", "r", "kp", "kd", "kv", "ka")
REF = "cacc.tfctrl:TFController"

#: name -> (description, ff_signal, h, r, blocks)
PRESETS: dict[str, tuple[str, str, float, float, dict[str, str]]] = {
    "cacc": ("CACC, Ploeg 2014 (PD + V2V through 1/(hs+1))", "u", 0.7, 2.5,
             {"e": "0.2/(h*s + 1)", "e_dot": "0.7/(h*s + 1)", "dv": "0",
              "u_ff": "1/(h*s + 1)"}),
    "cthp": ("CTHP, Ma 2025 case A (static, realized-accel V2V)", "a", 0.95, 2.5,
             {"e": "0.009", "e_dot": "0", "dv": "0.63", "u_ff": "0.5"}),
    "acc": ("ACC (radar-only PD)", "u", 1.4, 2.5,
            {"e": "0.2", "e_dot": "0.7", "dv": "0", "u_ff": "0"}),
    "lag-ff": ("CACC with a filtered feedforward (your starting point)", "a",
               0.9, 2.5, {"e": "0.15", "e_dot": "0", "dv": "0.6",
                          "u_ff": "0.8*lag(0.2)"}),
}


# ----------------------------------------------------------- rational algebra
@dataclass(frozen=True)
class Rational:
    """num(s)/den(s), coefficient arrays highest power first."""

    num: np.ndarray
    den: np.ndarray

    @staticmethod
    def const(c: float) -> "Rational":
        return Rational(np.array([float(c)]), np.array([1.0]))

    def _norm(self) -> "Rational":
        num, den = np.trim_zeros(self.num, "f"), np.trim_zeros(self.den, "f")
        if den.size == 0:
            raise ValueError("division by zero")
        if num.size == 0:
            return Rational.const(0.0)
        return Rational(num / den[0], den / den[0])

    def __add__(self, o):
        return Rational(np.polyadd(np.polymul(self.num, o.den),
                                   np.polymul(o.num, self.den)),
                        np.polymul(self.den, o.den))._norm()

    def __neg__(self):
        return Rational(-self.num, self.den)

    def __sub__(self, o):
        return self + (-o)

    def __mul__(self, o):
        return Rational(np.polymul(self.num, o.num),
                        np.polymul(self.den, o.den))._norm()

    def __truediv__(self, o):
        if not np.any(o.num):
            raise ValueError("division by zero")
        return Rational(np.polymul(self.num, o.den),
                        np.polymul(self.den, o.num))._norm()

    @property
    def is_zero(self) -> bool:
        return not np.any(self.num)

    @property
    def is_const(self) -> bool:
        return self.num.size == 1 and self.den.size == 1

    @property
    def order(self) -> int:
        return self.den.size - 1

    @property
    def proper(self) -> bool:
        return self.num.size <= self.den.size

    def __call__(self, s):
        return np.polyval(self.num, s) / np.polyval(self.den, s)


S = Rational(np.array([1.0, 0.0]), np.array([1.0]))


def _builtin(name: str, args: list[float]) -> Rational:
    c = Rational.const
    want = {"lag": 1, "lead": 2, "pi": 2, "lowpass2": 2}
    if name not in want:
        raise ValueError(f"unknown function {name}(); use one of "
                         + ", ".join(f"{k}()" for k in want))
    if len(args) != want[name]:
        raise ValueError(f"{name}() takes {want[name]} argument(s)")
    if name == "lag":
        return c(1) / (c(args[0]) * S + c(1))
    if name == "lead":
        return (c(args[0]) * S + c(1)) / (c(args[1]) * S + c(1))
    if name == "pi":
        return c(args[0]) + c(args[1]) / S
    wn, z = args
    return c(wn**2) / (S * S + c(2 * z * wn) * S + c(wn**2))


def parse(text: str, params: dict[str, float] | None = None) -> Rational:
    """Parse a rational expression in ``s`` (no ``eval``: a small AST walk)."""
    params = params or {}
    src = (text or "0").strip().replace("^", "**") or "0"
    try:
        tree = ast.parse(src, mode="eval")
    except SyntaxError as exc:
        raise ValueError(f"cannot read {text!r}: {exc.msg}") from None

    def walk(node) -> Rational:
        if isinstance(node, ast.Expression):
            return walk(node.body)
        if isinstance(node, ast.Constant) and isinstance(node.value, (int, float)):
            return Rational.const(node.value)
        if isinstance(node, ast.Name):
            if node.id == "s":
                return S
            if node.id in params:
                return Rational.const(params[node.id])
            raise ValueError(f"unknown name {node.id!r} in {text!r} (use s, "
                             + ", ".join(PARAM_NAMES) + ")")
        if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.USub, ast.UAdd)):
            v = walk(node.operand)
            return -v if isinstance(node.op, ast.USub) else v
        if isinstance(node, ast.BinOp):
            a, b = walk(node.left), walk(node.right)
            if isinstance(node.op, ast.Add):
                return a + b
            if isinstance(node.op, ast.Sub):
                return a - b
            if isinstance(node.op, ast.Mult):
                return a * b
            if isinstance(node.op, ast.Div):
                return a / b
            if isinstance(node.op, ast.Pow):
                if not b.is_const or float(b.num[0]) != int(b.num[0]) or b.num[0] < 0:
                    raise ValueError("powers must be whole numbers >= 0")
                out = Rational.const(1.0)
                for _ in range(int(b.num[0])):
                    out = out * a
                return out
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
            args = [walk(a) for a in node.args]
            if node.keywords or not all(a.is_const for a in args):
                raise ValueError(f"{node.func.id}() takes plain numbers")
            return _builtin(node.func.id, [float(a.num[0]) for a in args])
        raise ValueError(f"cannot read {text!r}: only numbers, s, + - * / ** "
                         "and lag/lead/pi/lowpass2 are allowed")

    return walk(tree)


# ----------------------------------------------------------------- design
@dataclass
class Design:
    """A Lab controller: blocks per signal + spacing policy + V2V payload."""

    blocks: dict[str, str] = field(default_factory=lambda: dict(PRESETS["cacc"][4]))
    ff_signal: str = "u"
    h: float = 0.7
    r: float = 2.5
    name: str = "my-controller"

    def __post_init__(self):
        if self.ff_signal not in ("a", "u"):
            raise ValueError("ff_signal must be 'a' or 'u'")
        unknown = set(self.blocks) - set(SIGNALS)
        if unknown:
            raise ValueError(f"unknown signal(s) {sorted(unknown)}; use {SIGNALS}")
        if self.h < 0 or self.r < 0:
            raise ValueError("h and r must be >= 0")

    @classmethod
    def preset(cls, key: str) -> "Design":
        _, ff, h, r, blocks = PRESETS[key]
        return cls(dict(blocks), ff, h, r, key)

    def tfs(self, extra: dict[str, float] | None = None) -> dict[str, Rational]:
        """Parsed, validated blocks (all four signals present)."""
        params = {"h": self.h, "r": self.r, **(extra or {})}
        out = {}
        for sig in SIGNALS:
            tf = parse(self.blocks.get(sig, "0"), params)
            if not tf.proper:
                raise ValueError(f"block {sig}: more zeros than poles (improper)"
                                 + ("; use the e_dot block instead of s*e"
                                    if sig == "e" else ""))
            out[sig] = tf
        return out

    @property
    def options(self) -> dict:
        """Plugin options for ``cacc.tfctrl:TFController``."""
        return {"blocks": {k: str(v) for k, v in self.blocks.items()},
                "ff_signal": self.ff_signal}

    @property
    def controller_block(self) -> dict:
        """Scenario ``controller:`` entries this design needs (spacing policy)."""
        return {"h": self.h, "r": self.r}

    def to_dict(self) -> dict:
        return {"name": self.name, "controller": REF, "ff_signal": self.ff_signal,
                "spacing": {"h": self.h, "r": self.r},
                "blocks": {k: str(self.blocks.get(k, "0")) for k in SIGNALS}}

    @classmethod
    def from_dict(cls, d: dict) -> "Design":
        sp = d.get("spacing") or {}
        return cls(dict(d.get("blocks") or {}), d.get("ff_signal", "u"),
                   float(sp.get("h", 0.7)), float(sp.get("r", 2.5)),
                   d.get("name", "my-controller"))

    def save(self, path) -> None:
        from pathlib import Path

        Path(path).write_text(yaml.safe_dump(self.to_dict(), sort_keys=False),
                              encoding="utf-8")

    @classmethod
    def load(cls, path) -> "Design":
        from pathlib import Path

        return cls.from_dict(yaml.safe_load(Path(path).read_text(encoding="utf-8")))


def is_design_file(ref: str) -> bool:
    return ref.lower().endswith((".yaml", ".yml")) and ":" not in ref.replace(
        "\\", "/").split("/")[-1]


# --------------------------------------------------------------- controller
class TFController:
    """The blocks realized in state space; a :mod:`cacc.plugins` controller."""

    def __init__(self, params, blocks: dict[str, str] | None = None,
                 ff_signal: str = "u"):
        from scipy.signal import tf2ss

        design = Design(dict(blocks or PRESETS["cacc"][4]), ff_signal,
                        params.h, params.r)
        tfs = design.tfs({"kp": params.kp, "kd": params.kd, "kv": params.kv,
                          "ka": params.ka})
        self.ff_signal = ff_signal
        self.uses_v2v = not tfs["u_ff"].is_zero
        self.parts = []  # (signal index, A, B, C, D, state slice)
        k = 0
        for j, sig in enumerate(SIGNALS):
            tf = tfs[sig]
            if tf.is_zero:
                continue
            if tf.is_const:  # a plain gain: no state
                A, B, C, D = (np.zeros((0, 0)), np.zeros((0, 1)), np.zeros((1, 0)),
                              np.array([[float(tf.num[0])]]))
            else:
                A, B, C, D = tf2ss(tf.num, tf.den)
            n = A.shape[0]
            self.parts.append((j, A, B[:, 0], C[0], float(D[0, 0]), slice(k, k + n)))
            k += n
        self.n_states = k

    def output(self, xc, e, e_dot, u_ff, dv=0.0):
        sig = (e, e_dot, dv, u_ff)
        return float(sum(C @ xc[sl] + D * sig[j] for j, _, _, C, D, sl in self.parts))

    def deriv(self, xc, e, e_dot, u_ff, dv=0.0):
        sig = (e, e_dot, dv, u_ff)
        dx = np.zeros(self.n_states)
        for j, A, B, _, _, sl in self.parts:
            dx[sl] = A @ xc[sl] + B * sig[j]
        return dx


# ----------------------------------------------------------------- analysis
def gamma(omega, design: Design, tau: float, theta: float = 0.0,
          act_delay: float = 0.0) -> np.ndarray:
    """Analytic string-stability transfer Gamma(j omega) of a block design."""
    s = 1j * np.asarray(omega, dtype=float)
    k = {sig: tf(s) for sig, tf in design.tfs().items()}
    G = np.exp(-act_delay * s) / (s**2 * (tau * s + 1.0))
    KE = k["e"] + s * k["e_dot"]
    phi = s**2 if design.ff_signal == "a" else 1.0 / G
    num = G * (KE + s * k["dv"] + k["u_ff"] * np.exp(-theta * s) * phi)
    return num / (1.0 + G * (KE * (1.0 + design.h * s) + s * k["dv"]))


def closed_loop_poles(design: Design, tau: float) -> np.ndarray:
    """Poles of one follower's loop (dead times ignored) and of the V2V
    feedforward filter; all must lie in the open left half-plane."""
    t = design.tfs()
    one = Rational.const(1.0)
    L = t["e"] * (one + Rational.const(design.h) * S) + S * (t["e_dot"] * (
        one + Rational.const(design.h) * S) + t["dv"])
    plant_den = np.polymul([1.0, 0.0, 0.0], [tau, 1.0])
    char = np.polyadd(np.polymul(plant_den, L.den), L.num)
    poles = [np.roots(np.trim_zeros(char, "f"))]
    if not t["u_ff"].is_zero and t["u_ff"].order:
        poles.append(np.roots(t["u_ff"].den))
    return np.concatenate(poles)


@dataclass
class Report:
    """What the Lab prints next to the plots."""

    peak_gain: float
    peak_omega: float
    string_stable: bool
    internally_stable: bool
    n_states: int
    poles: np.ndarray


def analyse(design: Design, tau: float, theta: float = 0.0,
            act_delay: float = 0.0, omega=None) -> tuple[np.ndarray, np.ndarray, Report]:
    """(omega, |Gamma|, report) for the Lab's plots."""
    from scipy.signal import tf2ss  # noqa: F401  (fail early if scipy is broken)

    w = np.logspace(-3, 2, 3000) if omega is None else np.asarray(omega)
    g = np.abs(gamma(w, design, tau, theta, act_delay))
    poles = closed_loop_poles(design, tau)
    stable = bool(np.all(poles.real < -1e-9))
    k = int(np.argmax(g))
    n = sum(tf.order for tf in design.tfs().values() if not tf.is_zero)
    return w, g, Report(float(g[k]), float(w[k]), bool(g[k] <= 1.0 + 1e-6),
                        stable, n, poles)


def contributions(res, design: Design, follower: int = 1) -> dict[str, np.ndarray]:
    """Re-run each block on the signals follower ``follower`` measured in
    ``res``: per-block command, their sum and the applied (saturated) ``u``.

    The V2V input is the ideal delayed predecessor signal (link noise, loss
    and attacks are not replayed), so with a clean link the sum matches the
    simulator to integration accuracy and any gap to ``u`` is saturation.
    """
    from scipy.signal import lsim

    i = int(follower)
    n = res.vel.shape[1] - 1
    if not 1 <= i <= n:
        raise ValueError(f"follower must be 1..{n}")
    cfg = res.config
    v_prev, v, a = res.vel[:, i - 1], res.vel[:, i], res.acc[:, i]
    e = res.err[:, i - 1]
    sig = {"e": e, "e_dot": v_prev - v - design.h * a, "dv": v_prev - v}
    if design.ff_signal == "a" or i == 1:
        ff = res.acc[:, i - 1]
    else:
        ff = res.u[:, i - 2]
    lag = int(round(cfg.delay / cfg.dt))
    sig["u_ff"] = np.concatenate([np.full(lag, ff[0]), ff[:ff.size - lag]]) if lag else ff
    out = {}
    for name, tf in design.tfs().items():
        if tf.is_zero:
            continue
        if tf.is_const:
            out[name] = float(tf.num[0]) * sig[name]
        else:
            _, y, _ = lsim((tf.num, tf.den), sig[name], res.t)
            out[name] = np.asarray(y)
    out["sum"] = sum(out.values()) if out else np.zeros_like(res.t)
    out["u"] = res.u[:, i - 1]
    return out


def to_matlab(design: Design, tau: float, theta: float = 0.0) -> str:
    """A MATLAB script that rebuilds the design with Control System Toolbox
    ``tf`` objects and plots |Gamma| (drop the blocks into Simulink as
    Transfer Fcn blocks)."""
    t = design.tfs()

    def vec(a):
        return "[" + " ".join(f"{float(x):.10g}" for x in a) + "]"

    lines = [f"% {design.name}: exported from CACC Studio Controller Lab",
             f"h = {design.h:g}; r = {design.r:g}; tau = {tau:g}; theta = {theta:g};",
             "s = tf('s');"]
    for sig in SIGNALS:
        lines.append(f"K_{sig} = tf({vec(t[sig].num)}, {vec(t[sig].den)});"
                     f"  % {SIGNAL_LABELS[sig]}: {design.blocks.get(sig, '0')}")
    phi = "s^2" if design.ff_signal == "a" else "1/G"
    lines += ["G = 1/(s^2*(tau*s + 1));",
              "KE = K_e + s*K_e_dot;",
              f"Gamma = G*(KE + s*K_dv + K_u_ff*exp(-theta*s)*{phi}) / "
              "(1 + G*(KE*(1 + h*s) + s*K_dv));",
              "Gamma = minreal(Gamma);",
              "w = logspace(-3, 2, 2000);",
              "[mag, ~] = bode(Gamma, w);",
              "semilogx(w, squeeze(mag)); yline(1, '--'); grid on;",
              "xlabel('\\omega [rad/s]'); ylabel('|\\Gamma(j\\omega)|');",
              f"title('{design.name}: string stability');"]
    return "\n".join(lines) + "\n"


def resolve(controller: str, options: dict | None = None):
    """``(controller ref, options, controller block)`` for a controller
    argument that may be a Controller Lab design file (``*.yaml``)."""
    if is_design_file(controller):
        d = Design.load(controller)
        d.tfs()  # fail early with the block that does not parse
        return REF, d.options, d.controller_block
    return controller, dict(options or {}), None
