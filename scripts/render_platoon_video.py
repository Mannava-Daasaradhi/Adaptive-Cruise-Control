"""Render a platoon run as a top-down highway animation (MP4/GIF).

Input is either a ROS 2 recorder CSV (``results/ros2/*.csv``) or a scenario
YAML (``scenarios/*.yaml`` — the offline core is simulated first, so this
works without ROS). Cars are drawn to physical scale, colored by spacing
error (green = tight, red = |e| >= 1 m); the camera tracks the platoon while
world-fixed lane dashes slide past; speed and spacing-error traces run
below the road with a moving time cursor. See docs/decisions/D-015.

Usage (Windows, `cacc` conda env):

    python scripts/render_platoon_video.py results/ros2/<stamp>_cthp_n6.csv
    python scripts/render_platoon_video.py scenarios/ma2025_sine.yaml
    python scripts/render_platoon_video.py <input> --frames 6   # PNG check
"""

from __future__ import annotations

import argparse
import logging
import shutil
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.animation as manim
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyBboxPatch

from cacc import setup_logging

log = logging.getLogger("cacc.scripts.render_platoon_video")

CAR_L, CAR_W = 4.0, 1.9
LANE_HALF = 3.6
ERR_SAT = 1.0
VIEW_BACK, VIEW_AHEAD = 18.0, 30.0  # camera margins [m]


def error_color(e: float) -> tuple[float, float, float]:
    s = min(abs(e) / ERR_SAT, 1.0)
    return 0.15 + 0.75 * s, 0.75 - 0.65 * s, 0.15


# --------------------------------------------------------------------- input
def load_csv(path: Path) -> dict[int, dict[str, np.ndarray]]:
    raw = np.genfromtxt(path, delimiter=",", names=True)
    out = {}
    for idx in np.unique(raw["index"]).astype(int):
        r = raw[raw["index"] == idx]
        o = np.argsort(r["t"])
        out[idx] = {"t": r["t"][o], "p": r["position"][o],
                    "v": r["velocity"][o], "e": r["spacing_error"][o]}
    # trim to the co-observed interval (SIGINT teardown skew)
    t_cut = min(d["t"].max() for d in out.values())
    for d in out.values():
        m = d["t"] <= t_cut
        for k in ("t", "p", "v", "e"):
            d[k] = d[k][m]
    return out


def simulate_scenario(path: Path, controller: str) -> dict[int, dict[str, np.ndarray]]:
    from cacc.platoon import PlatoonSim, load_scenario
    sc = load_scenario(path)
    res = PlatoonSim(sc.config, controller, sc.leader).run()
    out = {0: {"t": res.t, "p": res.pos[:, 0], "v": res.vel[:, 0],
               "e": np.zeros_like(res.t)}}
    for i in range(1, res.pos.shape[1]):
        out[i] = {"t": res.t, "p": res.pos[:, i], "v": res.vel[:, i],
                  "e": res.err[:, i - 1]}
    return out


def at(d: dict[str, np.ndarray], key: str, t: float) -> float:
    return float(np.interp(t, d["t"], d[key]))


# ------------------------------------------------------------------- figure
def build_figure(runs, t0):
    ids = sorted(runs)
    fig = plt.figure(figsize=(12.8, 7.2), facecolor="#16181f")
    gs = fig.add_gridspec(3, 1, height_ratios=[2.2, 1, 1], hspace=0.35,
                          left=0.07, right=0.98, top=0.92, bottom=0.08)
    ax_road = fig.add_subplot(gs[0])
    ax_v = fig.add_subplot(gs[1])
    ax_e = fig.add_subplot(gs[2])

    ax_road.set_facecolor("#2e2e33")
    ax_road.set_ylim(-LANE_HALF - 2.0, LANE_HALF + 2.6)
    ax_road.set_yticks([])
    ax_road.tick_params(colors="w", labelsize=8)
    for s in ax_road.spines.values():
        s.set_color("w")
    ax_road.set_xlabel("position [m]", color="w", fontsize=8)
    for y in (LANE_HALF, -LANE_HALF):
        ax_road.axhline(y, color="w", lw=2)
    dashes, = ax_road.plot([], [], color="w", lw=2, ls=(0, (4, 6)))
    dashes.set_data([], [])

    cars, labels = {}, {}
    for i in ids:
        box = FancyBboxPatch((0, -CAR_W / 2), CAR_L, CAR_W,
                             boxstyle="round,pad=0.02,rounding_size=0.5",
                             ec="k", fc="w", zorder=3)
        ax_road.add_patch(box)
        cars[i] = box
        labels[i] = ax_road.text(0, CAR_W / 2 + 0.7, "", color="w",
                                 ha="center", fontsize=8, zorder=4,
                                 clip_on=True)
    hud = ax_road.set_title("", color="w", fontsize=11, loc="left")

    for ax, ylab in ((ax_v, "v [m/s]"), (ax_e, r"$\delta_i$ [m]")):
        ax.set_facecolor("#1c1e26")
        ax.tick_params(colors="w", labelsize=8)
        for s in ax.spines.values():
            s.set_color("w")
        ax.set_ylabel(ylab, color="w", fontsize=9)
        ax.grid(alpha=0.25)
    ax_e.set_xlabel("t [s]", color="w", fontsize=9)

    colors = plt.cm.viridis(np.linspace(0.1, 0.95, len(ids) - 1))
    ax_v.plot(runs[0]["t"], runs[0]["v"], "w--", lw=1.2, label="leader")
    for j, i in enumerate(ids[1:]):
        ax_v.plot(runs[i]["t"], runs[i]["v"], color=colors[j], lw=0.9)
        ax_e.plot(runs[i]["t"], -runs[i]["e"], color=colors[j], lw=0.9)
    ax_v.legend(fontsize=7, loc="upper right", framealpha=0.2,
                labelcolor="w")
    cur_v = ax_v.axvline(t0, color="w", lw=1)
    cur_e = ax_e.axvline(t0, color="w", lw=1)

    return fig, ax_road, dashes, cars, labels, hud, cur_v, cur_e


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="recorder CSV or scenario YAML")
    parser.add_argument("--controller", default="cthp",
                        help="controller for scenario input (default cthp)")
    parser.add_argument("--speedup", type=float, default=10.0,
                        help="time-lapse factor (default 10x)")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--out", default=None,
                        help="output path (default results/videos/<stem>)")
    parser.add_argument("--frames", type=int, default=0,
                        help="dump N evenly spaced PNG frames instead of video")
    args = parser.parse_args()
    setup_logging()

    src = Path(args.input)
    if src.suffix.lower() in (".yaml", ".yml"):
        runs = simulate_scenario(src, args.controller)
    else:
        runs = load_csv(src)
    ids = sorted(runs)
    t0 = max(d["t"].min() for d in runs.values())
    t1 = min(d["t"].max() for d in runs.values())

    fig, ax_road, dashes, cars, labels, hud, cur_v, cur_e = build_figure(runs, t0)

    def draw(t: float) -> None:
        lead_x = at(runs[0], "p", t)
        tail_x = at(runs[ids[-1]], "p", t)
        x_lo, x_hi = tail_x - VIEW_BACK, lead_x + VIEW_AHEAD
        ax_road.set_xlim(x_lo, x_hi)
        grid = np.arange(np.floor(x_lo / 10) * 10, x_hi + 10, 10.0)
        xs = np.ravel(np.column_stack([grid, grid + 4, np.full_like(grid, np.nan)]))
        ys = np.zeros_like(xs)
        dashes.set_data(xs, ys)
        for i in ids:
            p = at(runs[i], "p", t)
            v = at(runs[i], "v", t)
            e = at(runs[i], "e", t)
            cars[i].set_x(p - CAR_L)  # p = front bumper
            fc = (0.95, 0.95, 0.95) if i == 0 else error_color(e)
            cars[i].set_facecolor(fc)
            labels[i].set_position((p - CAR_L / 2, CAR_W / 2 + 0.9))
            labels[i].set_text("LEAD" if i == 0 else f"v{i}")
        hud.set_text(
            f"t = {t:6.1f} s    leader {at(runs[0], 'v', t):5.1f} m/s    "
            f"CACC platoon, case A (Ma 2025)")
        cur_v.set_xdata([t, t])
        cur_e.set_xdata([t, t])

    if args.frames:
        outdir = Path(args.out) if args.out else src.parent
        for k, t in enumerate(np.linspace(t0, t1, args.frames)):
            draw(float(t))
            fp = outdir / f"{src.stem}_frame{k}.png"
            fig.savefig(fp, dpi=110, facecolor=fig.get_facecolor())
            log.info("frame %d (t=%.1f s) -> %s", k, t, fp)
        return

    times = np.arange(t0, t1, args.speedup / args.fps)
    have_ffmpeg = shutil.which("ffmpeg") is not None
    outdir = Path("results/videos")
    outdir.mkdir(parents=True, exist_ok=True)
    if args.out:
        out = Path(args.out)
    else:
        out = outdir / (src.stem + (".mp4" if have_ffmpeg else ".gif"))

    anim = manim.FuncAnimation(fig, lambda t: draw(float(t)), frames=times)
    if have_ffmpeg and out.suffix == ".mp4":
        writer = manim.FFMpegWriter(fps=args.fps, bitrate=2500)
    else:
        out = out.with_suffix(".gif")
        writer = manim.PillowWriter(fps=min(args.fps, 20))
        log.warning("ffmpeg not found - writing GIF (slower, larger)")
    log.info("rendering %d frames (%.0fx time-lapse) -> %s",
             len(times), args.speedup, out)
    anim.save(out, writer=writer,
              savefig_kwargs={"facecolor": fig.get_facecolor()})
    log.info("done: %s", out)


if __name__ == "__main__":
    main()
