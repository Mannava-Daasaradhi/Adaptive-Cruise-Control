"""Build the presentation's simulation assets from a live run of the platoon.

The slide deck shows the simulation four ways (Simulink block diagram, live
browser demo, animation filmstrip, cross-backend numbers). Three of those are
produced elsewhere; this script produces the filmstrip, so the deck stays
rebuildable from source rather than depending on hand-cropped screenshots.

    python scripts/make_sim_assets.py

Writes results/frames/:
    ma2025_sine_frame{0..3}.png   raw light-theme frames (via render_platoon_video)
    platoon_filmstrip.png         the four road bands stacked into one image

Prerequisite: the light-theme frames. This script renders them itself.
"""

from __future__ import annotations

import logging
import subprocess
import sys
from pathlib import Path

from PIL import Image

from cacc import setup_logging

log = logging.getLogger("cacc.scripts.make_sim_assets")

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "results" / "frames"

# The renderer's road panel occupies a fixed fraction of the 12.8x7.2in figure:
# title band through the "position [m]" axis label. Cropping to it turns four
# full frames into a compact strip that reads as an animation on one slide.
CROP_TOP, CROP_BOT = 0.038, 0.500
N_FRAMES = 4


def render_frames() -> list[Path]:
    cmd = [sys.executable, str(ROOT / "scripts" / "render_platoon_video.py"),
           str(ROOT / "scenarios" / "ma2025_sine.yaml"),
           "--controller", "cthp", "--frames", str(N_FRAMES),
           "--light", "--out", str(OUT)]
    OUT.mkdir(parents=True, exist_ok=True)
    subprocess.run(cmd, check=True, cwd=ROOT)
    return [OUT / f"ma2025_sine_frame{k}.png" for k in range(N_FRAMES)]


def filmstrip(frames: list[Path], dest: Path) -> Path:
    bands = []
    for fp in frames:
        im = Image.open(fp).convert("RGB")
        w, h = im.size
        bands.append(im.crop((0, int(h * CROP_TOP), w, int(h * CROP_BOT))))
    w = bands[0].width
    gap = 10
    total = sum(b.height for b in bands) + gap * (len(bands) - 1)
    strip = Image.new("RGB", (w, total), (255, 255, 255))
    y = 0
    for b in bands:
        strip.paste(b, (0, y))
        y += b.height + gap
    strip.save(dest)
    log.info("filmstrip %dx%d -> %s", strip.width, strip.height, dest)
    return dest


def main() -> int:
    setup_logging()
    frames = render_frames()
    missing = [f for f in frames if not f.exists()]
    if missing:
        log.error("renderer produced no frames: %s", missing)
        return 1
    filmstrip(frames, OUT / "platoon_filmstrip.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
