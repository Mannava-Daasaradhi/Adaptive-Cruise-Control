"""Generate the synthetic OpenACC-format sample logs shipped in examples/data.

Three production-like ACC tunings behind a human-driven leader who performs
speed perturbations (the OpenACC test style), 10 Hz, GNSS-like noise. The
middle follower hands over to its driver for 40 s (``Driver2 = Human``) so the
engagement mask is exercised. Ground truth lives in the file names and in
tests/test_twin.py; regenerate with:

    python scripts/make_synthetic_openacc.py
"""

from pathlib import Path

import numpy as np

from cacc.fielddata import write_openacc
from cacc.twin import LinearACC, synthesize_log

TRUTH = {
    "SedanA(T1.2-sluggish)": LinearACC(k_s=0.06, k_v=0.25, T=1.2, s0=3.0, tau=0.9),
    "SUVB(T1.5)": LinearACC(k_s=0.10, k_v=0.30, T=1.5, s0=4.0, tau=0.7),
    "EVC(T2.2-smooth)": LinearACC(k_s=0.08, k_v=0.55, T=2.2, s0=5.0, tau=0.5),
}


def leader_speed(t: np.ndarray) -> np.ndarray:
    """Cruise at 25 m/s with four smooth perturbations (-4, +3, -6, +2 m/s)."""
    v = np.full_like(t, 25.0)
    for t0, dur, dv in ((30, 12, -4.0), (110, 15, 3.0), (190, 10, -6.0),
                        (260, 20, 2.0)):
        ramp = np.clip((t - t0) / dur, 0.0, 1.0)
        back = np.clip((t - t0 - dur - 20) / dur, 0.0, 1.0)
        v += dv * (0.5 - 0.5 * np.cos(np.pi * ramp)) \
            - dv * (0.5 - 0.5 * np.cos(np.pi * back))
    return v


def main() -> None:
    t = np.round(np.arange(0.0, 340.0, 0.1), 2)
    names = ["Lead(Human)"] + list(TRUTH)
    engaged = np.ones((t.size, 4), bool)
    engaged[:, 0] = False
    engaged[(t >= 150) & (t < 190), 2] = False  # SUV-B driver takeover
    log = synthesize_log(list(TRUTH.values()), t, leader_speed(t), names,
                         seed=7, engaged=engaged)
    out = Path(__file__).parents[1] / "examples" / "data"
    out.mkdir(parents=True, exist_ok=True)
    write_openacc(log, out / "synthetic_openacc_platoon.csv")
    print(f"wrote {out / 'synthetic_openacc_platoon.csv'}")


if __name__ == "__main__":
    main()
