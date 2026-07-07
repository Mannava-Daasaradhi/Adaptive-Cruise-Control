# D-006 — Paper parameters extracted from the PDF (HTML was truncated)

**Date:** 2026-07-05 · **Status:** accepted · **Type:** data provenance

## Context
Reproducing Section IV of Ma 2025 requires the exact simulation parameters.
The arXiv HTML rendering truncated the parameter tables, and this machine
has no poppler (`pdftoppm`), so the PDF could not be read page-by-page as
images by tooling.

## Decision
Install `pypdf` into the conda env and extract the full text of the local
preprint (`references/Ma2025_TITS_TimeHeadway_NoisyV2V_arXiv-2404.08889.pdf`)
to a UTF-8 file, then transcribe Section IV. (First attempt printed to a
cp1252 console and died on Greek letters — the fix was writing the file
directly with `encoding="utf-8"`.)

## Extracted parameters (single source for all scenarios/tests)
- Plant: τ0 = 0.5 s, τ = τ0 (homogeneous), d = 5 m, N = 12 followers.
- Channel: ρ = 5, n = 16 bits, γ = (0.8055, 0.5767, 0.1829, 0.2399, 0.8865,
  0.0287, 0.4899, 0.1679, 0.9787, 0.7127, 0.5005, 0.4711, 0.0596, 0.6820,
  0.0424, 0.0714) — committed as `cacc.network.MA2025_GAMMAS`;
  E[w] = 1.0482.
- Case A: ka = 0.5, h = 0.95 s (h_lb = 0.9375 s), kp = 0.009, kv = 0.63.
- Case B: same but h = 0.65 s (< h_lb ⇒ designed string-unstable).
- Case C: ka* = 0.3183, h = 0.88 s (h* = 0.8727 s), kp = 0.003, kv = 0.85.
- Leader maneuver (their eq. (46)): a0(t) = 0.5·sin(0.1·(t − 10)) m/s² for
  10 < t < 10 + 20π s (exactly one cycle), else 0. t_final = 200 s.

## Gap in the paper and our resolution
The paper never states the platoon's operating speed v0 — legitimately,
because the *linear* error dynamics are speed-independent. Our simulators
and visualizations need one; we chose **v0 = 20 m/s** (72 km/h, typical
highway platooning studies) and noted it in `scenarios/ma2025_sine.yaml`.
Nothing in the reproduction depends on this choice; it only scales absolute
positions and the visual gap (r + h·v0 ≈ 20 m).

## Consequences
- All parameter mirrors trace back to this extraction:
  `scenarios/ma2025_sine.yaml` (offline),
  `ros2_ws/src/cacc_platoon/config/ma2025_case_a*.yaml` (ROS),
  `tests/test_cthp.py` (pinned constants).
- If the published (paywalled) version differs from the preprint, the
  numbers above must be re-checked against IEEE Xplore on campus.
