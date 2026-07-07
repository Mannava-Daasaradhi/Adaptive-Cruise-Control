# D-001 — Base paper switched to Ma, Pagilla & Darbha 2025

**Date:** 2026-07-05 · **Status:** accepted · **Type:** project scope

## Context
The project originally anchored on Ploeg, van de Wouw & Nijmeijer,
"Lp String Stability of Cascaded Systems: Application to Vehicle Platooning"
(2014) — a classic, but 12 years old at project time. The user explicitly
asked for a *recent* IEEE **Transactions** paper to serve as the base paper
for a sem-5 Control Systems course project whose deliverables are a
simulation, an IEEE-style report, and slides. The existing codebase
(`src/cacc`) already implemented a constant-time-headway policy (CTHP) with
acceleration feedforward over V2V, so the ideal base paper would use the
same architecture and add a quantitative, reproducible headline result.

## Options considered
1. **Ma, Pagilla & Darbha, "Selection of Time Headway in Connected and
   Autonomous Vehicle Platoons Under Noisy V2V Communication," IEEE T-ITS
   26(1):1029–1038, Jan. 2025, DOI 10.1109/TITS.2024.3498701** — CHOSEN.
   Same CTHP + accel-feedforward architecture; closed-form minimum
   string-stable headway under multiplicative n-bit channel noise
   (their Theorem III.2); free preprint arXiv:2404.08889.
2. Vegamoor, Rathinam & Darbha 2022 (T-ITS, packet drops) — good, older,
   kept as the runner-up if the professor rejects the noise angle.
3. Bouadi et al. 2024 (T-ITS, delay, Lyapunov–Krasovskii functionals) —
   very recent but the LKF machinery is too heavy for a sem-5 deliverable.
4. Xing, Ploeg & Nijmeijer 2020 (TVT, delay compensation) — TVT not T-ITS,
   and older.

## Decision
Adopt Ma 2025 as the primary base paper. Demote Ploeg 2014 to "classic
foundation" status in `02_IEEE_Base_Paper.md`. Reframe the proposal
(`01_Proposal.md`) as: *base paper covers noise → minimum headway; this
project reproduces it and extends the robustness study to V2V delay and
packet loss* (the extension is the novelty; see [D-008]).

## Why this one
- **Architecture match:** their control law u_i = ka·w(t)·a_{i−1} −
  kv(v_i−v_{i−1}) − kp·δ_i is exactly a CTHP with realized-acceleration
  feedforward — `src/cacc` needed one new controller class, not a rewrite
  (see [D-003]).
- **Reproducible numbers:** the paper prints closed-form values
  (h_lb = 0.9375 s at ka = 0.5, ρ = 5; ka* = 0.3183; h* = 0.8727 s) that we
  pinned as unit tests ([D-007]).
- **Headline = project metric:** minimum string-stable time headway is both
  their result and our course-demo metric (throughput ∝ 1/h).
- **Access:** preprint is free (`references/Ma2025_TITS_TimeHeadway_
  NoisyV2V_arXiv-2404.08889.pdf`); final version needs campus IEEE Xplore.

## Consequences
- `02_IEEE_Base_Paper.md` rewritten around Ma 2025 with five verified
  supporting Transactions references (Bouadi 2024, Vegamoor 2022, Xing
  2020, Li 2023, Zhang 2021 — all DOI-checked).
- Reproduction scope defined in [D-007]; extension in [D-008].
- The proposal title became "…Under Imperfect V2V Communication: Noise,
  Delay, and Packet Loss".
