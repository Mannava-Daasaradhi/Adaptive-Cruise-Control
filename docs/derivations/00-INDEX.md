# Derivations compendium — index

Every result in the project's six reference papers, derived from first
principles. Written to be defended on a whiteboard with no notes.

**Rule of the corpus:** nothing is quoted from a paper without being rebuilt
from something more primitive. Where a paper states a result without proof, the
proof is supplied here. Where a paper's printed number disagrees with what its
own stated parameters produce, that is recorded as a finding, not smoothed
over.

---

## The documents

| # | file | covers |
|---|---|---|
| I | [`01-foundations.md`](01-foundations.md) | Results **F-1 … F-24**: the longitudinal model, spacing policies, signal and system norms, induced-gain theorems, Routh–Hurwitz, the Bernoulli channel, and the competing string-stability definitions |
| II | [`02-ma2025-base-paper.md`](02-ma2025-base-paper.md) | **Ma, Pagilla & Darbha 2025** (the base paper) — every equation (1)–(45), both proofs of Theorem 2(b), the `k_a*` optimisation, all five numerical cases, and the reproduction status |
| III | [`03-ploeg-string-stability.md`](03-ploeg-string-stability.md) | **Ploeg 2014** — L_p string stability (Def. 1, Thms 1–2), `Γ(s)`, H∞ synthesis, two-vehicle look-ahead, graceful degradation (dCACC) |
| IV | [`04-koroglu-delay-headway.md`](04-koroglu-delay-headway.md) | **Köroğlu 2024** — minimum normalised headway under delay, `h̄_o = √(2δ̄(√(1+0.5δ̄)+1)) ≈ 2√δ̄` |
| V | [`05-zhao-fdia-resilient.md`](05-zhao-fdia-resilient.md) | **Zhao et al. 2024** — FDIA, the F-total model, MSR filtering, graph robustness, stability under attack |
| VI | [`06-ke-deep-learning-cacc.md`](06-ke-deep-learning-cacc.md) | **Ke 2022** — MDPs, Bellman, TD/Q-learning, double Q-learning, DQN/DDQN, and a critical appraisal of learned CACC |
| VII | [`07-synthesis-and-drills.md`](07-synthesis-and-drills.md) | **cross-paper synthesis**, the master formula sheet, the twelve numbers to memorise, a 38-question bank, and ten timed whiteboard drills |

---

## How to use this

**If you have a week.** Read Part I once, properly — everything else calls back
to it by number. Then Part II end to end; it is the base paper and carries the
most weight. Then III, IV, V, VI in any order. Finish with VII and do the ten
drills against a clock.

**If you have a day.** Part VII §7.4 (formula sheet) and §7.5 (the twelve
numbers), then Part II §§2.4–2.8 (the derivations that will actually be asked
for), then Part VII §7.6 Tier 3 (the questions that separate understanding from
memorisation).

**If you have an hour.** Part VII §§7.2–7.5, and be able to draw the diagram in
§7.2 from memory. It is the whole literature in one picture.

**Every derivation in Parts II–VI is verified numerically.** Run
`matlab/scripts/theorem_checks.m` to see every closed form checked against an
independent route (grid search, polynomial residual, `roots()`), and
`matlab/run_all.m` to regenerate the base paper's figures.

---

## Reading order by question

| if you are asked… | go to |
|---|---|
| "what is string stability?" | I §6 (F-22 … F-24), III §3.3 |
| "derive the transfer function" | II §2.4 |
| "why does headway help?" | I §2 (F-5), II §2.4 obs. 2 |
| "derive the minimum headway" | II §§2.5.2, 2.7.1 |
| "what's the optimal gain?" | II §2.8 |
| "what does noise do?" | I §5, II §§2.2, 2.6.3 |
| "what does delay do?" | III §3.4, IV §§4.5–4.6 |
| "what about packet loss?" | III §3.7 |
| "what about attacks?" | V |
| "can machine learning do it?" | VI §6.7 |
| "how do the papers relate?" | VII §§7.1–7.3 |
| "what are the limitations?" | II §2.11, V §5.7, VI §6.7 |

---

## Sources

All PDFs are in [`../../references/`](../../references/).

1. G. Ma, P. R. Pagilla, S. Darbha, "Selection of Time Headway in Connected and
   Autonomous Vehicle Platoons Under Noisy V2V Communication," *IEEE T-ITS*
   **26**(1):1029–1038, 2025. doi 10.1109/TITS.2024.3498701
2. J. Ploeg, N. van de Wouw, H. Nijmeijer, "L_p String Stability of Cascaded
   Systems: Application to Vehicle Platooning," *IEEE T-CST* **22**(2):786–793,
   2014. doi 10.1109/TCST.2013.2258346
3. J. Ploeg, *Controllers for Cooperative and Automated Driving*, PhD thesis,
   TU Eindhoven, 2014.
4. H. Köroğlu, "String-Stable Cooperative Adaptive Cruise Control With
   Minimized Time Headway in the Face of Delayed Communication," *IEEE L-CSS*
   **8**:400–405, 2024. doi 10.1109/LCSYS.2024.3392716
5. C. Zhao, R. Ma, M. Wang, J. Xu, L. Cai, "Safeguard Vehicle Platooning Based
   on Resilient Control Against False Data Injection Attacks," *IEEE T-ITS*
   **25**(11):17023–17036, 2024. doi 10.1109/TITS.2024.3424687
6. H. Ke, *Cooperative Adaptive Cruise Control using V2V Communication and Deep
   Learning*, M.A.Sc. thesis, University of Windsor, 2022.

---

## Related project documents

* [`../theory/`](../theory/) — T-01 … T-08, the same physics as implemented in
  `src/cacc`, with this project's own findings (D-009 binding noise end, D-010
  frequency-vs-time verdicts, T-07 sufficiency gap)
* [`../validation/V-05-simulink-reproduction.md`](../validation/V-05-simulink-reproduction.md)
  — the Simulink reproduction results and the open Case 3 discrepancy
* [`../../matlab/README.md`](../../matlab/README.md) — how to run the Simulink
  backend
* [`../decisions/`](../decisions/) — D-001 … D-024, why the project is built
  the way it is
