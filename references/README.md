# References

The papers this project builds on. The PDFs are **not** in the repository:
publisher versions (IEEE and others) may not be redistributed, and arXiv's
default licence does not allow third-party redistribution either. Keep your
own copies in this folder (`references/*.pdf` is git-ignored) and fetch them
from the links below.

## Base paper

| # | Reference | Link |
|---|---|---|
| 1 | G. Ma, P. R. Pagilla, S. Darbha, "Selection of Time Headway in Connected and Autonomous Vehicle Platoons Under Noisy V2V Communication," *IEEE T-ITS* 26(1):1029–1038, 2025 | [doi:10.1109/TITS.2024.3498701](https://doi.org/10.1109/TITS.2024.3498701) · [arXiv:2404.08889](https://arxiv.org/abs/2404.08889) |

## IEEE Transactions / Letters foundations

| # | Reference | Link |
|---|---|---|
| 2 | J. Ploeg, D. P. Shukla, N. van de Wouw, H. Nijmeijer, "Controller Synthesis for String Stability of Vehicle Platoons," *IEEE T-ITS* 15(2):854–865, 2014 | [doi:10.1109/TITS.2013.2291493](https://doi.org/10.1109/TITS.2013.2291493) |
| 3 | J. Ploeg, N. van de Wouw, H. Nijmeijer, "Lp String Stability of Cascaded Systems: Application to Vehicle Platooning," *IEEE T-CST* 22(2):786–793, 2014 | [doi:10.1109/TCST.2013.2258346](https://doi.org/10.1109/TCST.2013.2258346) |
| 4 | S. Öncü, J. Ploeg, N. van de Wouw, H. Nijmeijer, "Cooperative Adaptive Cruise Control: Network-Aware Analysis of String Stability," *IEEE T-ITS* 15(4):1527–1537, 2014 | [doi:10.1109/TITS.2014.2302816](https://doi.org/10.1109/TITS.2014.2302816) |
| 5 | H. Köroğlu, "String-Stable Cooperative Adaptive Cruise Control With Minimized Time Headway in the Face of Delayed Communication," *IEEE L-CSS* 8:400–405, 2024 | [doi:10.1109/LCSYS.2024.3392716](https://doi.org/10.1109/LCSYS.2024.3392716) |
| 6 | C. Zhao, R. Ma, M. Wang, J. Xu, L. Cai, "Safeguard Vehicle Platooning Based on Resilient Control Against False Data Injection Attacks," *IEEE T-ITS* 25(11):17023–17036, 2024 | [doi:10.1109/TITS.2024.3424687](https://doi.org/10.1109/TITS.2024.3424687) |

## Theses

| # | Reference | Where |
|---|---|---|
| 7 | J. Ploeg, *Analysis and Design of Controllers for Cooperative and Automated Driving*, PhD thesis, TU Eindhoven, 2014 | TU/e research portal |
| 8 | H. Ke, *Cooperative Adaptive Cruise Control using V2V Communication and Deep Learning*, M.A.Sc. thesis, University of Windsor, 2022 | University of Windsor Scholarship repository |

## Background reading (preprints, not cited as evidence)

| # | Reference | Link |
|---|---|---|
| 9 | Stochastic Lp string stability under packet losses | [arXiv:2403.11043](https://arxiv.org/abs/2403.11043) |
| 10 | ACC in autonomous vehicles — review | [arXiv:2510.03300](https://arxiv.org/abs/2510.03300) |
| 11 | Ma et al., parasitic actuation delay in CACC | [arXiv:2509.20722](https://arxiv.org/abs/2509.20722) |
| 12 | Samii et al., predictor-based CACC with actuation delay | [arXiv:2507.21579](https://arxiv.org/abs/2507.21579) |
| 13 | Liu, Somarakis, Motee, cascading collision risk under delayed communication | [arXiv:2312.17147](https://arxiv.org/abs/2312.17147) |
| 14 | Lyons et al., distributed attack-resilient platooning (FDIA) | [arXiv:2411.04789](https://arxiv.org/abs/2411.04789) |
| 15 | Pandey, Motee, entropic value-at-risk of platoon collisions | [arXiv:2605.08015](https://arxiv.org/abs/2605.08015) |

## Prior art for the product and the flagship extensions

Read these before claiming novelty (see `docs/flagship/00_Flagship_Overview.md`):

- G. Gunter et al., "Are commercially implemented adaptive cruise control systems string stable?", *IEEE T-ITS* — fits linear delay models to 7 production ACC cars ([arXiv:1905.02108](https://arxiv.org/abs/1905.02108)). Closest prior work to `cacc calibrate` / `cacc audit`.
- A. Pfadler, G. Jornod, A. El Assaad, P. Jung, "Predictive Quality of Service: Adaptation of Platoon Inter-Vehicle Distance to Packet Inter-Reception Time," IEEE VTC2020-Spring; and US 11,972,689 B2 (Volkswagen AG, priority 2019-09-09). Closest prior work to Flagship A.
- US 12,043,272 B2 (GM Global Technology Operations, priority 2022-07-26), radar/camera fusion for V2X misbehaviour detection; R. van der Heijden et al., analysing attacks on CACC ([arXiv:1710.05789](https://arxiv.org/abs/1710.05789)). Closest prior work to Flagship B.
- F. Acciani, P. Frasca, G. Heijenk, A. Stoorvogel, "Stochastic String Stability of Vehicle Platoons via Cooperative Adaptive Cruise Control With Lossy Communication," *IEEE T-ITS* 23(8):10912–10922, 2022 ([arXiv:1905.04779](https://arxiv.org/abs/1905.04779)). Closest prior work to Flagship C.
- M. Makridis et al., "OpenACC. An open database of car-following experiments to study the properties of commercial ACC systems," *Transportation Research Part C*, 2021 ([arXiv:2004.06342](https://arxiv.org/abs/2004.06342)). The field data the audit is meant to run on.
