# IEEE Base Paper (Transactions)

## ⭐ Primary base paper — IEEE Transactions
**G. Ma, P. R. Pagilla, and S. Darbha,**
**"Selection of Time Headway in Connected and Autonomous Vehicle Platoons Under Noisy V2V Communication,"**
***IEEE Transactions on Intelligent Transportation Systems*, vol. 26, no. 1, pp. 1029–1038, Jan. 2025.**

- **Journal:** IEEE T-ITS — flagship ITS Transactions.
- **DOI:** 10.1109/TITS.2024.3498701
- **Free PDF:** arXiv:2404.08889 (author preprint) — downloaded to
  `references/Ma2025_TITS_TimeHeadway_NoisyV2V_arXiv-2404.08889.pdf`.
  Final published version via campus IEEE Xplore.
- **What it does:** predecessor–follower platoon with **constant time headway policy (CTHP)**
  and V2V acceleration feedforward; derives controller-gain constraints and the
  **minimum string-stable time headway** when the communicated acceleration is corrupted
  by noise (robust string stability under imperfect V2V).

### Why it's the right anchor
- **Recent (Jan 2025 issue)** — satisfies any recency requirement.
- Uses **exactly the controller architecture already implemented in `src/cacc`**
  (CTH spacing policy + predecessor-acceleration feedforward).
- Its headline result — minimum stable time gap under imperfect V2V — is this
  project's headline metric.
- **Project novelty on top of it:** extend the imperfect-V2V analysis from *noise* to
  **communication delay + packet loss** in simulation (already supported by
  `src/cacc/network.py`), compare ACC vs CACC, and report smallest string-stable
  time gap → road capacity and energy estimates.

## Classic foundation (kept as key supporting reference)
**J. Ploeg, D. P. Shukla, N. van de Wouw, and H. Nijmeijer,** "Controller Synthesis for
String Stability of Vehicle Platoons," *IEEE Trans. Intell. Transp. Syst.*, vol. 15, no. 2,
pp. 854–865, Apr. 2014. DOI: 10.1109/TITS.2013.2291493 — defines string stability formally
and the CACC feedforward controller; PDF already in `references/`.

## Supporting IEEE Transactions papers (verified, with DOIs)
1. **M. Bouadi, R. Jiang, B. Jia, and S. Zheng,** "String Stability Analysis of Cooperative
   Adaptive Cruise Control Vehicles Considering Multi-Anticipation and Communication Delay,"
   *IEEE T-ITS*, vol. 25, pp. 11359–11369, 2024. DOI: 10.1109/TITS.2024.3371426 — delay analysis
   (Lyapunov); cite for the delay extension.
2. **V. Vegamoor, S. Rathinam, and S. Darbha,** "String Stability of Connected Vehicle Platoons
   Under Lossy V2V Communication," *IEEE T-ITS*, vol. 23, no. 7, pp. 8834–8845, Jul. 2022.
   DOI: 10.1109/TITS.2021.3086809 (free: arXiv:2009.00438) — packet-loss extension; same
   TAMU framework as the base paper.
3. **H. Xing, J. Ploeg, and H. Nijmeijer,** "Compensation of Communication Delays in a
   Cooperative ACC System," *IEEE Trans. Veh. Technol.*, vol. 69, no. 2, pp. 1177–1189,
   Feb. 2020. DOI: 10.1109/TVT.2019.2960114 — delay compensation baseline.
4. **J. Li, C. Chen, J. He, B. Yang, et al.,** "Energy-Efficient Cooperative Adaptive Cruise
   Control for Electric Vehicle Platooning," *IEEE T-ITS*, 2023 (early access Dec. 2023).
   DOI: 10.1109/TITS.2023.3338698 — supports the fuel/energy angle.
5. **Y. Zhang, Y. Bai, M. Wang, et al.,** "Cooperative Adaptive Cruise Control With Robustness
   Against Communication Delay: An Approach in the Space Domain," *IEEE T-ITS*, vol. 22, no. 9,
   pp. 5496–5507, Sep. 2021. IEEE Xplore document 9082096 — alternative (space-domain) approach.

## IEEE-format citations (copy-paste)
```
[1] G. Ma, P. R. Pagilla, and S. Darbha, "Selection of time headway in connected and
    autonomous vehicle platoons under noisy V2V communication," IEEE Trans. Intell.
    Transp. Syst., vol. 26, no. 1, pp. 1029–1038, Jan. 2025, doi: 10.1109/TITS.2024.3498701.
[2] J. Ploeg, D. P. Shukla, N. van de Wouw, and H. Nijmeijer, "Controller synthesis for string
    stability of vehicle platoons," IEEE Trans. Intell. Transp. Syst., vol. 15, no. 2,
    pp. 854–865, Apr. 2014, doi: 10.1109/TITS.2013.2291493.
```
