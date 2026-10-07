"""The derivation of the headway formula, then the six things we added.

The full working lives in `docs/derivations/02-ma2025-base-paper.md` §2.6-2.8
and `report/PRESENTATION_GUIDE.md`. On the deck the derivation earns two
pages, because "derive eq. (17) for me" is the single most likely question a
reviewer asks and the deck used to state it as a result rather than work it.

Every equation here is typeset by `eqmath` and carries a plain-English gloss;
nothing on these pages is ASCII art.
"""

from .theme import ACCENT, ACCENT2, math_slide, set_section


def build(prs, fig):
    # ------------------------------------------ 9-10 THE DERIVATION ------
    set_section("Where the formula comes from")
    math_slide(
        prs, "Where the headway formula comes from — turning the test into algebra",
        [
            ("1", "SQUARE BOTH SIDES — the test becomes a polynomial",
             [r"\left\|\tilde{H}\right\|_\infty \leq 1 "
              r"\quad\Longleftrightarrow\quad "
              r"\left|N(j\omega)\right|^2 \leq \left|D(j\omega)\right|^2 "
              r"\quad \forall\,\omega",
              r"\left|N\right|^2 = (k_p - \tilde{k}_a\omega^2)^2 + k_v^2\omega^2 "
              r"\qquad "
              r"\left|D\right|^2 = (k_p - \omega^2)^2 + (\gamma\omega - \tau\omega^3)^2"],
             "“No frequency is amplified” is awkward to work with, but squaring "
             "it turns it into two polynomials to compare. Write each one out "
             "by separating the real and imaginary parts."),
            ("2", "SUBTRACT — and watch two things vanish, eq. (25)",
             [r"\left|D\right|^2 - \left|N\right|^2 = \omega^2\left[\,"
              r"\tau^2\omega^4 + \left(1 - \tilde{k}_a^2 - 2\tau\gamma\right)\omega^2 "
              r"+ \gamma^2 - 2k_p - k_v^2 + 2\tilde{k}_a k_p \,\right] \;\geq\; 0"],
             "k_p² cancels between the two, and a whole ω² factors out. What is "
             "left is a QUADRATIC in ω² that must never go negative — and our "
             "code checks this identity numerically to 1.8×10⁻¹², so there is "
             "no algebra slip hiding here."),
        ],
        goal="The reviewer's most likely question is “where does 0.9375 come "
             "from?”. It comes from four steps, and these are the first two.",
        subtitle="Eq. (25) — the string-stability inequality, derived rather "
                 "than quoted",
        accent=ACCENT,
        note="Do this on the whiteboard if asked: it is four lines. "
             "docs/derivations/02-ma2025-base-paper.md §2.6.1 has every "
             "cancellation written out.")

    math_slide(
        prs, "Where the headway formula comes from — squeezing the gains out",
        [
            ("3", "DEMAND EACH COEFFICIENT BE POSITIVE, then robustify — eqs. (28a), (28b)",
             [r"(28a)\quad \gamma \;\leq\; "
              r"\frac{1-\left(1+1/\rho\right)^2 k_a^2}{2\tau_0}"
              r"\qquad\qquad "
              r"(28b)\quad \gamma \;\geq\; "
              r"\sqrt{2k_p\left[1-\left(1-1/\rho\right)k_a\right] + k_v^2}"],
             "A quadratic is certainly non-negative if every coefficient is. "
             "Then make it hold for EVERY channel the radio might produce: "
             "(28a) is worst when the channel over-reports, (28b) when it "
             "under-reports. Note (28a) contains NO h_w — remember that."),
            ("4", "CHAIN THE TWO — the gains cancel and only headway survives, eq. (17)",
             [r"\mathrm{square\ (28b)},\ \ \gamma = k_v + h_w k_p \;\Longrightarrow\; "
              r"h_w\left(2k_v + h_w k_p\right) \;\geq\; 2\left(1 - m k_a\right)",
              r"\mathrm{but\ (28a)} \;\Longrightarrow\; 2k_v + h_w k_p \leq 2\gamma "
              r"\leq \frac{1 - n k_a^2}{\tau_0}",
              r"\Longrightarrow\quad h_w \;\geq\; "
              r"\frac{2\tau_0\left(1 - m k_a\right)}{1 - n k_a^2} "
              r"\;=\; h_{w,lb} \qquad m := 1-\frac{1}{\rho},\ \ "
              r"n := \left(1+\frac{1}{\rho}\right)^2"],
             "This is the whole trick. (28b) puts a FLOOR under γ and (28a) puts "
             "a CEILING on it; sandwich them and k_v and k_p drop out, leaving "
             "an inequality in headway alone. Put ρ = 5, τ₀ = 0.5, k_a = 0.5 in "
             "and you get 0.60 / 0.64 = 0.9375 s."),
        ],
        closer="READ THE ASYMMETRY: (28a) has no h_w in it, so no amount of "
               "headway can rescue an over-aggressive γ. That single fact is "
               "why headway-only adaptation hits a wall at ρ* ≈ 1.8, and why "
               "Phase 4 (re-tuning the gains) had to exist.",
        subtitle="Eqs. (28a), (28b) → (17) — the step the deck used to skip",
        accent=ACCENT,
        note="The split in step 3 is SUFFICIENT, NOT NECESSARY: a quadratic can "
             "have a negative middle coefficient and still stay non-negative. "
             "Our counterexample (k_a=0.5, k_v=0.70, k_p=0.009, h_w=1.50) fails "
             "(28a) yet has ‖H̃‖∞ = 1.000000 — so h_lb is CONSERVATIVE for a "
             "platoon whose gains are already fixed.")

    # ------------------------------------------- 11-12 WHAT WE ADDED -----
    set_section("What we added")
    math_slide(
        prs, "What we added (1–3) — undo the delay, measure the channel, move the headway",
        [
            ("1", "PREDICTOR — undo the V2V delay before it destabilises the string",
             [r"\tilde{H}_\theta(s) = \frac{\tilde{k}_a s^2 e^{-\theta s}P(s) "
              r"+ k_v s + k_p}{D(s)} \qquad "
              r"P(s) = 1 + \hat{\theta}A(s)\frac{1 - e^{-sT/2}}{T/2}"],
             "Take it away and the platoon goes string-unstable at a delay of "
             "≈40 ms — too small to think to look for. With it, it survives to "
             "≈220 ms. Delays are evaluated EXACTLY as e^(−jωθ), never Padé, "
             "because our margins are 10⁻³ and Padé phase error would corrupt "
             "the very verdicts we need."),
            ("2", "ESTIMATOR — work out how noisy the radio is, online",
             [r"\hat{w} = \frac{y}{a_{ref}(t - \hat{\theta})} \qquad\qquad "
              r"\hat{\rho} = \frac{\kappa}{Q_{99}\left(\left|\hat{w}-1\right|\right)}"],
             "Compare what the radio SAID against what the radar SAW, and the "
             "spread of the disagreement tells you ρ. Because |w−1| ≤ 1/ρ "
             "STRICTLY, any high quantile errs in a known direction — so ρ̂ is "
             "conservative BY CONSTRUCTION, not by tuning."),
            ("3", "ADAPTER — move the headway, but at a speed a passenger can accept",
             [r"h_i(t) = \mathrm{table}\!\left(\hat{\rho}\right) \qquad "
              r"\left|\dot{h}\right| \leq 0.05\ \mathrm{s/s} \qquad "
              r"\mathrm{extra\ term}\ -\dot{h}v \leq 1\ \mathrm{m/s^2}"],
             "Without it you would need one certified design for every possible "
             "ρ. The rate limit is what keeps a headway change a COMMANDED "
             "manoeuvre rather than a lurch."),
        ],
        subtitle="Phases 1 and 3 (D-008, D-016) — each removes one assumption "
                 "the base paper needs",
        accent=ACCENT2,
        note="Nothing here replaces the paper — it all sits on a reproduction "
             "that matches its printed digits to four decimals.")

    math_slide(
        prs, "What we added (4–6) — re-tune the gains, survive a liar, price the risk",
        [
            ("4", "SCHEDULER — when headway alone is not enough, re-tune the gains",
             [r"\gamma_{target}\!\left(\hat{\rho}\right) = 0.9\cdot"
              r"\frac{1 - \left(1 + 1/\hat{\rho}\right)^2 k_a^2}{2\tau_0}"],
             "Below ρ* ≈ 1.8 there is NO feasible design at any headway — "
             "because the (28a) ceiling ignores h_w entirely. Holding γ at 90 % "
             "of that ceiling restores feasibility, and lands at a SHORTER "
             "headway than the failed headway-only attempt (2.331 s vs 2.50 s)."),
            ("5", "TRUST GATE — the radar is the only second opinion there is",
             [r"r = \left|y_{V2V} - a_{radar}\right| \qquad "
              r"g = \mathrm{clip}\!\left(1 - \frac{r - r_0}{r_1 - r_0},\,0,\,1\right)"],
             "One spoofed link steers the car behind it into a collision "
             "(−27.4 m gap, i.e. cars overlapping). A predecessor chain has no "
             "neighbours to vote with, so consensus filtering is unavailable — "
             "but the same quantity arrives TWICE, over radio and off radar. "
             "Trust the radio only as far as physics allows."),
            ("6", "CERTIFICATE — replace a binary verdict with a priced risk",
             [r"\mathrm{exceed}(h) = \sum_{w} P(w)\cdot"
              r"\mathbf{1}\!\left\{\left\|\tilde{H}(\cdot\,;k_a w)\right\|_\infty > 1\right\} "
              r"\qquad h_{cc}(\varepsilon) = \min\left\{h : \mathrm{exceed}(h) \leq \varepsilon\right\}"],
             "Worst-case design pays the full price for events of probability "
             "10⁻⁶. But the law of w is known EXACTLY — 2¹⁶ = 65 536 atoms — so "
             "there is no need to BOUND, we can INTEGRATE. A regulator sets ε "
             "and the method returns the headway: −10.5 % for a 1-in-100 budget."),
        ],
        subtitle="Phases 4 and 5 (D-017, D-022, D-023) — the flagship extensions",
        accent=ACCENT2,
        note="Each row is measured on the results pages later in the deck; none "
             "of these numbers is asserted without a run behind it.")

    _ = (fig, ACCENT2)
