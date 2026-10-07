"""Title, team, problem statement, the base paper, novelty, systems.

Scope rules for the whole deck:
  * the ONLY paper discussed is the base paper (Ma, Pagilla & Darbha 2025);
  * no candidate comparison and no paper-selection rationale;
  * no section-divider slides -- every page carries content.

Split across three modules (core / theory / evidence) so no single file runs
past ~450 lines.
"""

from .theme import (ACCENT, ACCENT2, AMBER, CRIMSON, bullets, math_slide,
                    set_section, table_slide, team_slide, title_slide,
                    two_col)


def build(prs, fig):
    # ------------------------------------------------------------ 1 TITLE --
    set_section("CACC under noisy V2V")
    title_slide(
        prs,
        "Selection of Time Headway for String-Stable CACC\nunder Noisy V2V Communication",
        "The base paper reproduced to its printed digits, then extended — "
        "validated across three independent simulation backends",
        [
            "Control Systems — Project Presentation",
            "Base paper:  G. Ma, P. R. Pagilla, S. Darbha, “Selection of Time Headway in "
            "Connected and Autonomous Vehicle Platoons Under Noisy V2V Communication,”",
            "IEEE Transactions on Intelligent Transportation Systems, vol. 26, no. 1, "
            "pp. 1029–1038, January 2025.   doi: 10.1109/TITS.2024.3498701",
            "",
            "Backends:  Python scientific core  ·  ROS 2 distributed (one process per "
            "vehicle)  ·  MATLAB / Simulink block model",
            "Status:  reproduced to the paper's printed digits  ·  87 unit tests green  ·  "
            "Monte-Carlo certification run CERTIFIED",
        ])

    # ------------------------------------------------------------- TEAM --
    team_slide(
        prs,
        [
            ("Mannav Daasaradhi", "cb.ai.u4aid24030"),
            ("Manikanta Koushik", "cb.ai.u4aid24037"),
            ("Prajith", "cb.ai.u4aid24039"),
            ("Sakthi Charukesh", "cb.ai.u4aid24049"),
        ],
        subtitle="Control Systems — Selection of Time Headway for String-Stable "
                 "CACC under Noisy V2V Communication",
        note="Base paper reproduced to its printed digits, then extended — "
             "validated across the Python, ROS 2 and MATLAB / Simulink backends.")

    # -------------------------------------------------- 2-3 PROBLEM ------
    set_section("Problem statement")
    two_col(
        prs, "Problem statement — the physical problem",
        "WHY PLATOONING",
        [
            (0, "Highway capacity is spacing-limited, not speed-limited", "b"),
            (1, "capacity ≈ v / (L + r + h·v) vehicles per lane per second"),
            (1, "at v = 25 m/s: every 0.1 s of headway costs 2.5 m per vehicle"),
            (1, "humans need h ≈ 1.5–2 s; standard ACC still needs h ≥ 1 s"),
            (0, "Two failure modes — only one is obvious", "b"),
            (1, "individual: does vehicle i track its predecessor? Solved by "
                "any stable feedback loop."),
            (1, "STRING stability: does a disturbance GROW as it travels "
                "backwards down the platoon?", "r"),
            (0, "String instability is what phantom traffic jams are", "b"),
            (1, "growth is geometric, gᴺ: a per-hop gain g = 1.0035 is "
                "invisible over 11 vehicles (1.039×) and a factor 2 over 200"),
            (1, "nothing in a 200 s time trace reveals it — the verdict must "
                "be frequency-domain", "a"),
        ],
        "WHY V2V CHANGES THE PROBLEM",
        [
            (0, "Radar alone cannot close the gap", "b"),
            (1, "predecessor-following at CONSTANT SPACING is provably "
                "string-unstable: the double integrator forces sup|Γ| > 1"),
            (1, "constant time headway d = r + h·v fixes it — but the price is "
                "exactly the road space we wanted back"),
            (0, "V2V feedforward is the lever", "b"),
            (1, "broadcasting the predecessor's ACCELERATION lets the follower "
                "act BEFORE the gap changes"),
            (1, "in this model it halves the headway floor: 2τ₀ → τ₀", "g"),
            (0, "…but the lever is a radio", "b"),
            (1, "the one signal CACC adds over ACC is the one that travels "
                "over an imperfect channel", "r"),
            (1, "design for a clean channel, deploy on a dirty one → SILENT "
                "string instability", "r"),
        ],
        accent=CRIMSON,
        note="A 12-vehicle platoon at h = 0.95 s occupies 345 m; at h = 0.88 s it occupies "
             "324 m — 1.75 m per vehicle, ≈6 % more flow.")

    bullets(
        prs, "Problem statement — the research problem",
        [
            (0, "THE QUESTION THE BASE PAPER ASKS", "h"),
            (0, "Given a signal-to-noise ratio ρ on the V2V link carrying the "
                "predecessor's acceleration, what is the SMALLEST time headway h_w at "
                "which the platoon is still GUARANTEED string stable — for every "
                "admissible channel realisation and every vehicle lag τ ∈ (0, τ₀] ?", "a"),
            (0, "Its answer is a closed form: the bound h_{w,lb}(k_a, ρ), the feedforward "
                "gain k_a* that minimises it, and the resulting minimum h*_{w,lb}."),
            (0, "WHAT ITS ANSWER ASSUMES — each assumption is a deployment gap", "h"),
            (1, "channel statistics are TIME-INVARIANT and known at design time"),
            (1, "the guarantee holds in the MEAN sense and is BINARY (pass / fail)"),
            (1, "there is NO communication delay and NO packet loss"),
            (1, "every link is HONEST; only the radio is noisy, the radar is clean"),
            (1, "the gains are chosen ONCE, offline, for one value of ρ"),
            (0, "THE THREE QUESTIONS WE ADD", "h"),
            (0, "1.  Real channels vary. What happens when ρ changes DURING operation, "
                "and can the platoon ESTIMATE it and adapt online?", "g"),
            (0, "2.  Can a binary worst-case verdict be made QUANTITATIVE — a calibrated "
                "probability of exceedance instead of a bound paying for 10⁻⁶ events?", "g"),
            (0, "3.  What if a link LIES? A predecessor-following chain has no redundancy, "
                "so consensus-style filtering cannot be applied.", "g"),
        ],
        subtitle="From “does the paper work?” to “would it survive deployment?”",
        accent=CRIMSON,
        note="Success criteria: reproduce the paper exactly · quantify its deployment gaps · "
             "build and validate the fix · certify on a Monte-Carlo harness · agree across "
             "three independent backends.")

    # ------------------------------------------------- 5 DECODER RING ----
    set_section("Decoder ring")
    table_slide(
        prs, "The decoder ring — every symbol in the deck, in plain words",
        ["Symbol", "Name", "What it actually means", "Value here"],
        [
            [("h_w", "b"), "time headway",
             "SECONDS of gap you keep behind the car ahead. Multiply by speed "
             "to get metres. THE quantity this whole project is about.",
             ("0.95 s", "g")],
            [("d = r + h_w·v", "b"), "spacing policy",
             "the gap you aim for: a fixed bumper distance plus h_w seconds of "
             "travel — so the faster you go, the more room you leave",
             "r → d = 5 m"],
            [("δ_i", "b"), "spacing error",
             "how far car i actually is from that target gap. Drive δ to zero "
             "and the gap is right at every speed.", "metres"],
            [("τ", "b"), "actuator lag",
             "how sluggish the engine/brake loop is — you ask for an "
             "acceleration, you get it about τ seconds later", "≤ τ₀ = 0.5 s"],
            [("ρ", "b"), "channel SNR",
             "radio quality. BIG ρ = clean radio, small ρ = noisy. Everything "
             "in the paper is a function of it.", "5  (13.98 dB)"],
            [("w", "b"), "channel factor",
             "the number the TRUE acceleration gets multiplied by in transit. "
             "w = 1 would be a perfect radio.", "1 ± 1/ρ"],
            [("k_a", "b"), "feedforward gain",
             "how hard you act on what the RADIO tells you the car ahead is "
             "doing", "0.5"],
            [("k_v", "b"), "damping gain",
             "how hard you act on the speed difference you SEE on radar",
             "0.63"],
            [("k_p", "b"), "spacing gain",
             "how hard you act on the gap error you SEE on radar", "0.009"],
            [("γ = k_v + h_w k_p", "b"), "combined authority",
             "total feedback strength. THE ONE PLACE headway enters the "
             "maths — remember this and slide 10 makes sense.", ("—", "a")],
            [("H̃(s)", "b"), "error propagation",
             "the filter that turns the car-ahead's gap error into THIS car's "
             "gap error. One filter, every hop.", ("—", "a")],
            [("‖H̃‖∞", "b"), "peak gain",
             "the WORST amplification that filter does, over every frequency. "
             "The single number the verdict is taken on.", ("must be ≤ 1", "g")],
        ],
        subtitle="Read this page once and every equation later in the deck is "
                 "already introduced",
        col_w=[1.85, 1.75, 6.15, 1.45], accent=CRIMSON,
        note="Rule of thumb for the whole deck: k_a is the RADIO gain, k_v and "
             "k_p are the RADAR gains, h_w is the price in road space, and ρ is "
             "how much you can trust the radio today.")

    # ----------------------------------------------- 6-7 BASE PAPER ------
    set_section("Base paper")
    math_slide(
        prs, "Base paper — the car, and the gap it aims for",
        [
            ("1", "THE CAR — you ask for an acceleration, you get it late",
             [r"\ddot{x}_i = a_i \qquad "
              r"\tau\,\dot{a}_i + a_i = u_i \qquad \tau \in (0,\tau_0]"],
             "u is the acceleration you COMMAND, a is what the car actually "
             "delivers, and τ is the delay between them. τ is never known "
             "exactly — only that it is no worse than τ₀ = 0.5 s, which is why "
             "every guarantee must hold for a whole RANGE of τ."),
            ("2", "THE GAP IT AIMS FOR — spacing error, eq. (2)",
             [r"e_i = x_i - x_{i-1} + d \qquad\Longrightarrow\qquad "
              r"\delta_i = e_i + h_w v_i"],
             "δ is how far this car is from where the time-headway policy says "
             "it should be. Push δ to zero and the gap is correct at any speed."),
        ],
        subtitle="Eqs. (1) and (2) — the plant and the spacing policy",
        goal="Two ideas before any control appears: a car cannot change speed "
             "instantly, and the gap it aims for grows with speed.",
        note="τ is the DOMINANT lag of the inner acceleration loop, not one "
             "component's time constant. A mixed fleet is absorbed by "
             "τ₀ = maxᵢ τᵢ, so a design certified on (0, τ₀] covers every "
             "vehicle below τ₀.")

    math_slide(
        prs, "Base paper — the control law, and the radio that corrupts it",
        [
            ("3", "THE CONTROL LAW — three terms, three information sources, eq. (6)",
             [r"u_i \;=\; k_a\,w_{i,i-1}\,a_{i-1} \;-\; k_v\,(v_i - v_{i-1}) "
              r"\;-\; k_p\,\delta_i",
              r"\mathrm{FEEDFORWARD\ (radio)} \qquad "
              r"\mathrm{DAMPING\ (radar)} \qquad \mathrm{SPACING\ (radar)}"],
             "Left to right: act on what the car ahead is DOING, on the speed "
             "difference, and on the gap error. ONLY the first term travels "
             "over the radio — so only the first is corrupted. That asymmetry "
             "is the entire premise of the paper."),
            ("4", "THE RADIO — the 16-bit multiplicative channel, eq. (5)",
             [r"w_{i,i-1}(t) = \left(1-\frac{1}{\rho}\right) "
              r"+ \frac{1}{\rho}\sum_{j=0}^{15} z_{i,j}(t)\,2^{-j}",
              r"\Longrightarrow\quad 1-\frac{1}{\rho} \;\leq\; w \;<\; "
              r"1+\frac{1}{\rho} \qquad E[w] = 1.0482"],
             "The acceleration you RECEIVE is the true one multiplied by w. w "
             "is built from 16 random bits, so it can never stray further than "
             "1/ρ away from 1 — it is BOUNDED, and that is the only reason a "
             "hard guarantee is possible at all. Note E[w] ≠ 1: the channel is "
             "optimistically biased by 4.8 %."),
        ],
        subtitle="Eqs. (6) and (5) — N = 12 followers · τ₀ = 0.5 s · d = 5 m · "
                 "ρ = 5 (13.98 dB) · v_ss = 25 m/s",
        closer="A Gaussian noise model would have infinite tails, so NO finite "
               "headway could ever guarantee string stability. Bounded support "
               "is the only reason the deterministic theorem exists at all.")

    math_slide(
        prs, "Base paper — how errors travel, and the test they must pass",
        [
            ("1", "ONE FILTER GOVERNS EVERY HOP — eq. (14)",
             [r"\bar{\delta}_i(s) = \tilde{H}(s;\tau)\,\bar{\delta}_{i-1}(s)"
              r"\qquad \gamma := k_v + h_w k_p",
              r"\tilde{H}(s;\tau) = \frac{\tilde{k}_a s^2 + k_v s + k_p}"
              r"{\tau s^3 + s^2 + \gamma s + k_p}"],
             "Errors do not ADD down the platoon, they get MULTIPLIED by this "
             "one filter at every hop. Headway enters only through γ, and γ "
             "only in the bottom line — so headway can add damping, never gain. "
             "A 200-car question just became a one-car question."),
            ("2", "THE TEST — robust string stability, Definition 2",
             [r"\left\|\tilde{H}(j\omega;\tau)\right\|_\infty \leq 1 "
              r"\qquad \forall\,\tau \in (0,\tau_0] \quad "
              r"\forall\,\tilde{k}_a \in I"],
             "No frequency may be amplified — for any actuator lag up to τ₀ and "
             "any channel the radio might hand you. Note it can never be BETTER "
             "than 1: at zero frequency the filter equals exactly 1, so the "
             "margin is zero by construction."),
        ],
        subtitle="Eqs. (14) and Definition 2 — why a 200-car question is really "
                 "a one-car question",
        note="Gains MULTIPLY down a chain, so the per-hop gain is raised to the "
             "Nth power. A per-hop 1.0035 is 1.039× over 11 vehicles and a "
             "factor 2 over 200 — which is why the verdict is taken in the "
             "frequency domain and never off a time trace.")

    math_slide(
        prs, "Base paper — the answer, and the best it can do",
        [
            ("3", "THE ANSWER — the smallest headway that can be certified, eq. (17)",
             [r"h_{w,lb}(k_a,\rho) = \frac{2\tau_0\left[1-(1-1/\rho)k_a\right]}"
              r"{1-(1+1/\rho)^2 k_a^2} \qquad "
              r"\mathrm{valid\ for}\ \ 0 < k_a < \frac{1}{1+1/\rho}"],
             "Given how clean the radio is (ρ) and how hard you lean on it "
             "(k_a), this is the shortest headway that can still be certified. "
             "Go below it and NO choice of the two radar gains can rescue you."),
            ("4", "THE BEST YOU CAN DO — the optimum, eqs. (18) and (19)",
             [r"k_a^{*} = \frac{1-1/\sqrt{\rho}}{1+1/\sqrt{\rho}}\cdot"
              r"\frac{1}{1+1/\rho} \qquad "
              r"h^{*}_{w,lb} = \tau_0\,\frac{\left(1+1/\sqrt{\rho}\right)^2}"
              r"{1+1/\rho}"],
             "Trusting the radio more shortens the headway — until the noise it "
             "carries hurts more than the information helps. k_a* is that "
             "balance point. At ρ = 5 it is 0.3183, which buys 0.8727 s."),
        ],
        subtitle="Eqs. (14), (16)–(19) — the whole result set, each line said in "
                 "words underneath",
        note="At ρ = 5, τ₀ = 0.5 s the paper prints k_a bound 0.8333, "
             "h_{w,lb}(0.5) = 0.9375 s, k_a* = 0.3183 and h*_{w,lb} = 0.8727 s — "
             "all four reproduced to 0.00 %. The five numerical cases of §IV: "
             "1 stable (h = 0.95) · 2 unstable (h = 0.65) · 3 at the optimum "
             "(h = 0.88) · 4 platoon length 345 m vs 324 m · 5 heterogeneous τᵢ.")

    # ---------------------------------------------- 6-7 NOVELTY ----------
    set_section("Novelty & comparison")
    table_slide(
        prs, "Novelty, phase-wise — the seven phases",
        ["Phase", "What it adds", "Assumption removed", "Key result", "ADR"],
        [
            [("0  Reproduce", "b"),
             "Full re-implementation from the paper: CTHP law, 16-bit channel, RK4 platoon, "
             "frequency-domain analysis",
             ("— (fidelity baseline)", "q"),
             ("printed digits exact:\n0.9375 / 0.3183 / 0.8727 / 0.8333", "g"),
             "D-001…07"],
            [("1  Gap audit", "b"),
             "V2V latency θ on the feedforward path + Bernoulli packet loss with ZOH, at "
             "FIXED case-A gains",
             "“no delay”, “no packet loss”",
             ("DELAY binds, not noise: only ≈0.2 s of budget at the paper's own design", "w"),
             "D-008"],
            [("2  Distribute", "b"),
             "One OS process per vehicle on ROS 2; wall-clock substepped integration; radar "
             "extrapolation by message age",
             "“a monolithic simulator is enough”",
             ("< 2 % per-hop agreement with the offline core under real OS jitter", "g"),
             "D-011…15"],
            [("3  Adapt  ★", "b"),
             "Online SNR estimation ρ̂ from radar-vs-beacon residuals → rate-limited headway "
             "adaptation + timestamp predictor",
             "“γ_{i,j} are time-invariant”",
             ("ρ̂ median {10.4, 3.1, 10.7} vs true {10, 3, 10}; delay penalty flatlined", "g"),
             "D-016"],
            [("4  Re-tune", "b"),
             "Online gain scheduling: hold γ at 90 % of the (28a) ceiling for the estimated ρ",
             "“gains are fixed once chosen”",
             ("breaks the fixed-gain wall at ρ* ≈ 1.8:\n‖H̃‖∞ 1.019 → 0.99999", "g"),
             "D-017"],
            [("5  Flagship  ★★", "b"),
             "A: spatial ρ(x) map with look-ahead · B: physics-consistency V2V trust gate · "
             "C: exact channel law → chance-constrained headway",
             "“reactive is enough”, “links are honest”, “binary worst case”",
             ("A: 0 % unstable in-zone (vs 6.9 %) · B: collision → 8.4 m gap · C: −10.5 % "
              "headway at 1e−2 risk", "g"),
             "D-021…23"],
            [("6  Certify", "b"),
             "Monte-Carlo scenario×seed harness → PASS/FAIL report; plus Simulink as an "
             "independent third backend",
             "“one implementation is evidence”",
             ("CERTIFIED across 4 scenario families; Simulink ↔ Python agree to 1e−4", "g"),
             "D-018, 24"],
        ],
        subtitle="★ = core research contribution   ★★ = the flagship extensions",
        col_w=[1.25, 3.3, 2.0, 3.3, 0.85], highlight_rows=(3, 5))

    # -------------------------------------------------- 8 SYSTEMS --------
    set_section("Systems")
    two_col(
        prs, "Systems — what we built, and the three backends it runs on",
        "THE MODEL AND THE CHANNEL",
        [
            (0, "S1  VEHICLE + CONTROLLERS   vehicle.py, controllers.py", "h"),
            (1, "state x = [p, v, a]; ȧ = (u − a)/τ, u saturated to "
                "[−8, +3] m/s² BEFORE integration"),
            (1, "τ = 0.5 s, L = 4 m, r = 1 m ⟹ d = 5 m, matching the paper"),
            (1, "three laws behind ONE interface — ACC (radar only), CACC "
                "(+ unity feedforward), CTHP (the base-paper law)"),
            (1, "CTHP is STATELESS, so the gain scheduler can swap the "
                "instance mid-run with no state transfer", "a"),
            (0, "S2  V2V CHANNEL   network.py, qos_map.py", "h"),
            (1, "four independently switchable impairments — NOISE (w per "
                "eq. 5, held 10 ms) · DELAY θ · LOSS + zero-order hold · "
                "ATTACK injection on one link"),
            (1, "the ρ-INDEPENDENT bit cache: it stores U′ = Σ z_j 2⁻ʲ, not "
                "w, so a ρ-schedule RESCALES the same realisation instead of "
                "redrawing it", "a"),
            (1, "⟹ two zone studies are seed-comparable: the difference "
                "between runs is the schedule, not the dice", "g"),
            (1, "QoSMap gives channel quality by POSITION, ρ(x), and exposes "
                "it at (x + v·preview) — the look-ahead that makes adaptation "
                "predictive rather than reactive"),
        ],
        "ANALYSIS, ADAPTATION, AND EXECUTION",
        [
            (0, "S3  ANALYSIS + CERTIFICATION   analysis.py, certificate.py", "h"),
            (1, "delays evaluated EXACTLY as e^{−jωθ}, never Padé — margins "
                "here are 10⁻³ and Padé phase error would corrupt exactly the "
                "verdicts we need", "a"),
            (1, "min_stable_headway BISECTS on h, because (28a)/(28b) are "
                "sufficient-not-necessary; hinf_worst grids k̃_a across I so "
                "the binding endpoint is CHECKED, not assumed", "a"),
            (1, "ChannelLaw.build(ρ) — the EXACT law of w, 2¹⁶ = 65 536 atoms"),
            (0, "S4  ADAPTATION   estimation.py, trust.py", "h"),
            (1, "ρ̂ from radar-vs-beacon residuals over an AGE-based window — "
                "a count-based window was tried first and FAILED (ρ̂ stuck at "
                "3.1 after zone exit)", "w"),
            (1, "h-table built ONCE by bisection, then rate-limited to "
                "0.05 s/s; scheduler holds γ at 90 % of the (28a) ceiling"),
            (0, "S5  THREE BACKENDS, ONE SET OF EQUATIONS", "h"),
            (1, "Python core — hand-written RK4 over the stacked state", "g"),
            (1, "ROS 2 — one OS process per vehicle; the nodes IMPORT cacc.*, "
                "so there is no second implementation to drift", "g"),
            (1, "Simulink — a width-N model built programmatically from a "
                "reviewable .m, so the .slx is a diffable artefact", "g"),
        ],
        note="Declarative YAML scenarios ⟹ a study is a name plus a seed; each "
             "run writes results/<study>/<stamp>/ with figures AND metrics.json, "
             "so every number on a later slide traces to a file.")

    _ = (AMBER, fig)
