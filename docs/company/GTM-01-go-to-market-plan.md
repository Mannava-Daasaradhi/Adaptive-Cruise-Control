# GTM-01 — Go-to-market plan (first 90 days)

> **Summary.** Sell a **String-Stability Audit** before selling software: a
> fixed-scope, 2–4 week paid pilot in which the customer's drive logs (or
> controller) go in and a twin, a verdict with uncertainty, and a V2V
> what-if come out. The open-source engine is the credibility and
> distribution layer; the audit is the revenue and learning layer; a license
> is what the audits turn into once they repeat. Everything below is a
> **hypothesis with a kill criterion**, to be confirmed or killed by the
> discovery conversations in GTM-02 — not a forecast.

## 1. Who buys first (ideal customer profile)

| rank | segment | the person | their pain in their words (hypothesis) | why now |
|---|---|---|---|---|
| 1 | **V2X stack / chipset vendors** | product manager or systems engineer for C-V2X applications | "OEMs ask what our latency and loss budgets *do* to their controllers; we have no controller-level evidence" | US DSRC → C-V2X by Dec 2026; OEM integration from MY2026–27 (P-04 [5]) |
| 2 | **ADAS Tier-1 / OEM validation teams** | longitudinal-control or ADAS validation engineer, team lead | "We tune ACC for comfort and safety; nobody signs off string stability, and regulators are starting to ask" | JRC OpenACC evidence (all tested ACCs unstable), UNECE discussion (FRAV working documents) |
| 3 | **Engineering-services firms** (ADAS validation outsourcing: e.g. KPIT, Tata Elxsi, LTTS, Capgemini Engineering) | practice lead for ADAS validation | "We need differentiated test assets to win validation contracts" | they resell capability to many OEMs — a channel, not just a customer |
| 4 | **Convoy / platooning programs** | autonomy lead (Kratos, Forterra, Carnegie Robotics, AV-trucking) | "Degraded or spoofed V2V must not collapse the convoy" | ATV-S down-select; commercial leader-follower corridors |

Start with **1 and 2** in parallel (fast feedback, the product fits today);
**3** as a channel once one audit exists to show; **4** opportunistically
(slow procurement).

## 2. Positioning

- **Category:** controller-level validation for longitudinal automation.
- **One-liner:** *"We tell you whether your ACC amplifies traffic waves —
  from one drive log — and what V2V would change."*
- **Against the status quo** (manual tuning + a few track maneuvers): a
  maneuver can pass while the car is string-unstable (our IDM example:
  every maneuver check passes, the sweep measures 1.038). Frequency-domain
  verdicts with uncertainty catch what maneuvers miss.
- **Against incumbent simulators** (CarMaker, VTD, dSPACE, CARLA): we are not
  a simulator replacement; we are the evaluation engine that runs beside
  them (plugins now, FMU next).
- **Proof points available today:** reproduces Ma et al. (IEEE T-ITS 2025) to
  printed digits; black-box sweep matches analytic Γ to 1e-4; twin
  calibration recovers known ACC parameters to < 3 % and ‖Γ‖∞ to 0.01 on
  noisy 10 Hz logs; nonlinear (IDM) platoons get the right verdict.
- **Missing proof point (priority):** the same study on the real OpenACC
  logs (`scripts/openacc_study.py`, blocked only by data access in the
  build environment).

## 3. The wedge offer: String-Stability Audit (paid pilot)

| | |
|---|---|
| **Input** | 30–120 min of the customer's platoon or car-following logs (10 Hz speed + gap; OpenACC-style test or naturalistic), *or* their controller as a Python plugin |
| **Output** | per-vehicle twin with parameter uncertainty; time-gap margin to string stability; ‖Γ‖∞ with model-free cross-check; V2V what-if (latency sweep); release-gate test plan they keep; 10-page report + 1-hour readout |
| **Duration** | 2–4 weeks |
| **Price hypothesis** | €6k–15k fixed (small enough for a team-lead purchase in many orgs — *verify in discovery*); first 1–2 pilots discounted or free in exchange for a named case study |
| **Success criterion** | the customer says the report changed a tuning or budget decision, and asks for the next vehicle / program |

Why a service first: the customer does not have to install or trust new
software to get value, data stays confidential, each audit shows which parts
of the workflow to productize, and revenue starts before the product is
"done".

## 4. Pricing hypotheses (to test, not to publish)

| offer | hypothesis | kill / revise if |
|---|---|---|
| Audit (per vehicle program) | €6k–15k | 3 qualified prospects all say "no budget line for this" |
| Team license (engine + FMU adapter + standards-shaped plans) | €15k–40k / year | pilots never ask for self-serve use |
| Channel license (engineering-services firm) | revenue share or per-project | no services firm engages after a case study exists |
| Open-source core | free (permissive) | never — it is the credibility and distribution layer |

## 5. 90-day plan

| weeks | engineering | founder | exit criterion |
|---|---|---|---|
| 1–2 | run `openacc_study.py` on the real JRC data (needs data access); publish "State of commercial ACC string stability" table | send 30 personalized outreach messages (GTM-02 §4); book 8–10 discovery calls | ≥ 8 calls booked |
| 3–4 | fix what the real data breaks (parser edge cases, delay model); FMU import spike | hold the calls; log every one (GTM-02 §5) | ≥ 3 prospects confirm the pain in their own words |
| 5–8 | whatever the calls ranked #1 (FMU import, standards plans, or report polish) | 1–2 pilot audits (discounted) with named case-study rights | 1 signed pilot / LOI |
| 9–12 | productize what repeated across pilots | first paid audit; decide license vs. services mix | 1 paid engagement, or a documented pivot decision |

**Kill criteria (be honest):** if after 12 discovery calls fewer than 3
people describe string stability or V2X-latency impact as a problem they
are *paid to solve this year*, the ADAS-validation positioning is wrong —
re-run P-04 §3 with convoy/defense or rail virtual coupling as the lead.

## 6. Metrics that matter now

Discovery calls held · prospects confirming the pain unprompted · pilots
proposed · pilots signed · time from log received to report delivered
(target < 1 day of compute + review). Not now: GitHub stars, sign-ups,
feature count.

## 7. Legal and data notes

- OpenACC is CC BY 4.0: commercial use allowed with attribution
  ("European Commission, Joint Research Centre (JRC): OpenACC").
- Customer logs are confidential: audits run on the customer's premises or
  in a customer-approved environment; the engine needs no network access.
- The base paper's results are cited, not copied; ISO standard texts must
  be purchased and must not be reproduced in the repository.

## Revision history
- 2026-09-25 — created (D-026 sprint).
