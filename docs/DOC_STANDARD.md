# DOC_STANDARD — the documentation quality contract

**Status:** in force (user directive, 2026-07-08). This file defines what
every document in this repository must contain, how it must be written,
and tracks the expansion of the corpus to the standard. It is the
instruction sheet for any person or model continuing the documentation
work — read it fully before touching any file under `docs/`.

---

## 1. The mandate

The project owner's directive (2026-07-08, verbatim intent):

> The entire project should be very deep, very detailed, and visual. The
> docs contain the decisions but do not explain them well enough — in
> particular they do not explain **why the alternatives were not chosen**
> in enough detail. Every doc must carry so much information that it is
> **at least 1,000 lines — more is better**. Update the entire corpus.

Binding rules derived from it:

1. **≥ 1,000 lines per content document.** Every file under `docs/`
   except this standard itself (a process instrument). The repo-root
   numbered deliverables (`01…10`) are summaries by design and are
   expanded last, to the same bar where their role permits.
   *This explicitly overrides the general "< 500 lines" file rule for
   Markdown documentation only — code files stay < 500 lines.*
2. **Density, not padding.** A thousand lines of filler is a regression.
   Every section must carry information a competent reader could not
   reconstruct in a minute: derivations, measured numbers, real code,
   real transcripts, real failure stories. If a sentence would survive
   in any other project's documentation unchanged, it is too generic —
   sharpen it with this project's specifics.
3. **Alternatives are first-class.** Any decision-shaped statement gets
   the full options treatment (template A §3): for *each* alternative —
   its mechanism *in this codebase*, what it would buy, what it would
   cost, the evidence, its failure modes, the specific kill reason, and
   the conditions under which it would win a re-match.
4. **Visual.** Every doc has diagrams: ASCII block/signal/timeline
   diagrams (always render everywhere) as the primary form; Mermaid
   (```mermaid fences) as an *additional* form where flow/sequence adds
   value (GitHub renders it; plain viewers still have the ASCII).
   Numbers get tables. Trajectories reference the PNG figures in
   `results/**` by relative path.
5. **Provenance.** Every quoted number traces to a
   `results/**/<stamp>/metrics.json`, a pinned unit test (name it), or a
   derivation shown in the doc. No orphan numbers.
6. **Honesty sections stay.** Limitations, surprises, and negative
   results are part of the record — they are what make the corpus
   credible (cf. `validation/V-04`).

## 2. Voice and style

* Write for a strong engineer who has *not* lived this project. Expand
  every codename on first use; never assume the reader watched the work.
* Prose in full sentences; bullets for enumerable facts; tables for
  numbers; code blocks for anything executable or verbatim.
* Math: unicode in prose (‖H̃‖∞, ρ, θ, τ), fenced blocks for multi-line
  derivations. State units everywhere ([s], [m/s²]).
* File references as `src/cacc/estimation.py:190`-style paths so they
  are clickable in IDEs; test references by pytest node name.
* Each doc begins with a **summary box** (what this doc says, in ≤ 8
  lines) and a **reading map** (which sections which reader needs),
  and ends with **revision history**.
* Cross-reference by ID (`D-016`, `T-07`, `R-05`, `E-03`…) — the corpus
  is a graph, and the IDs are its edges. Every doc links its neighbors.

## 3. Template A — decision records (`decisions/D-###-*.md`)

Sections, in order, with target depth (guide, not straitjacket):

| # | section | must contain | ~lines |
|---|---|---|---|
| 0 | Summary box + metadata table | status, date, type, depends-on, superseded-by, evidence run, pinned tests | 30 |
| 1 | Context | where the project stood; the triggering observation with the *actual* trace/number; timeline | 120 |
| 2 | Problem statement | precise; requirements R1…Rn; constraints C1…Cn; acceptance criteria as of decision time | 60 |
| 3 | Options | one subsection per option (3–6): mechanism here, sketch of the code it would need, pros, cons, risks, evidence, failure modes, **the kill reason**, re-match conditions; ends with a weighted comparison matrix | 350–450 |
| 4 | The decision | exact statement; scope; non-goals | 40 |
| 5 | Design as implemented | architecture/signal diagrams; the equations; code walkthrough with real excerpts; parameter table with provenance; edge cases met during implementation | 200 |
| 6 | Validation & evidence | pinned tests (names + what they pin); experiment tables from metrics.json (run stamp); what would falsify the decision | 120 |
| 7 | Consequences | positive/negative/neutral; debts incurred; interactions with other D-### | 80 |
| 8 | Operational notes | how to use/tune/debug it; common mistakes; observability | 60 |
| 9 | Revisit triggers | IF condition THEN reopen, with the option that would win | 40 |
| 10 | FAQ | 8–12 questions actually asked or foreseeably asked, answered concretely | 80 |
| 11 | Glossary + references + revision history | — | 40 |

## 4. Template B — theory docs (`theory/T-##-*.md`)

0 Summary box → 1 Setting & assumptions (each assumption numbered, with
its physical meaning and what breaks if it fails) → 2 Derivation (every
nontrivial step shown; lemmas separated; sign conventions explicit) →
3 Worked numeric examples (use the project's pinned numbers: case A,
ρ = 5, h = 0.95 s…) → 4 Numerical evaluation notes (how the code
computes it, pitfalls met: branch cuts, 0·∞ in interpolation, bisection
brackets) → 5 Interpretation & intuition (plots referenced, limiting
cases, "what moves when X moves" tables) → 6 Where it lives in code
(functions + tests) → 7 Boundaries of validity & open questions →
8 FAQ → 9 References + revision history.

## 5. Template C — reference docs (`reference/R-##-*.md`)

0 Summary box → 1 Role in the architecture (diagram: this module among
its neighbors) → 2 Public API, item by item (signature, semantics,
units, defaults *and why those defaults*, error behavior) → 3 Internal
design (state, invariants, threading/determinism guarantees) → 4 Worked
examples (runnable snippets with expected output) → 5 Interactions &
contracts with other modules → 6 Anti-patterns (misuse actually seen or
foreseeable, each with the failure it causes) → 7 Performance notes →
8 Test coverage map → 9 Extension points (how to add a controller /
profile / metric…) → 10 FAQ → 11 Revision history.

## 6. Template D — runbooks, experiments, validation, product

* **Runbooks (RB):** every step with the exact command, the *expected
  output* (paste real transcripts), the failure table
  (symptom → diagnosis → fix, from real incidents), and a "how you know
  it worked" checkpoint after each stage.
* **Experiments (E):** motivation → hypothesis → design (factors,
  levels, seeds) → exact reproduction commands → acceptance criteria
  (numeric, with the reason each threshold is where it is) → results
  with run stamps → threats to validity → follow-ups.
* **Validation (V):** claims ↔ evidence matrices; every claim in the
  corpus should be traceable to a row.
* **Product (P):** argument-driven; each capability claim paired with
  the engineering artifact that backs it.

## 7. Diagram conventions

* ASCII: boxes `┌─┐│└─┘`, arrows `──►`, signals labeled with units.
  Keep ≤ 78 columns so nothing wraps.
* Mermaid: `flowchart LR` for pipelines, `sequenceDiagram` for
  node/message timing, `stateDiagram-v2` for mode logic. Always place
  *after* the equivalent ASCII, never instead of it.
* Every figure/PNG referenced gets one paragraph saying what to look at
  and what the reader should conclude.

## 8. The expansion worklist (ledger)

Rules: expand in the order below (user's complaint was decisions first);
update this table (lines + date) after each doc; a doc is DONE only if
≥ 1,000 lines *and* it satisfies §§1–7. Line counts from 2026-07-08
inventory.

| order | doc | was | now | status |
|---|---|---|---|---|
| 1 | decisions/D-016-qos-adaptive-cacc | 80 | | |
| 2 | decisions/D-017-online-gain-retuning | 63 | | |
| 3 | decisions/D-019-adaptation-smoothing-stagger (new) | — | | |
| 4 | decisions/D-018-certification-harness | 43 | | |
| 5 | decisions/D-013-startup-anchor-and-radar-extrapolation | 48 | | |
| 6 | decisions/D-014-wall-clock-integration-substep | 60 | | |
| 7 | decisions/D-008-extension-design-delay-and-loss | 45 | | |
| 8 | decisions/D-001-base-paper-ma2025 | 50 | | |
| 9 | decisions/D-002-stack-python-plus-ros2 | 49 | | |
| 10 | decisions/D-004-noise-in-channel-not-controller | 46 | | |
| 11 | decisions/D-009-low-noise-end-is-worst-case | 33 | | |
| 12 | decisions/D-010-time-domain-contrast-is-subtle | 29 | | |
| 13 | decisions/D-005-standstill-distance-mapping | 28 | | |
| 14 | decisions/D-003-cthp-controller-integration | 43 | | |
| 15 | decisions/D-011-ros2-one-node-per-vehicle | 54 | | |
| 16 | decisions/D-012-ros2-jazzy-install-wsl | 46 | | |
| 17 | decisions/D-006-paper-parameters-from-pdf | 38 | | |
| 18 | decisions/D-007-reproduction-scope-and-validation | 43 | | |
| 19 | decisions/D-015-visualization-rviz2-and-mp4 | 45 | | |
| 20 | theory/T-07-fixed-gain-wall | 52 | | |
| 21 | theory/T-05-noise-model-and-estimation | 48 | | |
| 22 | theory/T-06-delay-and-predictor | 58 | | |
| 23 | theory/T-03-string-stability | 53 | | |
| 24 | theory/T-04-ma2025-theorem-notes | 46 | | |
| 25 | theory/T-08-adaptation-quasi-static-argument | 53 | | |
| 26 | theory/T-01-vehicle-model | 44 | | |
| 27 | theory/T-02-spacing-policy-and-error-dynamics | 40 | | |
| 28 | reference/R-05-estimation | 56 | | |
| 29 | reference/R-06-platoon | 50 | | |
| 30 | reference/R-03-network | 51 | | |
| 31 | reference/R-04-analysis | 48 | | |
| 32 | reference/R-02-vehicle-and-controllers | 48 | | |
| 33 | reference/R-09-ros2-nodes | 50 | | |
| 34 | reference/R-10-messages-and-topics | 43 | | |
| 35 | reference/R-11-scenario-schema | 66 | | |
| 36 | reference/R-01-package-overview | 49 | | |
| 37 | reference/R-07-metrics-and-logging | 31 | | |
| 38 | reference/R-08-scripts | 30 | | |
| 39 | experiments/E-03-qos-zone-experiment | 33 | | |
| 40 | experiments/E-05-gain-retuning-experiment | 36 | | |
| 41 | experiments/E-07-smoothing-stagger (new) | — | | |
| 42 | experiments/E-06-certification-suite | 42 | | |
| 43 | experiments/E-04-delay-predictor-experiment | 34 | | |
| 44 | experiments/E-02-delay-loss-study | 31 | | |
| 45 | experiments/E-01-reproduction-protocol | 29 | | |
| 46 | runbooks/RB-04-troubleshooting | 42 | | |
| 47 | runbooks/RB-03-ros2-workflows | 47 | | |
| 48 | runbooks/RB-01-environment-setup | 41 | | |
| 49 | runbooks/RB-02-offline-workflows | 44 | | |
| 50 | validation/V-04-limitations | 52 | | |
| 51 | validation/V-02-test-inventory | 46 | | |
| 52 | validation/V-01-vv-strategy | 42 | | |
| 53 | validation/V-03-cross-backend-validation | 41 | | |
| 54 | product/P-02-hardware-replacement-argument | 49 | | |
| 55 | product/P-01-virtual-validation-vision | 51 | | |
| 56 | product/P-03-roadmap | 46 | | |
| 57 | root 09_QoS_Adaptive_CACC | 166 | | |
| 58 | root 06/07/08, 10, 01, INDEX | var | | |

## 9. Working procedure (for the executing model)

1. Read the current short doc *and* the source files it describes
   (fresh — never from memory of an older session).
2. Re-verify every number against the latest `metrics.json` / test
   before writing it into prose; update run stamps.
3. Write the full doc in one pass following the genre template; then
   update the ledger row and `docs/INDEX.md` if structure changed.
4. `python -m pytest tests -q` must stay green (58); docs must not
   contradict pinned tests. If a doc reveals a code bug, that becomes a
   new decision record, not a silent fix.
5. Never invent history: if the *why* of an old choice is not
   recoverable from the decision log, transcripts, or code comments,
   say so explicitly in the doc ("rationale reconstructed from…").

---
*Created 2026-07-08 (Claude Fable 5) under the owner's depth directive;
the ledger is live state — keep it current.*
