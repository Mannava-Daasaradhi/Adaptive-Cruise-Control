# D-013 — ROS 2 startup: radar extrapolation + 2 s anchor hold

**Date:** 2026-07-06 · **Status:** accepted · **Type:** real-time correctness

## Problem
ROS nodes spawn and DDS-discover each other asynchronously. Consequences
observed in real runs (case A, 6 followers):
1. Nodes begin ticking at wall-clock offsets of tens of ms — each vehicle's
   notion of "exact initial spacing" refers to a slightly different
   instant, freezing an O(v0·Δt_spawn) position bias into the spacing error.
2. Worse: the **first predecessor-state message can be 10–100 ms stale**
   (DDS discovery/queueing latency). At v0 = 20 m/s, 100 ms of staleness is
   a **2 m** error — measured directly: vehicle 1's quiescent-phase mean
   error was +2.07 m in run `20260706-182620`.
3. Any startup offset decays on the *slow* closed-loop mode: the smallest
   pole is ≈ kp/γ with γ = kv + h·kp ⇒ time constant ≈ 70 s for case-A
   gains — so a startup bias pollutes the L2 norm of the **entire 200 s
   run**. Observed per-hop ratios were 0.66–1.33 instead of ≈ 1.

## Failed attempts (kept for the record)
- *Post-hoc detrending only* (subtract pre-maneuver mean in the analysis):
  removes constant bias but not the 70 s decay ramp — ratios stayed messy
  (1.44, 0.76, 0.92, 1.06, 1.01).
- *One-shot anchor to the first received message*: anchors to a possibly
  100 ms-stale message — vehicle 1 froze the full 2 m error at anchor time
  (the message's staleness became the bias).

## Decision (two mechanisms, both fidelity-positive)
1. **Radar extrapolation.** The radar surrogate extrapolates the last
   predecessor state message by its age before use:
   `p ← p + v·age + ½·a·age²`, `v ← v + a·age`, with
   `age = now − msg.stamp` (both nodes share the host clock; stamp set at
   publish). A real radar measures the *current* gap — a stamped-state
   message plus kinematic extrapolation is the honest surrogate, and it
   also removes the ±v0·dt ZOH jitter from the measured error. (The age
   clip started at 50 ms and was later raised to match the dt clamp — see
   [D-014] for why mismatched horizons re-broke this.)
2. **2 s anchor hold.** Each follower re-pins itself to exact desired
   spacing (`x_pos = p_pre − L − (r + h·v)`, `v = v_pre`) on **every tick
   during its first 2 s**, releasing only after all message streams are
   live and fresh — regardless of spawn/discovery order (a one-shot anchor
   can fire on a stale or pre-anchor predecessor state). The leader cruises
   at constant v0 until t = 10 s, so the hold is dynamically benign.

## Verification
After the fix (run `20260706-183232`): quiescent-phase mean errors dropped
from ±2 m to **±2 mm** (std ~2 mm) on all six followers.

## Consequences
- `vehicle_node.py` carries both mechanisms; `_anchor_hold = 2.0` s.
- The startup transient is no longer part of any reported metric; the
  analysis-side detrend (`--baseline-t`) remains as belt-and-braces.
- Follow-on problem (periodic OS stalls) surfaced immediately after and is
  resolved in [D-014].
