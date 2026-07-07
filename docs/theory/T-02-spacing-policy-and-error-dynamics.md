# T-02 — Spacing policy, error signals, and the sign map to Ma 2025

**Code:** `src/cacc/controllers.py` (definitions), `src/cacc/platoon.py`
(`_deriv` computes them per follower).

## Constant time-headway (CTH) policy

Front-bumper positions p, vehicle length L, standstill gap r, headway h:

    d_i      = p_{i−1} − p_i − L          actual bumper-to-bumper gap
    d_des,i  = r + h · v_i                desired gap (speed-dependent)
    e_i      = d_i − d_des,i              spacing error
    ė_i      = v_{i−1} − v_i − h · a_i    (measurable: radar + own IMU)
    dv_i     = v_{i−1} − v_i              relative speed (radar)

CTH is what makes small headways *possible*: the desired gap grows with
speed, which injects the damping term −h·a_i into the error dynamics.

## Sign map to the base paper (D-005)

Ma 2025 use point masses with spacing constant d and error δ_i with the
opposite orientation. With **d = r + L = 5 m** (our r = 1, L = 4):

    e_i = −δ_i

All reproduction plots show δ_i = −e_i so curves match the paper's
orientation. The choice r=1/L=4 (vs r=5/L=0) preserves the physical
length needed by the throughput metric and the visualizations.

## Why measured-e matters more than true-e (ROS backend)

In the distributed backend the controller regulates the *measured* error
(radar surrogate = predecessor state message extrapolated by its stamp
age, D-013/D-014). Any systematic measurement bias shifts the equilibrium
gap but not the recorded e — which is why the analysis detrends by the
pre-maneuver mean and why startup/stall artifacts were diagnosed entirely
in measured-e space (see the spike-forensics in D-014).

## Time-varying h (adaptive runs)

With h = h_i(t) (D-016), e_i uses the *instantaneous* h_i; a step Δh at
speed v instantly shifts e by −v·Δh (the commanded fallback), which the
loop then executes physically. Two consequences documented elsewhere:
- transition waves of amplitude ≈ v·Δh are *commanded*, not instability
  (interpretation caveat in `09_…` §4);
- ė gains a −ḣ·v term, negligible at the 0.05 s/s slew limit
  (0.05 × 20 = 1 m/s² worst-case equivalent — bounded by the rate limit
  and inside the margin; part of the quasi-static argument, T-08).

## Equilibrium

At cruise (v_i = v0, a = 0, e = 0) the platoon length is
N·(L + r + h·v0); at v0 = 20 m/s, each 0.1 s of headway costs 2 m per
vehicle — the direct link between h and road capacity used in the
capacity claims (`metrics.throughput_veh_per_hour`).
