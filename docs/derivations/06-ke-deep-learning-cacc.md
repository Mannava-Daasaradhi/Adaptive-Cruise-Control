# Part VI — Ke 2022: CACC by deep reinforcement learning

> H. Ke, *Cooperative Adaptive Cruise Control using V2V Communication and Deep
> Learning*, M.A.Sc. thesis, University of Windsor, May 2022. Ch. 4–5 published
> as H. Ke, S. Mozaffari, S. Alirezaee, M. Saif, "Cooperative Adaptive Cruise
> Control using Vehicle-to-Vehicle communication and Deep Learning," *33rd IEEE
> Intelligent Vehicles Symposium*, June 2022.

**Why this one is in the set.** It is the *learning-based* alternative to the
five model-based papers, it is built in **MATLAB/Simulink** on real hardware
(Quanser QCar, 802.11 WiFi V2V, 2 ms latency), and — read critically — it
demonstrates exactly what you give up when you replace a certified control law
with a trained policy. Be ready to argue both sides.

**Headline results.** CACC reduces the average inter-vehicular distance of ACC
by **44.74 %**; adding DDQN gives a further **40.19 %**.

---

## 6.1 Markov Decision Processes  (§3.1)

```
    M = (S, A, P, R, γ)
    T(s,a,s') = P(s_{t+1}=s' | s_t=s, a_t=a)       transition kernel
    R : S × A → ℝ                                  reward
    γ ∈ [0,1]                                      discount
```

**Why discount at all?** Three independent reasons, and you should be able to
give at least two: (i) it makes the infinite sum `Σγ^tR` converge for bounded
`R`; (ii) it encodes preference for sooner reward; (iii) it makes the Bellman
operator a **contraction** (§6.3), which is what guarantees the algorithms
converge.

### Return and value functions  (eqs. 3.1–3.4)

```
    R = Σ_{t=0}^∞ γ^t R(s_t)                                          (3.1)

    V^π(s) = E_π[ Σ_{t=0}^∞ γ^t R(s_t) | s₀ = s ]                     (3.2)
    Q^π(s,a) = E_π[ Σ_{t=0}^∞ γ^t R(s_t,a_t) | s₀=s, a₀=a ]           (3.4)
```

`V` answers "how good is this state under π"; `Q` answers "how good is taking
*this action* in this state and following π afterwards." The distinction
matters because **control** needs `Q`: to improve a policy you must compare
actions, and `V` alone cannot do that without a model of `P`.

### Bellman equations — derive them  (eqs. 3.7, 3.8)

Split the first term out of the sum and re-index:

```
    V^π(s) = E_π[ R(s₀) + Σ_{t=1}^∞ γ^t R(s_t) | s₀=s ]
           = R(s) + γ E_π[ Σ_{t=0}^∞ γ^t R(s_{t+1}) | s₀=s ]
           = R(s,π(s)) + γ Σ_{s'} T(s, π(s), s') V^π(s')               (3.7) ✓
```

The step `E[Σ_{t=1}γ^tR(s_t)] = γ Σ_{s'}T(s,π(s),s')V^π(s')` is the **Markov
property** doing all the work: conditioned on `s₁ = s'`, the future is
independent of how you arrived. Same argument with the first action pinned:

```
    Q^π(s,a) = R(s,a) + γ Σ_{s'} P(s'|s,a) V^π(s')                     (3.8) ✓
```

and note `V^π(s) = Q^π(s, π(s))`.

### Optimality  (eqs. 3.5, 3.6, 3.9, 3.10)

```
    V*(s) = max_π V^π(s) ,      Q*(s,a) = max_π Q^π(s,a)
    V*(s)   = max_a [ R(s,a) + γ Σ_{s'} P(s'|s,a) V*(s') ]             (3.9)
    Q*(s,a) = R(s,a) + γ Σ_{s'} P(s'|s,a) max_b Q*(s',b)               (3.10)
```

The `max` moving *inside* the expectation in (3.10) — you commit to `a` now,
then act optimally from `s'` — is the dynamic-programming principle. For finite
MDPs an optimal **deterministic stationary** policy always exists, which is why
`π*(s) = argmax_a Q*(s,a)` is well posed.

---

## 6.2 Dynamic programming  (§3.2)

DP assumes the model `(P, R)` is **known** — "a perfect environment." That is
what makes it *model-based*, and what makes it inapplicable to a real vehicle
(you do not have `P` for traffic).

### Policy iteration  (eq. 3.11, Algorithm 1)

```
    π₀ →E V^{π₀} →I π₁ →E V^{π₁} →I π₂ → … → π* →E V*
```

* **Evaluation (E):** solve (3.7) for `V^{π_i}` — a linear system, or iterate
  to convergence.
* **Improvement (I):** `π_{i+1}(s) = argmax_a [R(s,a) + γΣ_{s'}T(s,a,s')V^{π_i}(s')]`
  (3.11).

**Why it terminates.** The *policy improvement theorem* gives
`V^{π_{i+1}} ≥ V^{π_i}` pointwise; there are finitely many deterministic
policies; a strict improvement cannot repeat. So the loop stops after finitely
many iterations, at `π_{i+1} = π_i`, which by (3.9) is optimal.

### Value iteration  (eqs. 3.12, 3.13, Algorithm 2)

Skip explicit evaluation; iterate the Bellman *optimality* backup:

```
    V_{k+1}(s) = max_a [ R(s,a) + γ Σ_{s'} T(s,a,s') V_k(s') ] =: (BV_k)(s)
                                                                      (3.13)
```

### The contraction argument — the thesis states convergence, so prove it

> **Claim.** `B` is a γ-contraction in the sup norm:
> `‖BV₁ − BV₂‖_∞ ≤ γ‖V₁ − V₂‖_∞`.

*Proof.* Fix `s`. Using `|max_a f(a) − max_a g(a)| ≤ max_a |f(a) − g(a)|`:

```
    |BV₁(s) − BV₂(s)| ≤ max_a | γ Σ_{s'} T(s,a,s')(V₁(s') − V₂(s')) |
                      ≤ γ max_a Σ_{s'} T(s,a,s') |V₁(s') − V₂(s')|
                      ≤ γ ‖V₁ − V₂‖_∞ · max_a Σ_{s'}T(s,a,s')
                      = γ ‖V₁ − V₂‖_∞                     (since ΣT = 1)
```

Take the sup over `s`. ∎

By the **Banach fixed-point theorem** on the complete metric space
`(ℝ^{|S|}, ‖·‖_∞)`: `B` has a unique fixed point `V*`, and
`‖V_k − V*‖_∞ ≤ γ^k‖V₀ − V*‖_∞` — geometric convergence at rate γ, from any
initialisation. This is the theorem behind "`V₀ → V₁ → … → V*`" in the thesis.

---

## 6.3 Reinforcement learning: model-free  (§3.3)

RL drops the assumption that `P` is known: "the RL does not have such an ideal
environment available... categorized as a model-free learning algorithm."

### Exploration vs exploitation, ε-greedy  (eq. 3.14)

```
    π(a|s) = argmax_a Q(s,a)   w.p.  1 − ε + ε/|A|
    π(a|s) ≠ argmax_a Q(s,a)   w.p.  ε/|A|   (each non-greedy action)     (3.14)
```

Sanity check the probabilities: `(1 − ε + ε/|A|) + (|A| − 1)(ε/|A|) = 1 − ε + ε = 1` ✓.

The dilemma is fundamental: you can only learn the value of an action by taking
it, but you can only earn reward by taking the best one. ε-greedy is the
crudest resolution that still guarantees every state–action pair is visited
infinitely often — which is exactly the hypothesis Q-learning's convergence
proof needs.

### Monte Carlo  (eqs. 3.15–3.17)

```
    V^π(s) = E_π[G_t | s_t = s] ,     G_t = r_t + γr_{t+1} + γ²r_{t+2} + …
    V^π(s) ← V^π(s) + α( G_{i,t} − V^π(s) ) ,     α = 1/N(s)          (3.17)
```

With `α = 1/N(s)` this is exactly an incremental sample mean. **Unbiased**
(`E[G_t] = V^π(s)` by definition) but **high variance** (a whole trajectory's
randomness enters each sample), and **episodic only** — you must wait for
termination to know `G_t`.

### Temporal difference  (eqs. 3.18–3.19)

Replace the sampled return `G_t` with the **bootstrapped** one-step estimate:

```
    V^π(s) ← V^π(s) + α[ r_t + γV^π(s_{t+1}) − V^π(s) ]                (3.18)
    δ_t := r_t + γV^π(s_{t+1}) − V^π(s_t)          "TD error"          (3.19)
```

**The bias–variance trade you must be able to state:**

| | Monte Carlo | TD(0) | DP |
|---|---|---|---|
| samples? | yes | yes | no (uses model) |
| bootstraps? | no | **yes** | yes |
| bias | none | **biased** (V is wrong early) | none |
| variance | high | **low** | none |
| needs episode end? | yes | **no** | no |

TD "combines Monte Carlo ideas and dynamic programming ideas" — sampling from
MC, bootstrapping from DP. That sentence is the standard exam answer.

### SARSA vs Q-learning  (eqs. 3.20–3.22, Algorithm 3)

```
    SARSA (on-policy):
        Q(s_t,a_t) ← Q(s_t,a_t) + α( r_t + γ Q(s_{t+1}, a_{t+1}) − Q(s_t,a_t) )   (3.20)

    Q-learning (off-policy):
        Q(s_t,a_t) ← Q(s_t,a_t) + α( r_t + γ max_{a'} Q(s_{t+1}, a') − Q(s_t,a_t) ) (3.22)
```

The **only** difference is what fills the next-action slot: SARSA uses the
action actually taken by the behaviour policy (so it learns the value of the
ε-greedy policy *including its exploration mistakes*); Q-learning uses the
greedy maximum (so it learns `Q*` regardless of how it explored).

Practical consequence the thesis names: "SARSA has a better performance where
many negative rewards exist, and Q-learning has a better performance during the
early convergence." The classic illustration is the cliff-walking problem —
SARSA walks the safe path because it accounts for the chance of exploring off
the cliff; Q-learning walks the optimal edge and falls off during training.

**For CACC this matters**: the reward table (§6.6) has large negative values
(−500) for collisions. SARSA's conservatism would arguably suit safety-critical
training better. The thesis chooses Q-learning; that is a defensible but
questionable choice, and a fair thing to raise.

### Convergence conditions  (eq. 3.23)

Q-learning converges w.p. 1 if every state–action pair is visited infinitely
often, rewards are bounded (`|r_n| ≤ R`), and the step sizes satisfy the
**Robbins–Monro** conditions:

```
    Σ_{t=1}^∞ α_t = ∞          and        Σ_{t=1}^∞ α_t² < ∞           (3.23)
```

**Why each condition:**
* `Σα_t = ∞` — the total "distance" the iterates can travel is unbounded, so
  any initialisation error can be overcome. (With `Σα_t < ∞` the iterate is
  trapped within a finite ball of its start.)
* `Σα_t² < ∞` — the accumulated *variance* injected by the noise is finite, so
  the iterates settle rather than rattling forever.

`α_t = 1/t` satisfies both; `α_t = const` satisfies the first but not the
second — which is why constant-α methods track but do not converge (and why
constant α is nevertheless standard in deep RL, where the environment is
non-stationary anyway).

---

## 6.4 Double Q-learning and maximisation bias  (Algorithm 4)

This is the most important derivation in Part VI, and the thesis states the
algorithm without justifying it.

### The bias

Q-learning's target contains `max_{a'} Q̂(s',a')`, where `Q̂` is a **noisy
estimate**. By Jensen's inequality applied to the convex function `max`:

```
    E[ max_a Q̂(s',a) ]  ≥  max_a E[ Q̂(s',a) ]
```

So even with *unbiased* per-action estimates (`E[Q̂(s',a)] = Q(s',a)`), the
target is **systematically too large**. The inequality is strict whenever the
estimates have nonzero variance and more than one action is near-optimal.

Intuition: the `max` operator selects whichever action's estimate happened to
be over-estimated by noise. It is a "winner's curse" — the argmax is biased
toward the luckiest estimate, and you then *use that lucky value*. The bias
compounds through bootstrapping: an inflated target inflates `Q̂`, which
inflates the next target.

### The fix: decouple selection from evaluation

Maintain two independent estimates `Q₁, Q₂`, and on each update pick one at
random (probability ½) to update, using the **other** to evaluate the action the
**first** selected:

```
    a* = argmax_{a'} Q₁(s', a')
    Q₁(s,a) ← Q₁(s,a) + α( r + γ Q₂(s', a*) − Q₁(s,a) )
```
and symmetrically with the roles swapped.

**Why it removes the bias.** `a*` is chosen using `Q₁`, but evaluated using
`Q₂`, which is *independent* of `Q₁`. Hence
`E[Q₂(s', a*)] = Q(s', a*) ≤ max_a Q(s',a)` — the estimate of the selected
action is unbiased, so there is no systematic upward pressure. (It can now
*under*-estimate, which is the accepted trade.)

> *Note on the thesis' rendering:* its Algorithm 4 shows
> `Q₁(s,a) ← Q₁(s,a) + α(r + Q₁(s', argmax_{a'}Q₂(s',a')) − Q₁(s,a))`, i.e.
> `Q₂` selecting and `Q₁` evaluating, with no `γ`. Van Hasselt's canonical form
> is the one above (`Q₁` selects, `Q₂` evaluates, discount present). Use the
> canonical form; the *principle* — selection and evaluation must use
> independent estimators — is what matters and is identical either way.

---

## 6.5 Function approximation, neural networks, DQN  (§§3.4, 3.5)

### Why tabular fails

"For a huge data subset or complex sensations, it is nearly impossible to have
all the information in a table. Besides the issue of the need for memory,
**generalization**... is the most significant issue." The CACC state
`S = {H, d_diff, a_lead}` is continuous — a table would need discretisation,
and would learn nothing about a bin it has never visited.

Value function approximation: `V^π(s) ≈ V̂(s; w)`, `Q(s,a) ≈ Q̂(s,a; w)`.

### The network and its cost  (eqs. 3.24, 3.25)

```
    J(W,b;x,y) = ½‖h_{W,b}(x) − y‖²                                    (3.24)

    J(W,b) = [ (1/m)Σ_{i=1}^m J(W,b;x^{(i)},y^{(i)}) ]
             + (λ/2) Σ_l Σ_i Σ_j (W^{(l)}_{ij})²                        (3.25)
             ╰──────── weight decay (L2 regularisation) ───────────╯
```

The regulariser "tends to decrease the magnitude of the weights and helps
prevent overfitting"; λ trades fit against smoothness. Note the **biases are
not regularised** — standard practice, since penalising them only shifts the
function without controlling its complexity.

### Backpropagation — derive it  (eqs. 3.27–3.30)

With `z^{(l+1)} = W^{(l)}a^{(l)} + b^{(l)}`, `a^{(l+1)} = f(z^{(l+1)})`, define
`δ^{(l)}_i := ∂J/∂z^{(l)}_i`.

**Output layer** (`l = n_l`):

```
    δ^{(n_l)}_i = ∂/∂z^{(n_l)}_i [ ½‖h_{W,b}(x) − y‖² ]
                = (a^{(n_l)}_i − y_i) · f'(z^{(n_l)}_i)
                = −(y_i − a^{(n_l)}_i) f'(z^{(n_l)}_i)                 (3.27) ✓
```

**Hidden layers** (chain rule through every unit `j` of layer `l+1`):

```
    δ^{(l)}_i = Σ_j (∂J/∂z^{(l+1)}_j)(∂z^{(l+1)}_j/∂z^{(l)}_i)
              = Σ_j δ^{(l+1)}_j · W^{(l)}_{ji} · f'(z^{(l)}_i)
              = ( Σ_j W^{(l)}_{ji} δ^{(l+1)}_j ) f'(z^{(l)}_i)         (3.28) ✓
```

**Parameter gradients** (since `∂z^{(l+1)}_i/∂W^{(l)}_{ij} = a^{(l)}_j` and
`∂z^{(l+1)}_i/∂b^{(l)}_i = 1`):

```
    ∂J/∂W^{(l)}_{ij} = a^{(l)}_j δ^{(l+1)}_i                            (3.29)
    ∂J/∂b^{(l)}_i    = δ^{(l+1)}_i
    ⟹  ∇_{W^{(l)}}J = δ^{(l+1)}(a^{(l)})ᵀ  ,  ∇_{b^{(l)}}J = δ^{(l+1)} (3.30) ✓
```

Update with weight decay (eq. 3.31):

```
    W^{(l)} := W^{(l)} − α[ (1/m)ΔW^{(l)} + λW^{(l)} ]
    b^{(l)} := b^{(l)} − α[ (1/m)Δb^{(l)} ]
```

Note the **random initialisation** requirement: "the purpose of random
initialization is symmetry breaking, which means to prevent all the hidden
layer units learn the same function of the inputs." With identical
initialisation every hidden unit in a layer receives an identical gradient
forever, and the layer collapses to width one.

### DQN's two innovations  (eqs. 3.32, 3.33, Algorithm 5)

Naively plugging a network into Q-learning diverges. Two problems and two
fixes:

**Problem 1 — correlated samples.** Consecutive transitions along a trajectory
are highly correlated, violating the i.i.d. assumption SGD relies on.
**Fix — experience replay.** Store transitions `(s,a,r,s')` in a buffer `D` and
train on random minibatches. Also improves sample efficiency ("it can use the
data more than once for training").

**Problem 2 — non-stationary target.** In
`Δw = α(r + γ max_{a'}Q̂(s',a';w) − Q̂(s,a;w))∇_wQ̂(s,a;w)` (3.32), the target
depends on the very weights being updated — the network chases its own tail.
**Fix — fixed Q-targets.** Keep a frozen copy `w⁻`, updated every `C` steps:

```
    Δw = α( r + γ max_{a'} Q̂(s',a'; w⁻) − Q̂(s,a; w) ) ∇_w Q̂(s,a; w)    (3.33)
```

Algorithm 5 assembles both: ε-greedy action → store transition → sample
minibatch → `y_i = r_i` if terminal else `r_i + γ max_{a'}Q̂(s_{i+1},a';w⁻)` →
gradient step on `(y_i − Q̂(s_i,a_i;w⁻))²` → every `C` steps `w⁻ ← w`.

**DDQN** = DQN + the §6.4 decoupling, using the online net to select and the
target net to evaluate:

```
    y_i = r_i + γ Q̂( s_{i+1}, argmax_{a'} Q̂(s_{i+1}, a'; w) ; w⁻ )
```

The two networks DQN already maintains are reused, so DDQN costs nothing extra
— which is why it is a strict default improvement.

---

## 6.6 The CACC application  (§4)

### Hardware

Quanser **QCar**: NVIDIA Jetson TX2, 2-D lidar (8000 samples/s, ≤15 Hz, 18 m
range), 360° CSI camera suite, 802.11 a/b/g/n/ac WiFi, 720-count encoder.
Modelled and driven from **MATLAB Simulink** via the QUANSER blockset.

Practical details worth noting because they are the kind of thing that bites in
real builds:
* Lidar zero-heading is 270° off the vehicle's; corrected by adding 270°.
* Only **400 samples per revolution** collected, to hold 15 Hz scan rate — a
  deliberate resolution-for-latency trade.
* The 0.26 m lidar-to-bumper offset is subtracted.
* Nine measurements around zero heading are taken and the **minimum** used,
  "because the minimum value of these measurements can make the Q-Car operate
  based on the safest strategy"; zeros (no return) are excluded.
* **2.4 GHz** chosen over 5 GHz: "the 5 GHz band will have a better
  transmission rate but lower reliability due to the signal degradation."
  Reliability over throughput — the right call for a control loop.

### The ACC law  (eqs. 4.2, 4.3)

```
    d_r(t) = r + h v(t)                              CTH policy (F-4)  (4.2)

                    V_nominal
    V_Desired = ───────────────────── ( d_obstacle − d_stop )          (4.3)
                 d_tracking − d_desired
```

A **proportional speed law**: desired speed rises linearly with the excess of
measured distance over the stopping distance, saturating at `V_nominal`. This
is a distance→speed controller, not an acceleration controller — a different
architecture from every other paper here (all of which command acceleration).

### The CACC law  (eqs. 4.5, 4.6)

```
    ω = V_front − V_ego                                                (4.5)

                              (d_obstacle − r)
    V_Desired = V_nominal · ─────────────────                          (4.6)
                                  h · ω
```

**Critique this — it is the obvious question.** `ω` sits in the *denominator*,
so as the platoon settles into steady following (`ω → 0`) the desired speed
diverges. The thesis patches it with a mode switch: "the follower vehicle would
follow the same speed as the front vehicle while the inter-vehicular distance
is within ±10 % range of the desired distance **or the ω equals zero**."

Consequences you should be able to name:
1. The closed loop is **not smooth** — it is a switched system, and no
   stability argument is offered for the switching.
2. `V_Desired` is **not dimensionally a speed** unless the constants absorb it:
   `(m)/(s · m/s) = 1`, so `V_nominal × dimensionless` — it does work out, but
   only because `h·ω` has units of metres. Worth checking on the board.
3. **There is no string-stability analysis anywhere in the thesis.** Only two
   vehicles are ever considered (one lead, one ego). Every claim is about
   single-follower distance keeping.

### RL formulation  (eqs. 4.7–4.14)

**State** (three continuous variables):

```
    H = (S_lead − S_ego)/V_ego            headway time                 (4.8)
    V_diff = V_lead − V_ego                                            (4.9)
    ḋ_diff = V_diff                                                    (4.10)
    S = { H, d_diff, a_lead }                                          (4.11)
```

Using **headway time** rather than raw distance is the good modelling decision
in this thesis: it normalises the state by speed, so the learned policy
transfers across operating speeds. It is the same insight that makes CTH work
(F-4/F-5) — expressed here as a feature choice rather than a control law.

**Actions** (five, discrete):

```
    A = { AC_a, AC_s, DA_a, DA_s, NO }                                 (4.13)
```

aggressive/smooth accelerate, aggressive/smooth decelerate, no-op. Discretising
the action space is what makes DQN applicable at all — Q-learning needs a
finite `argmax`. A continuous-action method (DDPG, SAC) would avoid the
quantisation but is not attempted.

**Reward** (zone table, Fig. 4.13, over `H` × `V_diff`):

```
    H > 1.5 s              "Too Far"      −50
    0.33 < H < 1.5         "Close up" +1  /  "Far away" −3   (sign of V_diff)
    0.315 < H < 0.33       "Good Range"   +3
    0.285 < H < 0.315      "Best Range"   +25      (±5 % of h = 0.3 s)
    0.27 < H < 0.285       "Good Range"   +3
    0.06 < H < 0.27        "Closed" −3   /  "Close up" +1
    H < 0.06               "Too Close"    −50
    r = −500     if  d_obstacle > 5 m  or  d_obstacle < 0.2 m          (4.14)
```

Design points to note:
* `h = 0.3 s` — **not** a real-world headway. "The pre-defined headway time is
  0.3 seconds, which is ten times smaller than the pre-defined headway time in
  real-world (3 s), because the length of the Q-Car is 0.425 m, which is around
  ten times smaller than a general Sedan (4 m)." A **geometric scaling
  argument**, not a dynamic-similarity one. Strictly, dynamic similarity for a
  1:10 length scale would need time scaled by `√10 ≈ 3.16`, not 10 (Froude
  scaling). The chosen scaling is defensible for a distance-keeping demo but
  would not preserve the τ-to-h ratio that every result in Parts II–IV depends
  on.
* The reward is **shaped and asymmetric**: `V_diff` sign is used to reward
  *closing* when far and *opening* when close, which supplies gradient
  information the sparse ±50 terms would not. Standard and sensible.
* The `−500` terminal penalty on `d > 5 m` (out of lidar range) or `d < 0.2 m`
  makes those absorbing failure states.

**Network:** four layers — input, two hidden layers of **120 ReLU units** each,
output layer with one Q-value per action. Trained with the MATLAB Reinforcement
Learning Toolbox inside the Simulink loop (their Fig. 4.16), with the lead
car's initial position randomised over 10–11 m against a fixed ego at 9.6 m, so
each episode starts with a gap between 0.4 m and 1.4 m.

---

## 6.7 Critical appraisal — the argument to be able to make both ways

### What the learned controller buys

* **No model needed.** Zero knowledge of `τ`, the lidar's noise, or the WiFi
  latency is used in the design. Ma, Ploeg and Köroğlu all need `τ` (and
  Köroğlu needs `δ̄`) to even state their results.
* **Objective shaping is trivial.** "Stay in the best range, never crash, be
  smooth" is three lines of a reward table. Encoding the same preferences in an
  H∞ weighting function `W_e(s)` (Part III §3.6) is a research task.
* **It works on real hardware.** 44.74 % + 40.19 % distance reduction, measured
  on QCars over real WiFi, not simulated.
* **Nonlinearity and saturation come free.** The learned policy is not
  restricted to a linear law and never leaves the actuator envelope, because it
  only ever selects from five feasible actions.

### What it gives up — and this is the decisive list

1. **No string-stability guarantee, and no way to obtain one.** There is no
   `Γ(s)`, so `‖Γ‖_{H∞} ≤ 1` cannot even be *stated*. Only two vehicles are
   ever simulated or tested. A platoon of learned followers could amplify
   disturbances geometrically (F-7) with nothing in the training signal to
   penalise it — the reward is *per-vehicle* and local.
2. **No robustness certificate.** Ma certifies over `τ ∈ (0,τ₀]` and
   `k̃a ∈ I`; Köroğlu over `δ ∈ [0, δ̄τ]`; Zhao over `|M| ≤ F`. A trained policy
   is certified over *the distribution it was trained on* and nothing else.
3. **No internal-stability check.** Routh–Hurwitz (F-14) has no analogue.
4. **The switched control law is unanalysed** (§6.6).
5. **Sample-inefficient and unsafe to train online.** The −500 penalty means
   the agent learns collision avoidance *by colliding* in simulation. That
   sim-to-real gap is the load-bearing assumption of the whole approach.
6. **Not compositional.** Adding a vehicle, changing `h`, or changing `τ`
   requires retraining. Ma's `h_lb(k_a, ρ)` is a formula you evaluate.

### The synthesis worth stating

**These are not competitors; they answer different questions.** The model-based
papers answer *"is this design safe for every admissible platoon, channel and
lag?"* — a certification question. The learning paper answers *"what policy
performs best on the distribution I have?"* — an optimisation question.

The defensible architecture is **learning inside a certified envelope**: a
model-based law with proven `‖Γ‖_{H∞} ≤ 1` as the backbone, with learning
tuning the parameters the theory leaves free — exactly the structure of this
project's QoS-adaptive pipeline (D-016/D-017), where an online channel
estimator drives headway and gain adaptation but **every configuration
traversed is itself certified** by evaluating `‖H̃‖_∞` at the in-force
parameters (D-018, the quasi-static argument in `docs/theory/T-08`).

If asked "should we use RL for CACC?", that is the answer: use it to *choose*
among certified designs, not to *replace* certification.

---

## 6.8 Whiteboard drill

1. Write the MDP tuple; give two reasons for discounting.
2. Derive the Bellman equation for `V^π` from the definition of the return,
   naming where the Markov property is used.
3. Write both Bellman optimality equations.
4. **Prove the Bellman optimality operator is a γ-contraction in `‖·‖_∞`** and
   state what Banach's theorem then gives.
5. Contrast MC / TD / DP on: sampling, bootstrapping, bias, variance.
6. Write SARSA and Q-learning side by side; state the on/off-policy difference
   and when each is preferable.
7. State the Robbins–Monro conditions and justify each separately.
8. **Prove that `E[max_a Q̂] ≥ max_a E[Q̂]`** and explain how double Q-learning
   removes the resulting bias.
9. Derive the backpropagation recursion `δ^{(l)}` and the parameter gradients.
10. Name DQN's two innovations and the specific failure each prevents.
11. Write the CACC speed law (4.6) and identify its singularity.
12. Give three things this approach cannot certify that Parts II–V can.

---

*Next: [Part VII — cross-paper synthesis and the exam Q&A bank](07-synthesis-and-drills.md).*
