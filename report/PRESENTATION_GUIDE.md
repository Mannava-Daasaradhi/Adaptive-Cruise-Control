# Presentation guide — the only thing you need to read

Companion to `CACC_Control_Systems_Presentation.pptx` (20 slides).
Written for someone who has never taken a control course.

**Part 0** the story in a page · **Part 1** every term in plain words,
with an everyday comparison for each · **Part 2** the nine sentences on
the slides that are genuinely hard, worked through slowly · **Part 3**
the 20 slides, one at a time · **Parts 4–5** what to drop if you run out
of time, and where everything lives · **Part 6** how to actually run the
simulation · **Part 7** every symbol — what it is called, how it is
calculated, the value it takes, and the file that computes it.

If you only have 10 minutes: Part 0, then the decoder ring at the top of
Part 1. If something on a slide still does not click, it is almost
certainly one of the nine in Part 2. If you want to *run* something, jump
to Part 6. If someone points at a symbol on a slide and asks what it is
and where the number came from, that is Part 7.

---

## Part 0 — the story, in one page

> Cars on a highway waste space — not because they drive slowly, but
> because they leave big gaps. Shrink the gaps safely and the same road
> carries more traffic.
>
> The obvious way to shrink them is to let cars talk by radio: the car in
> front says "I'm braking", and the car behind brakes **before** it sees
> the gap close. That is **CACC**.
>
> The trap: if every car reacts slightly *more* than the one in front, a
> small wobble at the head grows as it travels backwards, and by car 200
> it is a full stop. That is a **phantom traffic jam**, and it is what
> "**string instability**" means. The growth per car can be a fraction of
> a percent — invisible on a video of the convoy.
>
> A 2025 IEEE paper (Ma, Pagilla & Darbha) works out exactly how small
> the gaps can be before this happens, given how noisy the radio is. It
> gives a clean formula.
>
> **We did three things.** (1) Rebuilt that paper from scratch until our
> numbers matched its printed numbers to four decimals — that is our
> credibility. (2) Showed its assumptions do not survive a real road: it
> assumes the radio has no delay, that its noise level is known and never
> changes, and that no car ever lies. (3) Built and measured the fixes —
> the convoy now measures the radio quality itself, opens its gaps
> *before* entering a bad zone, refuses to trust a car whose radio
> disagrees with the radar, and reports a **calibrated risk number**
> instead of a bare pass/fail.
>
> All of it checked three ways — a Python core, a distributed ROS 2 system
> with one OS process per car, and an independent MATLAB/Simulink model —
> agreeing to one part in ten thousand.

**If you get exactly one sentence:**

> A published control law says how close self-driving cars can safely
> follow each other over a noisy radio. We reproduced it exactly, showed
> its assumptions each fail measurably on a real road, and built and
> certified the fixes.

---

## Part 1 — every term, in plain words

Each entry has the same three lines:

- **Plain words** — what it actually means, no jargon, usually with an
  everyday comparison.
- **Its job** — why it exists in this project.
- **Without it** — what breaks. This is usually the fastest way to
  understand something.

### The decoder ring

If a word on a slide looks frightening, this is what it really means.
Full entries below.

| The scary word | What it actually means |
|---|---|
| double integrator | the pedal controls acceleration, which slowly builds speed, which slowly builds position — two slow stages between you and where the car is |
| transfer function | a formula answering "wobble of *this* rhythm goes in — how big is the wobble that comes out?" |
| frequency domain | testing the convoy against every *rhythm* of wobble separately, instead of watching one video |
| H-infinity norm, ‖H̃‖∞ | test every rhythm, write down how much each grows, take **the worst one** |
| string stability | does a wobble get **bigger or smaller** as it passes back down the line? |
| low-pass filter | lets slow changes through, smooths out fast ones — like car suspension |
| actuator lag, τ | the moment between pressing the pedal and the car actually responding |
| feedforward | acting on a warning **before** the problem arrives, instead of reacting after |
| SNR, ρ | how clean the radio is — like bars on a phone |
| bounded support | the error is never worse than a known amount — a scale that's 5 % off, never 50 % |
| quantile, Q₉₉ | line the readings up smallest to largest, take the one 99 % of the way up |
| conservative by construction | when it's wrong, it's always wrong in the **safe** direction, by maths not luck |
| sufficient, not necessary | pass ⟹ definitely safe. Fail ⟹ *maybe* still fine. A safe test, but a harsh one |
| chance-constrained | designing to a stated risk ("I accept 1 in 100") instead of to the worst thing imaginable |
| exceedance | the chance of ending up on the wrong side of the line |
| Monte-Carlo | run it many times with different random draws and count what happens |

---

### A — the everyday picture

#### platoon

**Plain words.** A line of cars driving as one unit, each following the
one directly in front. Here: one lead car and **12 followers**.

**Its job.** It is the thing being studied. Every claim in the deck is
about a platoon, not about a single car.

**Without it.** You would be studying one car following another, which is
a solved problem. All the difficulty comes from the fact that there is a
*chain*, and effects pile up along it.

---

#### string stability — **the central idea of the whole project**

**Plain words.** Imagine a line of people holding hands, and the person at
the front gives a tug. If each person tugs the next one *slightly harder*
than they were tugged, then by the twentieth person someone gets yanked
off their feet — even though every individual only added a tiny bit.

Same with cars. If each car brakes slightly harder than the car in front
did, the braking gets worse and worse down the line until someone at the
back has to stop dead. **String stability** is the property that this
does *not* happen — that a wobble **shrinks** as it travels backwards
instead of growing.

**Its job.** It is the property being proved, measured and certified
everywhere in this deck.

**Without it.** You get a **phantom traffic jam**: cars stopping on an
open motorway for no visible reason. It is one of the real causes of
everyday congestion.

---

#### why it is invisible — the single most important fact in the deck

**Plain words.** The growth per car can be tiny — say each car makes the
wobble **0.35 % bigger**. Over 11 cars that is a 4 % increase: you would
never notice it. But growth **multiplies**, so over 200 cars the same
0.35 % becomes a factor of **two**.

**Its job.** It is why we never judge stability by watching a simulation
video, and why the whole deck insists on a numerical verdict instead.

**Without it** — i.e. if you forget this — you look at an animation,
see nothing alarming, and conclude the design is fine. It isn't.

---

#### predecessor following

**Plain words.** Each car watches and listens to **only** the car
directly in front. It doesn't hear the leader, and it doesn't hear its
neighbours' opinions.

**Its job.** It is the cheapest and most realistic arrangement — no
roadside infrastructure, no complicated networking.

**Without it.** Other arrangements make stability *easier*, but they also
give you something we deliberately don't have: a second opinion. Because
each car has only one source of radio information, **there is nobody to
take a vote with** — which is exactly why a lying car cannot be filtered
out by majority rule, and why we had to invent a different defence.

---

### B — the car itself

#### τ, τ₀ — *say "tau" and "tau-nought"*

**Plain words.** Press the accelerator and the car does **not** speed up
instantly. The engine has to build up torque, the driveline takes up
slack, the brakes build pressure. τ is that hesitation — about **half a
second**. τ₀ is the slowest car we're willing to promise anything about.

**Its job.** It is the one number standing in for the entire mechanical
sluggishness of a car. Because we certify for a *range* (anything up to
τ₀), one design covers a mixed fleet of different vehicles.

**Without it.** Pretend the car responds instantly and the maths gets
easier — and the answer comes out **wrong in the dangerous direction**:
it tells you that you can drive closer than you really can. And if you
pin τ to one exact number instead of a range, your guarantee covers
**one specific car**; put a slightly different one in the convoy and
you're outside the proof.

---

#### "double integrator" — why a car is sluggish to reposition

**Plain words.** You control the pedal. But the pedal only sets
**acceleration**. Acceleration slowly builds up **speed**, and speed
slowly builds up **position**. So the thing you actually care about —
where the car is — is **two slow stages away** from the thing you
control. Stacking two of these build-ups is what "double integrator"
means.

Think of steering a large ship: you turn the wheel, which slowly changes
the rate of turn, which slowly changes the heading.

**Its job.** It is *why this problem is hard.* This two-stage sluggishness
is precisely what makes a fixed-distance convoy impossible to stabilise.

**Without it.** With a simpler, more responsive vehicle, cars could safely
hold a fixed gap and this entire field of research wouldn't exist.

---

#### saturation

**Plain words.** A real car cannot accelerate at 20 m/s². We clip every
command to what a real car can do: **−8 to +3 m/s²** (harder braking than
accelerating, as in reality).

**Its job.** Keeps the simulation physical.

**Without it.** The simulation produces beautiful recoveries that no real
car could perform. There's also a subtler point: once a car is flat out,
all the neat linear maths stops applying — which is exactly why our final
safety check measures the **actual gap** in a full nonlinear simulation
rather than trusting the formula.

---

### C — the gap rule

#### h_w — the time headway. **This is the answer the whole project is about.**

**Plain words.** The "two-second rule" from driving school. You keep a gap
equal to however far you travel in h_w seconds. Drive faster, the gap
automatically grows.

At motorway speed, **every 0.1 s of headway costs 2.5 metres per car.**

**Its job.** Every result in the deck is a statement about how *small*
h_w is allowed to be while the convoy stays safe. Smaller headway = more
cars per lane = more road capacity.

**Without it.** If h_w = 0 you'd keep a *fixed* gap regardless of speed —
and for cars with the two-stage sluggishness above, safe convoying at a
fixed gap is **mathematically impossible**. Not merely difficult:
impossible. Time headway is the cheapest known fix, and its price is
exactly the road space we were trying to save.

---

#### r — the standstill gap

**Plain words.** The metres you leave at a red light. Here 1 m of clear
space behind a 4 m car, so 5 m nose-to-nose.

**Its job.** Almost nothing mathematically — it's a constant, so it drops
out of every stability calculation. It only matters when counting how
much road the convoy occupies.

**Without it.** Cars touch at a red light. Nothing about stability
changes. Worth knowing precisely because people assume it's the safety
number — it isn't. The safety-relevant gap at speed is `h_w × speed`.

---

#### δ (delta) — the spacing error

**Plain words.** How far a car is from where the gap rule says it should
be. Zero when everything is calm; non-zero the moment anyone brakes or
accelerates.

**Its job.** It is the wobble that travels down the line. When we ask "does
the disturbance grow?", δ is the disturbance.

**Without it.** Nothing to propagate — string stability is *by definition*
a statement about how δ passes from car to car.

---

#### capacity

**Plain words.** How many cars per second a lane can carry. It depends on
gaps, not on speed.

**Its job.** It converts a control-theory result into a reason to care:
12 cars at 0.95 s headway occupy **345 m**; at 0.88 s they occupy
**324 m** — about **6 % more traffic** on the same road.

**Without it.** You have a stability result with no stated benefit, and no
answer to "so what?"

---

### D — the controller: three knobs

The follower computes one number — how hard to accelerate or brake — by
adding up three contributions. Those three, and how strongly each one
counts, are the whole controller.

#### ACC, CACC, CTHP — the three control laws

**Plain words.**
- **ACC** is ordinary adaptive cruise control: radar only, no radio.
- **CACC** adds a radio link to the car in front.
- **CTHP** is the specific CACC law this paper studies.

**Its job.** CTHP is what the deck analyses; ACC is the baseline it
improves on.

**Without the radio (i.e. plain ACC).** The minimum safe headway
**doubles**. That doubling is the entire prize.

---

#### k_p — the "get back where you belong" knob

**Plain words.** How hard the car pulls itself back when the gap is wrong.
Like a spring: the further off you are, the harder it pulls.

**Without it.** The car never corrects a gap it didn't intend to have — it
drifts away and never returns.

---

#### k_v — the "stop rushing at them" knob

**Plain words.** How hard the car reacts to *closing speed* — not the gap
itself, but how fast the gap is shrinking.

**Without it.** The gap swings back and forth. k_p on its own is a spring
with no shock absorber: the car overshoots, comes back, overshoots again.

---

#### k_a — the radio knob. **The one thing CACC adds over ACC.**

**Plain words.** The two knobs above are **reactive** — the gap has to go
wrong before they do anything. k_a is **predictive**: the car in front
broadcasts "I am braking at 2 m/s²", and you start braking *immediately*,
before the gap has even begun to shrink.

The everyday version: brake lights tell you something happened; a radio
message tells you what is *about* to happen.

**Its job.** It **halves the minimum safe headway.**

**Without it.** Set k_a = 0 and you have plain ACC again, with double the
headway. And note the irony the whole paper rests on: **the one signal
that buys you the improvement is the one that has to travel over an
unreliable radio.**

---

### E — the radio, and what's wrong with it

#### w — the fudge factor

**Plain words.** The radio doesn't deliver the number exactly. It delivers
it **multiplied by a random factor near 1** — sometimes 0.9, sometimes
1.1. The car has no way to know which.

**Its job.** It is the model of a noisy link, and it's the entire source
of difficulty.

**Without it** (w always exactly 1). The radio is perfect, the paper
collapses to a trivial case, and there is **no problem left to solve**.

---

#### bounded support — |w − 1| ≤ 1/ρ

**Plain words.** The fudge factor can be wrong, but **never wronger than a
known amount**. Like a kitchen scale that might read 5 % off but can never
read 50 % off.

**Its job.** This is the property that makes a *hard promise* possible at
all. Because the error can't exceed a known limit, you can design for the
very worst point in that range and genuinely be safe.

**Without it.** If you modelled the noise as the usual bell curve, there
would always be some tiny chance of an arbitrarily bad reading — and then
**no gap, however large, could ever be guaranteed safe.** The whole
deterministic half of the paper would evaporate.

It's also the property that makes our online estimator provably safe —
see ρ̂ below.

---

#### ρ (rho) — the signal-to-noise ratio

**Plain words.** How clean the radio is, as a single number. Like bars on
a phone. Big ρ = crisp link. ρ close to 1 = the broadcast number is
basically noise. In the reproduction, ρ = 5.

**Its job.** It's the one dial that describes radio quality, and it sets
how big the fudge factor can be.

**Without it.** With no limit on the fudge factor, nothing can be
certified at all. **ρ is the parameter the paper assumes you know and
that never changes — and it's the assumption every one of our extensions
attacks.**

---

#### k̃_a (k-a-tilde) — what the radio *actually delivered*

**Plain words.** You intended to use knob setting k_a. After the radio
mangled the number, you effectively used something else. The tilde means
"the corrupted version".

**Its job.** The gap between what you intended and what you got **is** the
paper.

**Without it.** No gap, no paper. Concretely: the channel here is
**biased optimistic by 4.8 %** — on average the number arrives slightly
*larger* than the truth, which is the unhelpful direction.

---

#### 16 coin flips — how w is built

**Plain words.** The paper builds the fudge factor from **16 independent
coin flips**, each contributing a smaller amount than the last — like the
binary digits of a number.

**Its job.** It's a realistic model of a corrupted digital transmission.
It's also an unexpected gift: 16 coin flips means there are exactly
**65 536 possible outcomes**, few enough that we can list every single
one — which is what our risk certificate is built on.

**Without it.** You'd have to approximate the distribution instead of
computing it exactly.

---

#### θ (theta) — communication delay. **Our addition; the paper assumes zero.**

**Plain words.** By the time the broadcast arrives and gets used, it is
already **out of date**. The car in front is no longer doing what it said.

**Its job.** It's the most important gap we found in the paper.

**Without accounting for it.** The convoy goes unstable at a delay of
about **40 milliseconds** — far too small for anyone to think to check.
The headline finding of the whole project: **delay, not noise, is what
actually binds.** At the paper's own design there is only about 0.2 s of
delay budget.

---

#### packet loss, and "hold the last value"

**Plain words.** Some broadcasts simply never arrive. When one goes
missing, the car keeps using the last number it received.

**Its job.** It's what any real system does, and we measured the cost:
losing **half** of all messages moves the amplification from 1.007 to
1.013 — barely anything.

**Without measuring it.** You wouldn't know that packet loss **degrades
gracefully** — which is genuinely reassuring, and it's the contrast that
makes the delay finding land.

---

#### spoofing — a car that lies

**Plain words.** A car broadcasting a number that isn't true, whether
through a fault or an attack.

**Its job.** It's the threat we defend against.

**Without a defence.** We measured it: a false "+5 m/s²" broadcast held
for 15 seconds produces a **collision** — the gap goes 27 metres
*negative*.

---

### F — the maths words

This is the section people find hardest, so each one gets a comparison
first and the symbol second.

#### frequency domain — testing every *rhythm* of wobble

**Plain words.** A wobble can be slow (a gentle sway over ten seconds) or
fast (a sharp jerk in half a second). Instead of watching one particular
event unfold, the frequency domain asks: **for each possible rhythm of
wobble, does it grow or shrink as it goes down the line?**

It's like testing a loudspeaker by playing every note separately, rather
than playing one song and hoping it covered everything.

**Its job.** It is the only way to get a trustworthy verdict, because a
single video only ever shows you one particular wobble.

**Without it.** You're back to eyeballing simulations, where a fatal
0.35 % growth is invisible.

---

#### transfer function — the "in-wobble → out-wobble" formula

**Plain words.** A compact formula that answers exactly one question: *a
wobble of this rhythm goes in — how big is the wobble that comes out?*
Engineers write it as a fraction. You don't need to read the fraction;
you need to know it exists and can be plotted as a curve.

**Its job.** It turns a question about physical cars into arithmetic.

---

#### H̃(s) — *"H-tilde"* — the propagation function

**Plain words.** The specific transfer function of **one hop** — one car
to the next. Feed in the wobble arriving at a car, and it tells you the
wobble leaving it.

**Its job.** It is the single most important object in the project. Once
you have it, a question about **200 cars** becomes a question about
**one** — because every hop behaves identically, so you just apply the
same factor over and over.

**Without it.** A 200-car convoy would need a 200-car model, and nothing
could be proved in general.

**One thing worth reading off it:** the headway only ever appears in the
*bottom* of the fraction, and the radio's corruption only in the *top*.
That is why the two behave so differently — headway can only ever calm
things down; the radio can only ever stir them up.

---

#### ‖H̃‖∞ — *"the H-infinity norm"*, or in the room: **"the worst-case amplification"**

**Plain words.** Take the curve above, look at **every** rhythm of wobble,
and write down the biggest amplification you find anywhere on it. That
one number is ‖H̃‖∞.

If it's **at most 1**, then no wobble of any rhythm ever grows — so the
convoy is safe **at any length**, forever. If it's above 1 even slightly,
some rhythm grows, and enough cars will turn that into a jam.

**Its job.** **It is the verdict.** Every pass/fail statement in the deck
is this number compared to 1.

**Without it.** You're left inspecting time traces, where the difference
between safe and unsafe is invisible.

---

#### why the score can never beat 1

**Plain words.** The slowest possible "wobble" is no wobble at all —
everyone cruising steadily. In that situation every car must settle at
**exactly the same speed** as the car in front, so the ratio between them
is exactly **1**. That case is always in the mix, so the worst case can
never come out *below* 1.

You can **break even. You can never win.**

**Its job.** It explains why this problem has no safety margin to spare at
the slow end — and therefore why *slow, drifting* radio noise is the
dangerous kind, rather than fast crackle.

**Without knowing it.** You'd try to design for a comfortable 0.9 and
waste a lot of time. It cannot be done, by physics, not by skill.

---

#### γ (gamma) — a nickname, not a physical thing

**Plain words.** Two quantities always turn up glued together in the
algebra, so we give the pair a single name to stop writing it out.
Bookkeeping, nothing more.

**Its job.** That one substitution is what makes a clean formula possible
at all.

**Without it.** The algebra stays tangled, a crucial cancellation never
happens, and the paper's headline formula never appears.

---

#### "sufficient, not necessary"

**Plain words.** The test we use says: **pass ⟹ definitely safe.** It does
*not* say: fail ⟹ definitely unsafe. So it's a safe test but a **harsh**
one — it rejects some designs that would actually have worked.

**Its job.** Knowing this changes how you use the formula.

**Without knowing it.** You'd trust the formula's answer as the true
limit. **We did, early on, and it was a real bug** — it put the failure
point at the wrong place. The fix: instead of trusting the formula,
search numerically for the true smallest safe headway.

---

#### the uncertainty range

**Plain words.** Because the radio's fudge factor lives in a range, the
effective radio knob does too. The design has to work **everywhere in
that range**, not just at the average.

**Its job.** It's what makes the guarantee *robust* rather than lucky.

**Without it.** One of the sharpest findings in the deck: for the paper's
failing case, checking only the **average** makes the problem look **three
times smaller** than it is, and checking the optimistic end of the range
declares an **unsafe design safe**. That's why our code checks the whole
range instead of assuming which end is worst.

---

#### the feasible region

**Plain words.** Plot every possible combination of the two radar knobs.
The combinations that pass both requirements form a shape. If the shape is
**empty**, no valid design exists at all.

**Its job.** It's the at-a-glance answer to "does a working design exist
here?"

---

### G — the six things we added

#### 1. the predictor — undo the delay

**Plain words.** The broadcast arrived late, so before using it, guess
forward: "this is what the car in front is *probably* doing now, given
what it told me a moment ago."

**Without it.** Instability at **40 ms** of delay. With it, the convoy
survives to about **220 ms** — five times the tolerance.

---

#### 2. the estimator — measure the radio instead of assuming it

**Plain words.** The same fact reaches the car **twice**: once over the
radio, and once from watching the car in front with radar. Comparing the
two tells you how much the radio is lying — so the car can work out the
radio quality **for itself**, without being told.

**Why it's provably safe, not just a hunch.** Because the fudge factor can
never exceed a known limit, this kind of measurement always errs in the
**same known direction**. We then deliberately push it further that way.
The result: the car's estimate always believes the radio is **worse** than
it really is. That's "**conservative by construction**" — it's guaranteed
by the maths, not achieved by tuning.

**Its job.** It removes the paper's biggest assumption.

**Without it.** Everything downstream is guessing at the one number that
matters. Measured: the car estimates {10.4, 3.1, 10.7} where the truth is
{10, 3, 10}.

---

#### 3. the adapter — open the gap when the radio degrades

**Plain words.** Bad radio ⟹ leave a bigger gap. But change it **slowly**,
because suddenly changing your target gap is itself a disturbance. We cap
the rate at 0.05 s of headway per second.

**Without the rate limit.** The convoy is constantly in configurations
nobody ever certified. With it, it slides smoothly through a continuous
range of certified settings, and a gap change is a **deliberate
manoeuvre** rather than a wobble.

---

#### 4. the scheduler — retune the knobs, not just the gap

**Plain words.** When the radio gets bad enough, opening the gap **stops
working entirely**. You have to turn the controller's knobs down as well.

**Why.** One of the two requirements doesn't mention the gap at all — so
no amount of extra gap can satisfy it.

**Without it.** Below a radio quality of about **1.8** there is **no
working design at any gap**. Measured: gap-only adaptation settles at
2.50 s and is *still* unstable; retuning reaches safety at a **shorter**
2.331 s.

---

#### 5. the trust gate — surviving a car that lies

**Plain words.** The radio says one thing; the radar sees another. The
bigger the disagreement, the less the car believes the radio — fading it
out **smoothly** rather than switching it off.

**Why smooth rather than a simple on/off alarm.** Both mistakes are
expensive. Cry wolf and you drop to radar-only, doubling your gap in live
traffic. Miss a real lie and you crash.

**Why this and not a majority vote.** Each car only hears **one** other
car, so there's no majority to take. The trick is that the same fact
arrives by **two different senses** — radio and radar — so we use
agreement between *senses* in place of agreement between *neighbours*.

**Without it.** Measured: collision, gap 27 m negative. With it: 8.4 m of
clearance and no loss of stability.

---

#### 6. the certificate — put a price on the risk

**Plain words.** Instead of "safe / unsafe", say: *"with this gap, the
chance of stepping over the line is 1 in 100."* Then let someone choose
what risk they'll accept.

**How it's possible.** Because the fudge factor comes from 16 coin flips,
there are only 65 536 possible outcomes — few enough to **list them all
and add up the bad ones exactly**. No approximation, no guessing.

**Without it.** Worst-case design pays full price for events that happen
once in a million. Measured: accepting a stated **1-in-100** risk buys
back **10.5 %** of headway.

---

#### the map — knowing where the bad radio is

**Plain words.** Radio quality depends on **where you are** — a tunnel, a
cutting, a built-up area. If the convoy knows the map, it can open its
gaps **before** it reaches the bad stretch, instead of scrambling once
it's already inside.

**Without it.** You can only react once you're already in trouble, and the
gap takes time to open. Measured: reacting leaves the convoy unstable
**6.9 %** of its time in the bad zone; anticipating leaves it **0 %**.

---

### H — how we built and checked it

#### fixed time steps

**Plain words.** The simulation moves forward in equal slices of time,
never varying. This sounds like a detail; it isn't.

**Why.** The radio's random number is held constant for 10 ms at a time.
Equal slices mean the simulation and the radio always agree about what
"now" is.

**Without it.** A cleverer variable-step method would fight the radio
model at every one of ~20 000 boundaries, and would draw fresh random
numbers mid-step — so the thing being simulated wouldn't be the thing we
claim to be simulating.

---

#### reusing the same dice

**Plain words.** We store the *raw coin flips*, not the finished fudge
factor. That way, changing the radio-quality settings **rescales the same
random draws** instead of rolling new ones.

**Without it.** Every time you changed a setting, the dice would be
re-rolled — so any difference between two runs could be the setting *or*
just luck, and you could never tell which.

---

#### exact delay, no shortcuts

**Plain words.** There's a standard shortcut for handling delays
approximately. We don't use it.

**Why.** The margins in this problem are a tenth of a percent. The
shortcut is wrong by around a percent — bigger than the effect we're
trying to measure.

---

#### the certification harness

**Plain words.** Run many scenarios with many different random seeds, and
check **three** things every time: the theory score, the measured growth
down the line, and — most importantly — **the actual smallest gap that
ever occurred**, in a full nonlinear simulation.

**Why the third one.** The elegant theory stops applying the moment a car
is braking flat out. So safety is checked by measuring the gap, not by
trusting the formula.

**Result.** 4 scenario families, 11 numeric checks, all passing:
**CERTIFIED**.

---

#### three independent implementations

**Plain words.** The same equations were built three times: a Python
version, a distributed version with **one operating-system process per
car** (so real timing jitter shows up), and a completely separate
MATLAB/Simulink block model.

**Its job.** The distributed one proves the equations survive real
execution (agreement within 2 %). The Simulink one is an **independent
rebuild in a different tool**, agreeing to one part in ten thousand.

**Without it.** One implementation is not evidence. A bug surviving all
three would have to be made three times, in three languages, in three
different ways.

---

### Five numbers to know cold

| Quantity | Paper | Ours |
|---|---|---|
| minimum safe headway at k_a = 0.5 | 0.9375 | 0.937500 — **exact** |
| best radio knob setting | 0.3183 | 0.318305 — **exact** |
| the shortest headway achievable at all | 0.8727 | 0.872678 — **exact** |
| largest usable radio knob | 0.8333 | 0.833333 — **exact** |
| convoy length at 0.95 / 0.88 s | 345.0 / 324.0 m | 345.00 / 324.00 m — **exact** |

If you remember nothing else numeric: **0.9375 and 0.8727**.

---

## Part 2 — the nine things people get stuck on

Part 1 gives you each term on its own. This part takes the nine specific
sentences on the slides that are genuinely hard, and works through them
slowly. If a slide confuses you, the explanation is almost certainly one
of these nine.

---

### 2.1 "the growth is geometric, gᴺ" — slide 3

**The sentence on the slide.**

> growth is geometric, gᴺ: a per-hop gain g = 1.0035 is invisible over 11
> vehicles (1.039×) and a factor 2 over 200

**What g is.** Take one car. A wobble arrives at it (the car in front
braked a bit). A wobble leaves it (this car brakes a bit too). **g is
how much bigger the wobble got.**

- g = 1.0 → the wobble left exactly as big as it arrived. Break even.
- g = 1.0035 → each car made it **0.35 % bigger**.
- g = 0.98 → each car **shrank** it by 2 %. This is what we want.

**Why "geometric" and not just "growing".** The wobble passes through car
1, then car 2, then car 3… and **each car multiplies it by g again**. So
after N cars the total is not g added N times — it is g **multiplied by
itself** N times:

```
after 1 car    g
after 2 cars   g × g        = g²
after 3 cars   g × g × g    = g³
after N cars   gᴺ
```

That is what "geometric" means: repeated **multiplication**, not repeated
addition. It is exactly compound interest.

**Now put the numbers in.** With g = 1.0035:

| after… | calculation | result | what you'd see |
|---|---|---|---|
| 11 cars | 1.0035¹¹ | **1.039** | 4 % bigger — invisible |
| 50 cars | 1.0035⁵⁰ | 1.191 | 19 % bigger — maybe noticeable |
| 200 cars | 1.0035²⁰⁰ | **2.011** | **twice as big** — a traffic jam |

**The thing to actually take away.** It is not that 1.0035 is a
particularly bad number. It is that **any** number above 1.0 — 1.001,
1.0001, anything — eventually doubles, then triples, given enough cars.
Multiplication does not care how small the step is.

That is why the safety test is **"g ≤ 1, full stop"** and not "g ≤ 1.01
is close enough". There is no "close enough" when the thing compounds.

**And the trap.** We can only realistically simulate about a dozen cars.
Over a dozen cars, a fatal g = 1.0035 shows up as a **4 % difference**
between the front and the back — which is the same size as ordinary
run-to-run variation. **You cannot see it.** Which leads straight to the
next question.

---

### 2.2 "the verdict must be frequency-domain" — slide 3

**Why not just run the simulation and look?** Three reasons, and each one
alone would be enough.

**Reason 1 — a simulation only tests ONE disturbance.** When you run a
simulation, the lead car does one particular thing: in our case a gentle
sine-wave manoeuvre. The convoy handles it. Great — but what about a
sharp stab of the brakes? A slow drift? A rhythmic on-off? **Each of
those is a different test**, and the convoy might pass one and fail
another. You would have to run infinitely many simulations to be sure.

**Reason 2 — the effect is too small to see** over the number of cars you
can afford to simulate. See 2.1: 4 % over eleven cars.

**Reason 3 — you want a statement about ALL platoon lengths**, including
the 200-car one you will never simulate.

**What "frequency domain" actually does.** Instead of asking "what
happened during this particular manoeuvre?", it asks:

> For a wobble that repeats at a given **rhythm** — one sway every 10
> seconds, one every second, one every tenth of a second — how much
> bigger or smaller does it get crossing one car?

Do that for **every possible rhythm**, and you get a curve. The highest
point on that curve is the worst any disturbance can ever do. Call it
`‖H̃‖∞`. Then:

- `‖H̃‖∞ ≤ 1` ⟹ **no** wobble of **any** rhythm grows, so nothing grows
  at **any** platoon length. A guarantee, not a sample.
- `‖H̃‖∞ > 1` ⟹ some rhythm grows, and enough cars turn it into a jam.

**The analogy.** Testing a bridge by driving one lorry across tells you
that one lorry is fine. Computing the bridge's resonant frequencies tells
you **no** vehicle at any speed can shake it apart. The first is a
sample; the second is a proof. Simulation is the lorry; the
frequency-domain test is the resonance calculation.

**Which is why the animation slide (12) says so explicitly:** it is a
sanity check that nothing silly is happening. It is not evidence.

---

### 2.3 "predecessor-following at constant spacing is provably string-unstable: the double integrator forces sup|Γ| > 1" — slide 3

This one sentence has four pieces of jargon. Taken one at a time:

**"predecessor-following"** — each car watches only the car directly in
front. Not the leader, not its neighbours' opinions. Just the one ahead.

**"constant spacing"** — you want to hold a **fixed distance**, say
always 5 metres, whether you are doing 10 km/h or 120 km/h. (The
alternative — the one that works — is a gap that **grows with speed**.)

**"Γ" (gamma)** — the amplification curve from 2.2: for each rhythm of
wobble, how much bigger it gets crossing one car. `sup|Γ|` is the highest
point on that curve — the worst case over all rhythms. Same idea as
‖H̃‖∞; Γ is just the generic name before we commit to a particular
control law.

**"the double integrator"** — the pedal only sets **acceleration**;
acceleration slowly builds **speed**; speed slowly builds **position**.
Two slow stages between the thing you control and the thing you care
about. (Plus the engine's own half-second hesitation on top.)

**So the sentence says:** *if every car only watches the one in front,
and they all try to hold a fixed distance, then no matter how you tune
the controller, the worst-case amplification is bigger than 1 — so the
convoy is unsafe. This is a theorem, not bad luck.*

**Why, intuitively.** Suppose the leader speeds up. To keep a *fixed*
gap, the follower must end up having travelled **exactly as far** as the
leader. But it started late (it only found out when the gap began to
open) and it is sluggish (two slow stages plus engine lag). To make up
ground it has lost, it must **push harder than the leader did** for a
while. That "harder than the car in front" is the amplification above 1 —
and it happens at *every* car in the line, so it compounds.

**And why time headway fixes it.** If the gap is allowed to **grow with
speed**, the follower no longer has to make up all the lost ground. Part
of the extra distance is now *supposed* to be there, because it is going
faster. Mathematically, this adds a braking term proportional to the
car's **own** acceleration — with **no matching term from the car in
front**. That one-sided term is pure damping, and it is what pulls the
amplification back down to 1.

**The price.** Exactly the road space we were trying to save. That trade
is the entire subject of this project.

---

### 2.4 "headway" — what it actually is

**The everyday version.** It is the two-second rule from driving school.
Watch the car in front pass a lamppost, and count how long until you pass
the same lamppost. That count, in **seconds**, is the time headway.

**The formula version.** The gap you keep is:

```
gap  =  r  +  h_w × your speed
        ↑        ↑
   fixed bit   the bit that grows with speed
```

- `r` is a small constant — 1 metre here. It is what stops you touching
  at a red light.
- `h_w` is the **headway**, in seconds. This is the number the whole
  project is about.

**Worked example** at motorway speed, 25 m/s (90 km/h):

| h_w | gap you keep | 12 cars occupy |
|---|---|---|
| 2.0 s (a careful human) | 51 m | 660 m |
| 1.0 s (production cruise control) | 26 m | 360 m |
| **0.95 s** (the paper's Case 1) | 24.75 m | **345 m** |
| **0.88 s** (the paper's Case 3) | 23.0 m | **324 m** |

**Why it is measured in seconds and not metres.** Because it
automatically scales. One number works at every speed: at 10 m/s a 0.95 s
headway is 9.5 m, at 30 m/s it is 28.5 m. Faster ⟹ bigger gap, without
anyone deciding anything.

**Why smaller is the goal.** 21 metres saved across 12 cars means about
**6 % more traffic on the same road**. That is the entire commercial
motivation.

**Why you cannot just set it to zero.** That is constant spacing — see
2.3. It is mathematically impossible to do safely with cars.

---

### 2.5 "the answer is a closed form" — slide 4

**What "closed form" means.** A single explicit formula you plug numbers
into and read the answer off. No simulation, no iteration, no "run this
program until it settles".

The paper's formula for the minimum safe headway is:

```
h_w,lb  =  2 τ₀ (1 − m·k_a) / (1 − n·k_a²)
```

Put in τ₀ = 0.5 s, the radio quality, and your chosen radio knob, and out
comes **0.9375 seconds**. One line of arithmetic on a calculator.

**The opposite of a closed form** would be: "simulate the convoy at
h = 0.90; check; if unstable try 0.95; check; try 0.925…" — a search. You
would get an answer eventually, but you would never know it was *exactly*
right.

**Why it matters here — three reasons.**

1. **It makes a reproduction checkable.** We can compare our 0.937500
   against the paper's printed 0.9375 and say "identical to four
   decimals". If the paper had only printed a graph, all we could do is
   say "our curve looks similar", which proves much less.
2. **It shows you what the answer depends on.** Look at the formula: the
   only things in it are the worst vehicle's sluggishness τ₀ and the
   radio quality. Nothing else. That is the paper's real insight — and
   also its weakness, because it assumes the radio quality is known and
   never changes.
3. **It is instant**, so it can be evaluated inside a moving car in real
   time. Our headway adapter relies on that.

**One honest caveat** (slide 8 mentions it): the formula is **safe but
harsh** — it sometimes says "unsafe" about a design that would actually
have worked. So our code does the slow numerical search anyway when it
needs the *true* answer, and uses the formula for understanding.

---

### 2.6 the 16-bit multiplicative channel, and the SNR — slide 5

**The situation.** The car in front measures its own acceleration — say
exactly **2.00 m/s²** — and broadcasts it. The car behind receives it.
**It does not receive 2.00.**

**"Multiplicative".** The received value is the true value **multiplied**
by a random factor `w` that hovers near 1:

```
received  =  w × 2.00      w might be 0.87, or 1.13, or 0.99…
```

Multiplicative rather than additive means **the error scales with the
signal**: a 10 % error on a big acceleration is a big error, on a small
one it is small. That is how a corrupted digital transmission actually
behaves.

**"16-bit".** The acceleration is transmitted as a digital number made of
**16 binary digits**. Each digit can arrive wrong. The paper models each
one as an independent coin flip, and each digit is worth half of the one
before it:

```
w  =  (1 − 1/ρ)  +  (1/ρ) × [ z₀·1 + z₁·½ + z₂·¼ + … + z₁₅·(1/32768) ]
       ↑                       ↑
   the floor            16 coin flips, each worth
                        half the previous one
```

You do not need to read that. What matters is the **consequence**:

**The error can never exceed a known limit.** Whatever the coins do, the
bracket lands between 0 and about 2, so:

```
1 − 1/ρ   ≤   w   <   1 + 1/ρ
```

**This is the single most important property in the paper.** The radio
can be wrong, but never wronger than 1/ρ. It is a kitchen scale that
might read 5 % off but can never read 50 % off.

**Now "SNR" and ρ.** SNR is *signal-to-noise ratio* — how much stronger
the wanted signal is than the interference. Engineers quote it in
decibels; ρ is the same thing on a plain scale, `ρ = 10^(SNR/20)`. In the
reproduction, SNR = 13.98 dB, which is **ρ = 5**.

**And ρ has a beautifully simple meaning: 1/ρ is the biggest fractional
error the radio can make.**

| ρ | SNR | w lies between | radio is… |
|---|---|---|---|
| 10 | 20 dB | 0.90 – 1.10 | ±10 % — good |
| **5** | **13.98 dB** | **0.80 – 1.20** | **±20 % — the paper's case** |
| 3 | 9.5 dB | 0.67 – 1.33 | ±33 % — poor |
| 2 | 6 dB | 0.50 – 1.50 | ±50 % — nearly useless |

**Two twists worth knowing.**

- **The average is not 1.** The 16 coins are not all fair — the paper's
  Table I gives each one its own odds. The average of w works out to
  **1.048**, so the radio is **biased optimistic by 4.8 %**: on average it
  reports slightly *more* acceleration than really happened. That is the
  unhelpful direction.
- **The coin odds are not known in general.** So the design cannot be
  tuned to one particular set of them. It has to work for the whole range
  of possibilities — which is why the deck keeps talking about
  designing "across the interval" rather than "at the average".

**Why bounded error is worth so much.** Suppose instead you modelled the
noise the usual way, as a bell curve. A bell curve has infinite tails:
there is always *some* tiny chance of an arbitrarily bad reading.
Then **no gap, however enormous, could ever be guaranteed safe.** The
paper's entire hard guarantee rests on the error having a hard ceiling.

---

### 2.7 the theorems, the cases, and where the numbers come from — slide 6

There is really only **one theorem** on the slide, in two halves.

#### Half one — "a working design exists"

> **If** you choose the radio knob below a ceiling, **and** you choose a
> headway above a certain minimum, **then** a pair of radar knobs exists
> that makes the convoy safe.

Two conditions in, one promise out.

**The ceiling on the radio knob** is `1/(1 + 1/ρ)`. At ρ = 5 that is
**0.8333**. Above that value the radio's noise is amplified more than the
early warning is worth, and nothing can save you.

**The minimum headway** is the formula from 2.5. At ρ = 5, τ₀ = 0.5 s and
a radio knob of 0.5, it evaluates to **0.9375 s**.

Note what the theorem does **not** say: it does not tell you *which*
radar knobs to use. It only promises some exist. Finding them is the
"feasible region" picture on slide 16.

#### Half two — "and here is the best possible choice"

The minimum headway depends on the radio knob you picked. So: **what is
the best radio knob?** Differentiate, solve, and you get:

- best radio knob **k_a\* = 0.3183**
- the headway it achieves: **h\*_w,lb = 0.8727 s**

**The counter-intuitive bit — and a good thing to be asked about.** The
best setting is **0.3183, not the largest allowed 0.8333**. More early
warning is *not* better. Why? Because the radio knob multiplies **both**
the useful warning **and** the radio's noise. Turn it up and you get more
warning but more noise; turn it down and you get less of both. 0.3183 is
where the trade is best.

#### The four headline numbers, and what each one is

| Number | What it is | Plain meaning |
|---|---|---|
| **0.8333** | ceiling on the radio knob | turn it above this and no design works at all |
| **0.9375 s** | minimum headway at knob = 0.5 | with that knob, this is the closest you may safely drive |
| **0.3183** | the best knob setting | the sweet spot between warning and noise |
| **0.8727 s** | minimum headway at the best knob | **the closest anyone can ever drive at this radio quality** |

We reproduce all four to **0.00 %** error. Because they are closed forms
(2.5), that comparison is meaningful to four decimal places.

#### The five cases — the paper's own worked examples

These are the paper's demonstrations, and they are what our slide 14
checks number by number.

| Case | Setup | Point being made | Result |
|---|---|---|---|
| **1** | knob 0.5, headway **0.95 s** | 0.95 > 0.9375, so it should be **safe** | errors **shrink**: 1.593 → 1.535 m |
| **2** | same knobs, headway **0.65 s** | 0.65 < 0.9375, so it should be **unsafe** | errors **grow**: 0.882 → 0.886 m |
| **3** | best knob 0.3183, headway 0.88 s | operating at the optimum | ⚠ our numbers are 15 % off — reported as an open item |
| **4** | compare Case 1 against Case 3 | how much **road space** the optimum saves | 345 m → **324 m** for 12 cars |
| **5** | Case 1 knobs, but every car has a **different** sluggishness | does "any car up to τ₀" really hold? | yes — it still works |

**Case 2 is the one worth understanding.** It is the *negative* test.
Same cars, same knobs, same manoeuvre — the **only** change is the
headway, dropped from 0.95 to 0.65. And the errors flip from shrinking to
growing. That is the paper's claim being demonstrated in the most direct
possible way.

It is also the hardest thing to reproduce, and therefore the most
convincing when you do. The headway is only **6 % below** the limit, and
the resulting growth is **0.5 % over eleven cars** — a tiny effect. We
match it to **0.02 %**, including the direction. Getting a big effect
right is easy; getting a half-percent effect right with the correct sign
means the model really is the same model.

---

### 2.8 the novelty slide, row by row — slide 7

This slide looks dense because it is a table, but it has **one idea**:

> The paper needs six things to be true. Each of our phases takes **one**
> of them away and measures what happens.

Read it as a list of assumptions being demolished one at a time.

| Phase | The paper assumes… | We remove it by… | And find |
|---|---|---|---|
| **0 Reproduce** | *(nothing — this is the baseline)* | rebuilding the paper from scratch | our numbers match its printed numbers exactly. **This is what earns the right to criticise it.** |
| **1 Gap audit** | "the radio is instant, and no message is ever lost" | adding a real delay and dropped messages | **delay is what actually binds, not noise.** The paper's own design only tolerates ≈0.2 s. Lost messages barely matter. |
| **2 Distribute** | "one simulator program is a fair test" | running **one operating-system process per car**, on real timing | the equations survive real execution — within 2 % |
| **3 Adapt ★** | "the radio quality is known and never changes" | letting each car **measure the radio quality itself** | it works: cars estimate {10.4, 3.1, 10.7} where the truth is {10, 3, 10} |
| **4 Re-tune** | "the controller knobs are set once, in advance" | re-tuning the knobs while driving | below a radio quality of ≈1.8, opening the gap **stops working entirely** — only re-tuning helps |
| **5 Flagship ★★** | "reacting is fast enough" · "no car lies" · "safe/unsafe is enough" | a map, a lie-detector, and a risk number | 6.9 % → **0 %** unstable · collision → **8.4 m clearance** · **10.5 % less headway** for a stated risk |
| **6 Certify** | "one working implementation is evidence" | a Monte-Carlo harness plus a third, independent build | **CERTIFIED**; the independent MATLAB build agrees to 1 part in 10 000 |

**The stars.** ★ marks the main research contribution (phase 3). ★★ marks
the three flagship extensions (phase 5).

**The "ADR" column** is just a reference — it points at the decision
document that records *why* we did it that way and what alternatives we
rejected. It is there so nothing on the slide is an unbacked assertion.

**If you say one thing about this slide:** *"Every phase deletes exactly
one assumption from the paper and reports exactly one measured number."*

---

### 2.9 what θ is, and what e^{−θs} does to the formula — slide 10

**θ (theta) is simply a delay, in seconds.** By the time the radio message
arrives, gets decoded and gets used, it is already **out of date**. The
car in front is no longer doing what the message said it was doing. θ is
how out of date it is — typically tens of milliseconds.

**The paper assumes θ = 0.** That is the gap Phase 1 attacks.

**Where it appears.** In the one-hop formula, the radio term picks up an
extra factor:

```
                    k̃_a s² · e^{−θs} · P(s)   +   k_v s   +   k_p
    H̃_θ(s)   =    ───────────────────────────────────────────────
                                    D(s)
                        ↑
              only the RADIO term gets it —
         the radar terms are not delayed, so they don't
```

**What `e^{−θs}` actually does — the important part.** In this kind of
formula, `e^{−θs}` is the standard way of writing "**delay this signal by
θ seconds**". And it has one very specific effect:

- It does **not** make the signal bigger or smaller. Its size is exactly
  1. The radio's contribution is not weakened at all.
- It **shifts the timing**. The contribution now arrives late relative to
  what is actually happening.

**Why late timing is fatal specifically here.** Remember what the radio
term is *for*: it is the **early warning**. Its entire value is that it
arrives **before** the gap starts changing. Delay it, and:

1. First it just stops helping — the "early" warning is no longer early.
2. Then it starts actively **hurting** — the correction arrives at the
   moment the situation has already reversed, so the car pushes when it
   should be pulling.

That second stage is what tips the convoy over the edge.

**And it is fatal at a shockingly small delay.** We measured the tipping
point at **θ ≈ 40 milliseconds**. That is far too small for anybody to
think to check for — which is exactly why it is a good finding.

**What `P(s)` next to it is.** That is our fix — the **predictor**. Since
we know roughly how stale the message is, we extrapolate it forward:
"given what the car in front told me 40 ms ago, and how things were
changing, here is what it is *probably* doing right now." Undo the
delay before using the number.

**How much it buys.** Without it, trouble at ≈40 ms. With it, the convoy
stays safe out to ≈**220 ms** — about five times the tolerance. And the
curve of "minimum safe headway versus delay" goes **flat**, meaning the
delay stops costing anything at all across the useful range.

---

## Part 3 — the 20 slides

The **footer number** at the bottom-right is not the slide index: the
title and team pages are unnumbered, so slide 3 carries footer 1. When
someone says "go back to 12", they mean the footer.

---

### Slide 1 — Title

**What it says.** Paper title, the DOI-carrying citation, three backends,
and a status line: printed digits reproduced, 197 tests green, harness
CERTIFIED.

**Say this.** "This is a 2025 IEEE paper on how close self-driving cars
can safely follow each other when their radio is noisy. We rebuilt it,
matched its printed numbers exactly, then extended it."

**If asked — "why this paper?"** Recent, and it produces **closed forms**,
so a reproduction can be checked to four decimals rather than eyeballed
off a figure. Its assumptions are clean, which makes them honestly
attackable.

---

### Slide 2 — Project Team

Mannav Daasaradhi · Manikanta Koushik · Prajith · Sakthi Charukesh.

---

### Slide 3 — Problem statement: the physical problem *(footer 1)*

**What it says.** Left: capacity is gap-limited, not speed-limited; two
kinds of stability, and only one is obvious; string instability grows
**geometrically**, so 1.0035 is invisible over 11 cars and a factor 2
over 200. Right: radar alone can't do constant spacing; time headway
fixes it but costs the road space we wanted; V2V feedforward halves the
floor — **but the lever is a radio.**

**Say this.** "String instability is what a phantom traffic jam is, and
it's invisible: a per-hop gain of 1.0035 does nothing over eleven cars
and stops the road over two hundred. That's why we never judge this from
a video."

**If asked — "isn't 1.0035 basically 1?"** 1.0035²⁰⁰ = 2.01. What matters
is the gain raised to the platoon length.

---

### Slide 4 — Problem statement: the research problem *(footer 2)*

**What it says.** The paper's exact question, its **five assumptions**
(channel known and static · guarantee binary and in-the-mean · no delay,
no loss · every link honest · gains chosen once offline), and our three
questions (can ρ be estimated online? can the verdict be made
quantitative? what if a link lies?).

**Say this.** "The paper answers 'what headway is safe for a known,
static, honest channel?' We ask what a platoon does when the channel is
unknown, time-varying and possibly hostile — and how much risk it's
accepting."

**If asked — "isn't that a straw man?"** Every assumption listed is one
the paper **states explicitly**. We take the authors at their word and
measure what each one costs.

---

### Slide 5 — Base paper: model, policy, control law, channel *(footer 3)*

**What it says.** Four blocks. The vehicle (eq. 1) with τ **uncertain** in
(0, τ₀] — which is what lets one design cover a mixed fleet. The spacing
errors (eq. 2). The control law (eq. 6) — **three terms, three
information sources**: the early-warning term off the **radio**, the
"stop rushing" term off the radar, and the gap term off radar and the
car's own sensors. And the radio model built from 16 coin flips, which
has three properties worth saying out loud — its error can **never exceed
a known limit**; its average is **not 1** but 1.048, so the radio is
**biased optimistic by 4.8 %**; and the coins' individual odds are
**unknown**, so the design has to work across a whole range of
possibilities rather than at one nominal value.

**Say this.** "Three terms. Two come off the radar and are clean. One
comes off the radio and gets multiplied by a number the car can't see.
That single asymmetry is the whole paper."

**If asked — "why 16 bits?"** The broadcast is a quantised digital value.
It's also a gift: 16 bits means the law of w has exactly 65 536 atoms,
which we can enumerate **exactly** — that's what flagship C is built on.

---

### Slide 6 — Base paper: Theorem 2 and the five cases *(footer 4)*

**What it says.** Eq. (14) with its two structural facts (headway enters
only via γ, γ only in the denominator); Definition 2 (robust string
stability — for **all** τ *and* **all** k̃_a); Theorem 2(b) the existence
bound (17); Theorem 2(c) the optimum (18)–(19); and the five §IV cases.

**Case 2 is the tightness test** — the headway is violated by only 6 %
and the errors **grow**.

**Say this.** "Theorem 2 gives two closed forms: the shortest safe
headway for any chosen feedforward gain, and the gain making that headway
as short as possible. At ρ = 5 those are 0.9375 s and 0.8727 s, and we
hit both exactly."

**If asked — "which case is the real test?"** Case 2. Anyone can
reproduce a stable run. Case 2 is 6 % on the wrong side and the growth is
0.5 % over eleven hops — and we match it to 0.02 % **including the
direction of the trend**.

---

### Slide 7 — Novelty, phase-wise: the seven phases *(footer 5)*

**What it says.** Seven rows: phase · what it adds · **assumption
removed** · key result · ADR. Each phase deletes exactly one assumption
and reports exactly one measured number.

| Phase | Removes | Result |
|---|---|---|
| 0 Reproduce | — | printed digits exact |
| 1 Gap audit | "no delay / no loss" | **delay binds, not noise** — ≈0.2 s of budget |
| 2 Distribute | "a monolithic sim is enough" | < 2 % per-hop under real OS jitter |
| 3 Adapt ★ | "channel is time-invariant" | ρ̂ {10.4, 3.1, 10.7} vs true {10, 3, 10} |
| 4 Re-tune | "gains fixed once chosen" | breaks the ρ\* ≈ 1.8 wall: 1.019 → 0.99999 |
| 5 Flagship ★★ | "reactive is enough / honest / binary" | 0 % unstable · collision → 8.4 m · −10.5 % headway |
| 6 Certify | "one implementation is evidence" | CERTIFIED; Simulink ↔ Python to 1e−4 |

**Say this.** "Every phase deletes one assumption from the paper and
reports one measured number. Nothing here is a claim without an artefact
behind it."

**If asked — "what surprised you most?"** Phase 1. We expected noise to
bind, because that's what the paper is about. It doesn't — **delay
does.**

---

### Slide 8 — Systems: what we built, and the three backends *(footer 6)*

**What it says.** Left — the car model; the three control laws sharing
one interface (the paper's law keeps no memory, so the scheduler can swap
it out mid-run without losing anything); the radio, with its four
independently switchable problems; and the map of where the radio is bad.

Right — how we compute. Delays handled **exactly** rather than by the
usual shortcut. The minimum safe headway found by **numerical search**
rather than by trusting the paper's formula. All 65 536 possible radio
outcomes listed exactly. The estimator, which averages over a fixed
**window of time** — averaging over a fixed *number of readings* was
tried first and failed, because when messages are sparse a fixed count
covers an arbitrarily long stretch, and the estimate got stuck. And the
three implementations.

**Say this.** "Two decisions here are why the numbers can be trusted. We
compute the delay exactly instead of approximating it — the usual
shortcut is wrong by about a percent, and our margins are a tenth of a
percent. And we search for the true minimum headway instead of using the
paper's formula, because that formula is safe but harsh."

**If asked — "why store the raw dice rather than the finished numbers?"**
So that changing a setting **reuses the same random draws** instead of
rolling new ones. Otherwise any difference between two runs could be the
setting *or* just luck, and you would never be able to tell which.

---

### Slide 9 — The maths in one page *(footer 7)*

**What it says.** Four steps.

1. **The car and the gap rule.** The pedal is two slow stages away from
   where the car actually is, and the gap rule turns out to behave like
   **suspension** — it smooths wobbles out instead of passing them on.
   That smoothing costs no fuel and no effort; you pay for it in metres.
2. **One formula covers every hop.** Work out how the wobble entering a
   car compares to the wobble leaving it, and the same answer applies at
   every car in the line — so a 200-car question becomes a one-car
   question. Reading it off: the **headway can only ever calm things
   down**, and the **radio can only ever stir them up**.
3. **The verdict.** Test every rhythm of wobble, take the worst
   amplification you find, and require it to be at most 1. But the "no
   wobble at all" case always scores **exactly 1** — so you can break
   even and you can never win.
4. **Two requirements on the knobs, then the answer.** Combine them and
   out drops the minimum safe headway; choose the radio knob to make that
   as small as possible and out drop the paper's headline numbers:
   **0.9375 · 0.3183 · 0.8727**.

**The closing band is the one to point at.** One of the two requirements
**never mentions the headway at all** — so no amount of extra gap can
satisfy it. That is why opening the gap eventually stops working
altogether, and it is what sets up slide 17.

**Say this.** "The gap rule is secretly a shock absorber — it smooths
wobbles out, and you pay for that in metres, not in fuel. And the very
best a convoy can ever do is break even, so there is no safety margin at
the slow end. That is exactly why a slowly-drifting noisy radio is the
dangerous kind."

**If asked — "why not design for a comfortable 0.9?"** Because when
everyone is simply cruising, every car ends up at exactly the same speed
as the one in front, so the ratio between them is exactly 1. That case is
always in the mix, so the worst case can never come out below 1. It is
physics, not a lack of skill.

**If asked for the full derivation:** offer to do it on the board. The
chain is: car model → gap rule → the one-hop formula → the "at most 1"
test → the two requirements on the knobs → the minimum headway → the best
radio knob. The key moment is a **cancellation**: squaring the second
requirement makes one whole term vanish, turning a curved inequality into
a straight one. That is the only reason a clean formula exists at all,
and it is why the set of workable knob settings is a shape with
**straight edges**.

---

### Slide 10 — What we added: six equations *(footer 8)*

**What it says.** One row per mechanism: predictor · estimator · adapter ·
scheduler · trust gate · certificate — each with its equation and **what
breaks without it**. All six are expanded in Part 1 above; the delay term is worked through in Part 2.9.

**Say this.** "The estimator is the piece I would defend hardest. The
radio's error can never exceed a known limit — so when our measurement of
it is wrong, it is **always wrong in the safe direction**. That is
guaranteed by the maths, not achieved by tuning."

**If asked — "why not just take a majority vote against a lying car?"**
Because each car only hears **one** other car — there is no majority to
take. Our defence uses a different kind of agreement: the same fact
arrives by **two senses**, radio and radar. Agreement between *senses*
replaces agreement between *neighbours*.

**If asked — "won't a moving headway itself destabilise things?"** That's
what the 0.05 s/s rate limit is for — it caps the extra `−ḣ·v` term at
1 m/s². The platoon moves through a continuum of **certified**
configurations. Visible waves in the demo during a headway change are
**commanded** (≈ v·Δh), not instability.

---

### Slide 11 — Simulation: the Simulink model *(footer 9)*

**What it says.** The whole convoy is drawn **once** and every signal
line carries all twelve cars at once — so there is one control law in the
diagram, not twelve copies. Twelve copies would be twelve places for a
typo to hide. Which car follows which is set by a **single block**, not
by wiring, so the arrangement can be changed without redrawing anything.
And the model file itself is **generated by a script**: the thing under
version control is a readable 300-line program, and the model is rebuilt
from it.

**Say this.** "The Simulink model isn't a translation of our Python — it's
an independent re-derivation of the same equations in a different tool,
and it agrees to one part in ten thousand."

**If asked — "why generate it from a script?"** A binary `.slx` can't be
code-reviewed or diffed. A 300-line `.m` can, and the model can't
silently drift from what the script says it is.

---

### Slide 12 — Simulation: the platoon in motion *(footer 10)*

**What it says.** Four instants of the same 200 s run, top-down, cars at
true 4 m length, colour-coded by spacing error. It shows the manoeuvre is
physically sensible — and it explicitly says **it cannot show string
stability**, because 1.0035 compounds to 1.039 over eleven cars.
Generated through `cacc.platoon`, so picture and numbers can't disagree.

**Say this.** "This animation is a sanity check, not evidence. If a
per-hop gain of 1.0035 were visible here, we wouldn't have needed the
rest of the deck."

---

### Slide 13 — Simulation: the live, interactive demo *(footer 11)*

**What it says.** `demo/cacc_live_sim.html` — a **live** simulator, not a
recording. Sliders for ρ, delay, packet loss, leader manoeuvre, platoon
length, and adaptation on/off. In the captured state the radio quality
was dropped from 5.0 to 3.0, and with adaptation on **the convoy opened
its own gap from 0.95 s to 1.17 s** without being told to. The measured
car-to-car growth stayed at exactly break-even, and the smallest gap
never fell below 22.1 m.

**Say this.** "Turn adaptation off at the same radio quality and the
car-to-car growth goes above break-even. That's the base paper's
predicted failure, reproducible live, in front of you."

**If asked — "is this a toy?"** The control law, the vehicle dynamics and
the 16-bit channel are ported from the project core — which is why its
numbers agree with the offline studies.

---

### Slide 14 — Results: the paper reproduced, number by number *(footer 12)*

**What it says.** Four closed forms **exact**. Cases 1, 2, 4, 5 under 1 %.
**Case 3 off by 15 %, reported as an open item.** Plus five independent
cross-checks, each by a different route: eq. (25) residual **1.8e−12** · a
400 001-point grid search giving k_a\* = 0.318304 vs the closed form's
0.318305 · Routh–Hurwitz agreeing with `roots(D)` · the Remark 4 limits ·
and a counterexample confirming the conditions are **sufficient, not
necessary**.

**Say this.** "Four closed forms exact to the printed digits, three cases
under one percent, and one case off by fifteen percent that we report as
an open item. That last row is the reason you should believe the other
eight."

**If asked — "what's wrong with Case 3?"** We don't know yet. It's
characterised — isolated to that one gain configuration, with Cases 1, 2,
4, 5 all matching — but not explained. We report it rather than hide it.

---

### Slide 15 — Results: time domain *(footer 13)*

**What it says.** Same cars, same leader, same 200 s — **only the headway
differs.** Case 1 (h = 0.95 > h_lb): peaks **shrink**, 1.5959 → 1.5436 m.
Case 2 (h = 0.65 < h_lb): peaks **grow**, 0.8821 → 0.8872 m.

Plus a finding that is **ours, not the paper's**: the paper's time-domain
figures come from the **equivalent deterministic system**, not from
stochastic realisations. We proved it by running all three channel modes
— only `mean` mode reproduces the printed profiles.

**Say this.** "Six percent of headway is the entire difference between
shrinking and growing. And we worked out that the paper's own figures
come from its mean-channel equivalent system — by running all three modes
and seeing which one reproduces the printed profiles."

---

### Slide 16 — Results: frequency domain *(footer 14)*

**What it says.** Case 1: **1.000000** → stable. Case 2: **1.003500** →
unstable. Feasible polygon: `a₁/a₂ = 1.0133 > 1` ⟹ non-empty.

**And the finding that matters — the low end of I binds.** For Case 2:
**1.0035** at k̃_a = 0.40, 1.0012 at the mean, **1.0000** at k̃_a = 0.60.
So checking only the mean understates the violation **3×**, and checking
the high end declares an unstable design **safe**.

**Say this.** "Case 2 is unstable at 1.0035. Evaluate it only at the
average channel and you'd call it three times less bad; evaluate at the
optimistic end and you'd call it safe. That's why we grid the whole
interval."

---

### Slide 17 — Results: the gaps we found, and what closed them *(footer 15)*

**What it says.** Phases 1–4, each measured.

| | Gap | Fixed |
|---|---|---|
| **1 delay & loss** | h_min ≤ 0.95 s only until θ ≈ 0.23 s | predictor: ‖H̃‖∞ 1.00923 → **1.00000**, h_min(θ) flatlines |
| **2 distribute** | a monolithic sim hides jitter, message age, clock skew | one OS process per car: **< 2 %** per-hop agreement |
| **3 adapt** | fixed headway leaves the platoon unstable the whole time in a bad zone | ρ̂ {10.4, 3.1, 10.7} vs {10, 3, 10}; headway opens and recovers |
| **4 re-tune** | ρ = 2, headway-only: h = 2.50 s and **still unstable** (1.019) | joint: h = **2.331 s**, ‖H̃‖∞ = **0.99999** |

**Say this.** "Phase 4 is the counter-intuitive one. Below a channel
quality of about 1.8, opening the gap doesn't work at **any** gap — you
have to re-tune the gains. And when you do, you land at a **shorter**
headway than the failing headway-only design."

**If asked — "why is re-tuning shorter?"** Because the headway-only
adapter is fighting a constraint headway can't touch. Lowering k_v moves
the (28b) floor down, which reopens the feasible window, which permits a
smaller h_w. The 2.50 s figure was never a solution — it was the adapter
running out of road.

---

### Slide 18 — Results: flagship A and B *(footer 16)*

**What it says.**

**A — predictive QoS map.** Reactive: h = 0.95 s at zone entry,
‖H̃‖∞ = 1.0022, **6.9 %** of in-zone time unstable, peak error 17.23 m.
Predictive (45 s look-ahead): h = **1.234 s already at entry**,
‖H̃‖∞ = 1.0000, **0.0 %** unstable, peak error 14.23 m.

**B — trust gate**, under +5 m/s² bias injected for 15 s. Ungated:
minimum gap **−27.4 m, a collision**, peak error 50.5 m. Gated:
**+8.37 m**, peak error 14.24 m, minimum trust weight 0.254.

**Say this.** "Ungated, a fifteen-second lie produces a collision — minus
27 metres of gap. Gated, the same attack leaves eight metres of clearance
and the platoon never loses string stability."

**If asked — "why 45 seconds of look-ahead?"** Adaptation is rate-limited
to 0.05 s/s, so opening 0.95 → 1.234 s takes ≈6 s; the look-ahead must
cover that plus the distance travelled while adapting at highway speed.

---

### Slide 19 — Results: flagship C, a priced risk *(footer 17)*

**What it says.** The law of w is known **exactly** — 65 536 atoms from
convolving the bit laws — so there's no need to bound; we **integrate**.
At ρ = 3: worst case 1.1544 s (exceedance 0) · **chance-constrained
1.0334 s (exceedance 0.0095)** · 1.0522 s (0.0010) · mean-square 0.9944 s
(**0.076 — rejected**). Analytic and Monte-Carlo agree to 3–4 decimals.

**Say this.** "We know the channel's distribution exactly, so we don't
have to bound it — we can integrate it. That buys ten percent of headway
for a stated one-in-a-hundred risk, instead of paying full price for a
one-in-a-million event."

**If asked — "is 1 % exceedance acceptable?"** That's deliberately not our
call — it's the point. The number is now **stated and calibrated**
instead of implicit. A regulator sets ε; the method returns the headway. A
worst-case bound offers no such dial.

---

### Slide 20 — The verdict, and what this project concludes *(footer 18)*

**What it says.** Four headline figures — **CERTIFIED** · **0.00 %**
reproduction error · **87** tests · **1e−4** cross-backend — then the
three certification criteria (theory, time domain, and **safety on the
nonlinear sim**, because saturation sits outside the linear envelope),
then six conclusions:

1. The base paper reproduces **exactly** — the credibility anchor.
2. Its guarantee is real but **thin**, and **delay** consumes the margin.
3. A fixed design can't survive a changing channel, and headway alone
   can't save it below ρ\* ≈ 1.8.
4. The channel **is** estimable online, and conservatively.
5. A calibrated risk budget is cheaper **and** more honest than a worst
   case.
6. **Reproducibility is the deliverable.**

**Say this.** Conclusion 2 and conclusion 5 are the two that are genuinely
ours. Lead with those.

**If asked — "no hardware?"** Correct, and deliberate. The thesis is that
virtual validation can substitute for a testbed — so that gap **is** the
argument, and the three-backend agreement plus the Monte-Carlo harness is
the evidence for it.

**If asked — "what about emergency braking?"** Explicitly out of scope,
for a precise reason: it's **outside the linear certified envelope**.
‖H̃‖∞ says nothing once the actuator saturates — which is exactly why the
harness checks minimum gap on the **nonlinear** simulation.

**If asked — "what failed along the way?"** Three things, all on record:
the **count-based** estimator window (ρ̂ stuck at 3.1 — fixed by going
age-based); using the paper's **closed form directly** instead of
bisecting (put the wall at ρ ≈ 3 instead of 1.8); and the **mean-square**
risk criterion (implemented, then rejected at 7.6 % exceedance).

---

## Part 4 — if the clock runs out

**Never drop:** 3 (string instability is invisible) · 6 (the two closed
forms) · 14 (we reproduce them exactly) · 20 (the verdict).

**Add if you have 10 minutes:** 16 (the frequency-domain verdict) and 17
(the four gaps, measured).

**Add if you have 20:** 4 (the five assumptions), 9 (the maths — because
the (28a) asymmetry explains slide 17), 18 (the two flagship
before/afters).

**Drop first:** 8 (systems) and 11–13 (the simulation trio) — everything
on them is in Parts 1 and 2 above.

---

## Part 5 — where things live

| Need | Path |
|---|---|
| the deck | `report/CACC_Control_Systems_Presentation.pptx` / `.pdf` |
| rebuild it | `python scripts/build_ppt.py` (needs `python-pptx`) |
| slides 1–8 | `scripts/ppt/content_core.py` |
| slides 9–10 | `scripts/ppt/content_theory.py` |
| slides 11–20 | `scripts/ppt/content_evidence.py` |
| layout & palette | `scripts/ppt/theme.py` |
| full derivations | `docs/theory/` (T-01 … T-10) |
| why any decision was made | `05_Decision_Log.md` → `docs/decisions/` |
| deep system tour | `docs/MASTER_SYSTEM_EXPLAINER.md` |

A clean build ends with `fit: every slide fits above its font floor`; if
it instead prints an overfull-slide report, that page needs splitting or
trimming.

Every number quoted on a results slide traces to a
`results/<study>/<stamp>/metrics.json` or to a pinned assertion in the
87-test suite. Scenarios are declarative YAML, so any study is
reproducible from a name plus a seed.

---

## Part 6 — running the simulation yourself

Everything below was run and verified on this machine on 2026-08-19. Run
every command **from the project root**
(`06_Cooperative-Adaptive-Cruise-Control/`).

### 6.0 The one thing to know first

The Python code lives in a **conda environment called `cacc`**. It is
already created and the project is already installed into it. You do not
need to set anything up — but you **must** use that environment, not
plain `python`.

Two ways to do it. Either activate it once per terminal:

```
conda activate cacc
python scripts/reproduce_base_paper.py --quick
```

or prefix each command, which needs no activation:

```
conda run -n cacc python scripts/reproduce_base_paper.py --quick
```

**If you type plain `python` you will get `ModuleNotFoundError: No module
named 'cacc'`.** That is the single most likely thing to go wrong, and it
means only that you are in the wrong environment.

---

### 6.1 The 30-second version — the live demo

No environment, no install, nothing to build.

```
demo/cacc_live_sim.html          ← just double-click it
```

It is a **real simulator running in the browser**, not a video: the
control law, the car dynamics and the noisy radio are all computing live
while you watch. Sliders for radio quality, delay, packet loss, leader
manoeuvre, convoy length, and adaptation on/off.

**The demonstration to do in front of people:** load the *Interference*
preset (radio quality drops 5 → 3). With adaptation **on**, the convoy
opens its own gap from 0.95 s to 1.17 s and stays at break-even. Turn
adaptation **off** at the same radio quality and the car-to-car growth
climbs above 1 — the base paper's predicted failure, on demand.

---

### 6.2 Check everything still works — the test suite

```
conda run -n cacc python -m pytest tests -q
```

**Takes ~40 s. Expect `197 passed` (about 3 minutes).** These are not toy tests — the
paper's printed numbers are pinned as assertions, so if this passes, the
reproduction is intact.

---

### 6.3 The main run — reproduce the paper

```
conda run -n cacc python scripts/reproduce_base_paper.py --quick     # ~2 min
conda run -n cacc python scripts/reproduce_base_paper.py             # full, longer
```

Writes to `results/base_paper/<timestamp>/`:

| File | What it shows |
|---|---|
| `fig1_design_space.png` | minimum headway vs the radio knob, for several radio qualities |
| `fig2_frequency_response.png` | the amplification curve — Case 1 safe, Case 2 unstable |
| `fig3_time_domain.png` | the spacing errors shrinking (Case 1) and growing (Case 2) |
| `fig4_optimal_case.png` | the optimum, and the convoy-length comparison |
| `fig5_delay_extension.png` | **our extension** — minimum headway vs radio delay |
| `metrics.json` | every number, including the paper-vs-ours comparison |

**How to check it worked.** Open `metrics.json` and look at
`theory_checks`. You should see:

```
h_lb(ka=0.5, rho=5)   paper 0.9375    ours 0.9375
h_star_lb(rho=5)      paper 0.8727    ours 0.8726779962…
```

Those are the two numbers from Part 2.7. If they match, the whole
reproduction is sound.

`--quick` uses 6 followers and 2 random seeds instead of 12 and 5. The
figures look the same; the numbers are slightly coarser. Use it while
you are experimenting, and drop it for anything you will show.

---

### 6.4 Run one scenario — the clearest single result

```
conda run -n cacc python scripts/run_scenario.py scenarios/leader_brake.yaml
```

Runs the **same** scenario twice, once with radar-only cruise control and
once with the radio link, and compares them. Takes about a minute and
writes `results/leader_brake/<timestamp>/figure.png` plus
`metrics.json`.

**This is the most persuasive thing you can run in one command.** The
measured result:

| | radar only (ACC) | with the radio (CACC) |
|---|---|---|
| worst spacing error | **6.76 m** | **0.76 m** |
| car-to-car growth | 1.041 → 1.077 — **growing** | 0.49 → 0.96 — **shrinking** |
| verdict in `metrics.json` | `"empirically_string_stable": false` | `"empirically_string_stable": true` |

Same cars, same braking manoeuvre, same headway of 0.7 s. The **only**
difference is whether the radio link is used. Radar-only amplifies the
disturbance down the line; the radio link damps it out.

The other scenarios in `scenarios/` work the same way:
`leader_sine.yaml` (the paper's gentle manoeuvre), `ma2025_sine.yaml`
(the exact reproduction setup), `qos_adaptive.yaml`,
`predictive_qos.yaml`, `spoof_defense.yaml`.

---

### 6.5 The extension studies — one command each

Each writes figures and a `metrics.json` into
`results/<study>/<timestamp>/`.

```
conda run -n cacc python scripts/qos_adaptive_study.py --quick     # Phases 3 & 4
conda run -n cacc python scripts/predictive_qos_study.py           # Flagship A
conda run -n cacc python scripts/spoof_defense_study.py            # Flagship B
conda run -n cacc python scripts/stochastic_cert_study.py          # Flagship C
conda run -n cacc python scripts/sweep_delay.py                    # delay budget
```

- **qos_adaptive_study** — the radio quality drops from 10 to 3 and
  recovers. Shows the car estimating it, opening its gap, and the
  fixed-knob wall at radio quality ≈ 1.8.
- **predictive_qos_study** — reacting to a bad zone vs anticipating it
  (6.9 % unstable → 0 %).
- **spoof_defense_study** — a lying car, with and without the trust gate
  (collision → 8.4 m clearance).
- **stochastic_cert_study** — the risk certificate: headway for a stated
  1-in-100 risk instead of the worst case.

---

### 6.6 The full certification — the product face

```
conda run -n cacc python scripts/certify.py --quick        # a few minutes
conda run -n cacc python scripts/certify.py --seeds 5      # the full run
```

Runs the whole matrix of scenarios × random seeds and writes a
**self-contained HTML report** to
`results/certification/<timestamp>/report.html` — open it in a browser.
Every check states its own pass/fail criterion, and the run ends with a
single verdict. **Expect `CERTIFIED`.**

This is the artefact that stands in for a hardware test rig, so it is the
right thing to show if someone asks "how do you know it is safe?"

---

### 6.7 The MATLAB / Simulink backend

MATLAB **R2026a** is installed on this machine, and only base MATLAB +
Simulink are needed — no extra toolboxes.

In the MATLAB command window:

```matlab
cd matlab
run_all                  % ~90 s, the deterministic-equivalent channel
run_all('stochastic')    % the realised 16-bit random channel
```

Writes twelve PNGs and a full transcript to `matlab/results/`
(`run_all_log.txt` — that log is where the paper-vs-ours comparison
lives). The Simulink model builds itself on the first run.

To open the block diagram and actually look at it:

```matlab
addpath models; open_system('cacc_platoon')
```

**Why bother, given the Python already works?** Because this is an
**independent rebuild** of the same equations in a different tool by a
different route — not a re-run of the same code. The two agreeing to one
part in ten thousand is much stronger evidence than either alone. It is
also where the block diagrams on slide 11 come from.

---

### 6.8 The ROS 2 backend — one process per car

This one runs in **WSL (Ubuntu)**, not on Windows, because ROS 2 needs
Linux. Ubuntu is installed on this machine. See
`docs/runbooks/RB-03-ros2-workflows.md` for the full procedure; the short
version is:

```
wsl
cd /mnt/c/Users/daasa/OneDrive/Desktop/sem\ 5/06-Control-System/PROJECT/06_Cooperative-Adaptive-Cruise-Control
./scripts/ros2_sync_build.sh      # build the workspace
./scripts/ros2_run_demo.sh        # one OS process per vehicle
./scripts/ros2_view_demo.sh       # live 3-D view in rviz2
```

This is the slowest one to get going and the least likely to be needed
for the presentation — its result (the equations survive being split
across twelve operating-system processes, agreeing within 2 %) is already
quoted on slide 17.

---

### 6.9 Regenerating the deck

```
conda run -n cacc python scripts/build_ppt.py
```

Rebuilds `report/CACC_Control_Systems_Presentation.pptx`. A good build
ends with:

```
slides: 20
fit: every slide fits above its font floor
```

If instead it prints `OVERFULL SLIDES`, a page has more content than fits
above its minimum readable font size — that page needs trimming or
splitting. The PDF is exported separately from PowerPoint.

---

### 6.10 If something goes wrong

| Symptom | Cause | Fix |
|---|---|---|
| `ModuleNotFoundError: No module named 'cacc'` | wrong Python environment | use `conda run -n cacc python …`, or `conda activate cacc` first |
| `ModuleNotFoundError: No module named 'pptx'` | same, for the deck build | same — the `cacc` env has it |
| a script can't find `scenarios/…` | wrong working directory | `cd` to the project root first |
| a run seems to hang | the full (non-`--quick`) runs are genuinely slow | add `--quick` while experimenting |
| figures look different from the slides | the slides use the full runs | drop `--quick` |

More in `docs/runbooks/RB-04-troubleshooting.md`.

**Nothing you run can damage anything.** Every script writes to a **new
timestamped folder** under `results/`; none of them overwrite earlier
runs or touch the source. Re-run as often as you like.

## Part 7 — every symbol: what it is called, and how it is calculated

Part 1 says what each term *means*. This part says what it is **called**, how
it is **calculated**, what **value** it takes in our runs, and **which file**
computes it. Five columns, no prose. If someone points at a slide and asks
"what is that?", the answer is one row here.

Values are the reproduction operating point unless stated: ρ = 5,
τ₀ = 0.5 s, k_a = 0.5, k_v = 0.63, k_p = 0.009, h_w = 0.95 s — the paper's
case A / case 1. Every number below was recomputed from `src/cacc/` on
2026-08-19, not copied off a slide.

---

### 7.1 The car and the road

| Symbol | Say it as | How it is calculated | Value here | Lives in |
|---|---|---|---|---|
| `x_i` | "x-i", position of follower *i* | state; `ẋ_i = v_i` | integrated by RK4 | `vehicle.py` |
| `v_i` | velocity of follower *i* | state; `v̇_i = a_i` | cruise `v_ss = 25 m/s` | `vehicle.py` |
| `a_i` | **realised** acceleration | state; `τ·ȧ_i + a_i = u_i` (eq. 1) | — | `vehicle.py` |
| `u_i` | **commanded** acceleration — the control input | the CTHP law, eq. (6); saturated **before** integration | clipped to `[−8, +3] m/s²` | `controllers.py` |
| `τ` | "tau" — actuator lag | the pole of `1/(τs+1)`; pedal-to-motion delay of the inner loop | `0.5 s` | `vehicle.py` |
| `τ₀` | "tau-nought" — worst-case lag | `τ₀ = maxᵢ τᵢ`; the design is certified for **all** `τ ∈ (0, τ₀]` | `0.5 s` | scenario YAML |
| `G(s)` | plant transfer function | `G(s) = 1 / (s²(τs+1))` — double integrator plus lag | two poles at the origin | `analysis.gamma` |
| `L` | vehicle length | constant | `4 m` | `VehicleParams` |
| `r` | standstill gap | the constant part of the spacing policy | `1 m` | `ControllerParams` |
| `d` | standstill spacing | `d = L + r` | `5 m` | `ControllerParams` |
| `N` | platoon size | number of **followers** (the leader is not counted) | `12` | scenario YAML |
| `a₀(t)` | leader manoeuvre | `0.5·sin(0.1(t − 10))` over exactly one period, then constant | peak `0.5 m/s²` | scenario YAML |

### 7.2 The gap rule (spacing policy)

| Symbol | Say it as | How it is calculated | Value here | Lives in |
|---|---|---|---|---|
| `h_w`, `h` | **time headway** | the *seconds* of gap per unit speed in `d_i = r + h_w·v_i` | `0.95 s` case 1 · `0.65 s` case 2 · `0.88 s` case 3 | `ControllerParams.h` |
| `d_i` | desired gap | `d_i = r + h_w·v_i` | `24.75 m` at `h = 0.95`, `v = 25` | `controllers.py` |
| `H(s)` | spacing-policy filter | `H(s) = h_w·s + 1`, so `\|H(jω)\| ≥ 1` for every ω | — | `analysis.gamma` |
| `e_i` | constant-spacing error | `e_i = x_i − x_{i−1} + d` | — | `platoon.py` |
| `δ_i` | "delta-i" — the **CTHP spacing error** | `δ_i = e_i + h_w·v_i` (eq. 2); this is what the controller actually regulates | peak `1.5959 m` at `i = 1` | `platoon.py` |
| capacity | lane throughput | `3600·v / (L + r + h·v)` veh/h | `3130.4 veh/h` | `metrics.throughput_veh_per_hour` |
| platoon length | road occupied | `N·(L + r + h·v)` | `345.00 m` at h = 0.95 → `324.00 m` at h = 0.88 | Case 4 check |

### 7.3 The three gains, and the one number they collapse into

| Symbol | Say it as | How it is calculated | Value here | Lives in |
|---|---|---|---|---|
| `k_a` | "k-a" — **feedforward gain** | multiplies the V2V-received predecessor acceleration | `0.5` case A · `0.3183` case 3 | `ControllerParams` |
| `k_v` | "k-v" — **damping / relative-speed gain** | multiplies `(v_i − v_{i−1})` from radar | `0.63` | `ControllerParams` |
| `k_p` | "k-p" — **spacing gain** | multiplies `δ_i` | `0.009` | `ControllerParams` |
| `γ` | "**gamma**" — the composite damping term | `γ := k_v + h_w·k_p`. **The only place `h_w` enters the transfer function** — and only in the denominator | `0.638550` | `analysis.gamma` |
| `k̃_a` | "**k-a-tilde**" — *effective* feedforward gain | `k̃_a := k_a·E[w]` (eq. 12); what the channel actually delivers on average | `0.524087` | `analysis.expected_w` |
| `I` | "the **noise interval**" | `I = [(1 − 1/ρ)·k_a , (1 + 1/ρ)·k_a]` — every gain the channel can hand you | `[0.4000, 0.6000]` | `analysis`, `certificate` |

> **The one number that shows how thin the guarantee is.** At case A,
> `γ = 0.638550` has to sit between the eq.-(28b) floor `0.638514` and the
> eq.-(28a) ceiling `0.640000`. The entire feasible window is **0.0015 wide**.
> That is "the guarantee is real but thin", quantified.

### 7.4 The radio, and what is wrong with it

| Symbol | Say it as | How it is calculated | Value here | Lives in |
|---|---|---|---|---|
| `w`, `w_{i,i−1}` | "w" — the **channel factor** | `w = (1 − 1/ρ) + (1/ρ)·Σ_{j=0}^{15} z_j·2^{−j}` (eq. 5); the received feedforward is `w·a_{i−1}` | held constant for `10 ms` per draw | `network.py` |
| `z_j` | bit *j* of the channel word | `z_j ~ Bernoulli(γ_j)`, independent | 16 bits | `network.py` |
| `γ_j` | "gamma-j" — **bit expectations** | `E[z_j]`; the paper's 16 fixed values | `MA2025_GAMMAS = (0.8055, 0.5767, …, 0.0714)` | `network.MA2025_GAMMAS` |
| `ρ` | "**rho**" — channel SNR as a **ratio**, not dB | `ρ = 10^{SNR_dB/20}` | `ρ = 5` ⟸ `13.98 dB` | scenario YAML |
| `E[w]` | expected channel factor | `E[w] = (1 − 1/ρ) + (1/ρ)·Σ γ_j·2^{−j}` (eq. 12) | `1.048174` — the channel is **biased optimistic by 4.8 %** | `analysis.expected_w` |
| support of `w` | "**bounded support**" | `1 − 1/ρ ≤ w < 1 + 1/ρ`, **strictly** | `[0.8, 1.2)` | `network.py` |
| `θ` | "**theta**" — V2V delay | enters as `e^{−jωθ}` on the feedforward path only — **exact**, never Padé | budget ≈ `0.23 s` at case-A gains | `analysis.gamma` |
| `p_loss` | packet-loss probability | dropped beacons held by zero-order hold | `0 → 50 %` sweep | `network.py` |
| the bit cache | the ρ-independent realisation store | caches `U′ = Σ z_j·2^{−j}`, **not** `w`, so changing ρ *rescales* the same dice instead of re-rolling them | — | `network.py` |

### 7.5 The verdict — string stability

| Symbol | Say it as | How it is calculated | Value here | Lives in |
|---|---|---|---|---|
| `H̃(s)` | "**H-tilde**" — the string transfer function | `H̃ = (k̃_a·s² + k_v·s + k_p) / (τ·s³ + s² + γ·s + k_p)` (eq. 14), giving `δ̄_i = H̃·δ̄_{i−1}` | — | `analysis.gamma(kind="cthp")` |
| `‖H̃‖∞` | "**the H-infinity norm of H-tilde**" | `sup_ω \|H̃(jω)\|`, evaluated as the max over a frequency grid | `1.000000` case 1 · `1.003500` case 2 | `analysis.hinf_norm` |
| the ω grid | the frequency grid | `np.logspace(−3, 2.5, 8000)` rad/s | 8000 points | `analysis.OMEGA_DEFAULT` |
| `hinf_worst` | worst case over the interval | max of `‖H̃‖∞` over `k̃_a ∈ I`. **MATLAB** grids the interior (41 points by default, 121 in the figures) so the endpoint claim is *checked*; **Python** takes the two endpoints | the **LOW** end binds (D-009) | `certify.py`, `matlab/+ma2025/hinf_worst.m` |
| string stability | Definition 2 | `‖H̃‖∞ ≤ 1` for **all** `τ ∈ (0, τ₀]` **and** all `k̃_a ∈ I`, plus internal stability | — | `analysis.is_string_stable` |
| `H̃(0)` | the DC gain | `k_p/k_p = 1`, always. So `‖H̃‖∞ < 1` is **impossible** — the best achievable is exactly 1, and the low-frequency margin is zero by construction | `1` | — |
| eq. (16) | the `k_a` admissibility bound | `0 < k_a < 1/(1 + 1/ρ)` | upper bound `0.833333` | `cthp_gains_feasible` |
| eq. (28a) | the **γ ceiling** | `γ ≤ [1 − (1 + 1/ρ)²·k_a²] / (2τ₀)` — **contains no `h_w` at all** | `0.640000` | `cthp_gains_feasible` |
| eq. (28b) | the **γ floor** | `γ ≥ √(2·k_p·[1 − (1 − 1/ρ)·k_a] + k_v²)` | `0.638514` | `cthp_gains_feasible` |
| the Routh check | internal stability of `D(s)` | `γ > τ₀·k_p`, from Routh–Hurwitz on `τs³ + s² + γs + k_p` | `0.6386 > 0.0045` ✓ | `cthp_gains_feasible` |
| `h_{w,lb}` | "h-w-l-b" — the **headway lower bound** | `h_lb = 2τ₀·[1 − (1 − 1/ρ)k_a] / [1 − (1 + 1/ρ)²k_a²]` (eq. 17) | `0.937500 s` | `analysis.cthp_h_lb` |
| `k_a*` | "k-a-star" — the **optimal feedforward gain** | `k_a* = [(1 − 1/√ρ)/(1 + 1/√ρ)]·[1/(1 + 1/ρ)]` (eq. 18) | `0.318305` | `analysis.cthp_optimal` |
| `h*_{w,lb}` | "h-star" — the **minimum achievable headway** | `h* = τ₀·(1 + 1/√ρ)² / (1 + 1/ρ)` (eq. 19) | `0.872678 s` | `analysis.cthp_optimal` |
| `min_stable_headway` | the *numerical* headway floor | **bisection** on `is_string_stable(h)`; tolerance `1e−3 s`, bracket `[0.01, 10]`. Used instead of eq. (17) because (28a)/(28b) are **sufficient, not necessary** | `0.789 s` noiseless at case-A gains | `analysis.min_stable_headway` |

### 7.6 The time-domain (empirical) numbers

| Term | Say it as | How it is calculated | Value here | Lives in |
|---|---|---|---|---|
| peak error | "max mod delta-i" | `max_t \|δ_i(t)\|`, per follower | `1.5959 → 1.5436 m` — case 1, attenuating | `metrics.peak_abs_errors` |
| `L₂` error | "the L-two norm" | `‖δ_i‖₂ = √(Σ δ_i(t)²·dt)` | — | `metrics.l2_errors` |
| amplification ratio | "**per-hop L₂ amplification**" | `‖δ_i‖ / ‖δ_{i−1}‖` — one number per *hop*, so `N−1` of them | `< 1` ⟹ attenuating | `metrics.amplification_ratios` |
| empirically string stable | the time-domain pass/fail | **every** hop ratio `≤ 1 + tol`, with `tol = 0.02` | — | `metrics.empirically_string_stable` |
| minimum gap | the **safety** criterion | bumper-to-bumper distance on the **nonlinear** run — actuator saturation sits outside the linear envelope `‖H̃‖∞` describes | `+8.37 m` gated vs `−27.4 m` ungated | `certify.min_gap` |
| accel RMS | comfort / energy proxy | `√(mean_t a_i(t)²)`, per follower | — | `metrics.accel_rms` |

> **Why both.** `‖H̃‖∞` is the *verdict*; the L₂ ratios are the *evidence*. A
> per-hop gain of 1.0035 compounds to only 1.039× over 11 vehicles — invisible
> in any time trace, which is exactly why the verdict is taken in frequency.

### 7.7 The six things we added

| # | Name | The quantity | How it is calculated | Value here | Lives in |
|---|---|---|---|---|---|
| 1 | **Predictor** | `P(s)` | `P(s) = 1 + θ̂·A(s)·(1 − e^{−sT/2})/(T/2)` with `A(s) = (1 − e^{−sT/2})/(sT/2)`; two `T/2` moving-average blocks estimate the slope of the received signal, and `θ̂` seconds of it are added back | `T = pred_base = 0.4 s`; at `θ = 0.15 s` it moves `‖H̃‖∞` from `1.00923 → 1.00000` | `analysis.gamma(pred_theta_hat=…)` |
| 2 | **Estimator** | `ŵ`, `ρ̂` | `ŵ = y_received / a_radar(t − θ̂)`, keep `\|ŵ − 1\|`, then `ρ̂ = clip(1 / Q₉₉(\|ŵ − 1\|), 1.5, 50)` | `≥ 25` samples required, window aged out at `20 s`, samples with `\|a_radar\| < 0.03 m/s²` or `ŵ ∉ (0.2, 1.8)` rejected, `25 Hz` | `estimation.ChannelEstimator` |
| — | *why it is safe* | "conservative by construction" | `\|w − 1\| ≤ 1/ρ` **strictly**, so **any** quantile under-estimates `1/ρ` and therefore **over**-estimates `ρ`. The adapter divides by `rho_safety` to push it back to the safe side | `rho_safety = 1.2` | `estimation.py` |
| 3 | **Adapter** | `h(t)` | `target = min(h_max, margin + h_req(ρ̂/1.2, θ))`, then `h ← h + clip(target − h, ±rate·dt)` | `margin = 0.08 s`, `rate = 0.05 s/s`, `h_max = 2.5 s` | `estimation.HeadwayAdapter` |
| — | `h_req` | required headway at **fixed** gains | bilinear interpolation of a table bisected once over `ρ ∈ {1.5 … 50}` × `θ ∈ {0 … 0.30}`, worst case over **both** ends of `I`. Deliberately **not** eq. (17), which presumes the gains are re-tuned for each ρ | an 11 × 7 table | `HeadwayAdapter._build_table` |
| 4 | **Scheduler** | `k_v(ρ̂)`, `k_p(ρ̂)` | `k_v(ρ) = kv_frac·[1 − (1 + 1/ρ)²k_a²]/(2τ₀)` — ride at 90 % of the (28a) ceiling; `k_p(ρ) = k_p0·k_v(ρ)/k_v0` holds the ratio; both rate-limited like `h` | `kv_frac = 0.9`; it reproduces the paper's own `k_v = 0.63` at `ρ = 10` (gives `0.628`) | `estimation.GainScheduler` |
| 5 | **Trust gate** | `r`, `g`, `u_ff_eff` | `r = \|a_V2V − a_radar\|` · `g(r) = 1/(1 + (r/r₀)²)` ∈ (0, 1] · `u_ff_eff = g·a_V2V + (1 − g)·a_radar` | `r₀ = 3.0 m/s²` | `trust.TrustGate` |
| — | the **certificate** | bounded injection | `\|u_ff_eff − a_radar\| = g(r)·r = r/(1 + (r/r₀)²) ≤ r₀/2` for every `r ≥ 0`, equality at `r = r₀` — **independent of how big the lie is**. So the spurious command is at most `k_a·r₀/2` | `≤ 0.75 m/s²` at `k_a = 0.5` | `trust.py` |
| 6 | **Certificate** | `ChannelLaw` | the **exact** law of `w`: iterated convolution of the 16 two-point bit laws, giving `2¹⁶ = 65 536` atoms `(w, p)` | exact, not sampled | `certificate.ChannelLaw.build` |
| — | `exceedance(h)` | the risk | `A(w) := ‖H̃(·; k̃_a = w·k_a)‖∞` is monotone **decreasing** in `w`, so the unstable set is a clean lower tail: `exceedance(h) = P(A(w) > 1) = P(w < w_crit(h)) = F(w_crit(h))` | analytic and Monte-Carlo agree to 3–4 decimals | `certificate.py` |
| — | `h_cc(ε)` | **chance-constrained headway** | `h_cc(ε) = min{h : exceedance(h) ≤ ε}`, i.e. `min_stable_headway` evaluated at the **ε-quantile of `w`** | at ρ = 3: `1.0334 s` (ε ≈ 1e−2) and `1.0522 s` (1e−3) against worst case `1.1544 s` ⟹ **−10.5 %** | `certificate.py` |
| — | `h_ms` | mean-square headway | the smallest `h` with `E[A(w)²] ≤ 1` | `0.9944 s` at exceedance `0.076` — **reported and rejected**, far too permissive to certify with | `certificate.py` |
| — | `h_cheb` | distribution-free (**Cantelli**) | closed form using only `E[w]` and `Var[w]` — robust to uncertainty in the `γ_j` | a conservative companion | `certificate.py` |
| — | `ρ(x)`, preview | the **QoS map** (flagship A) | `rho_at(x)` gives channel quality by **position**; `min_rho_ahead(x, v, H)` returns the worst ρ over `[x, x + v·H]`, and the adapter targets the **worse** of measured and previewed | `preview_s = 45 s` in study 5A | `qos_map.QoSMap` |

### 7.8 Two things to say correctly if you are asked

1. **The trust gate is rational, not a linear ramp.** It is
   `g(r) = 1/(1 + (r/r₀)²)`, with **one** parameter `r₀`. The `r₀/2`
   injection cap is a property of *that* function; a two-threshold linear
   clip would not give it. *(The deck printed a clip form until 2026-08-19;
   `scripts/ppt/content_theory.py` and the built `.pptx` are now corrected.
   The `.pdf` in `report/` is exported by hand from PowerPoint, so it stays
   one revision behind until you re-export it.)*

2. **"Grids `k̃_a` across `I`" describes the MATLAB backend.**
   `matlab/+ma2025/hinf_worst.m` sweeps `nGrid` points across the interval so
   the endpoint claim is checked rather than assumed. The Python hot paths
   (`certify.py`, `predictive_qos_study.py`) take the max over the **two
   endpoints**, which gives the same answer because `|H̃|` at the peak
   frequency is monotone in `k̃_a`. Both find the **low** end binding.

*(One more, in case it comes up: Part 3 of this guide walks 20 slides, but the
deck now builds **21** — the ROS 2 distributed-backend page sits between the
live demo and the reproduction table.)*

---

---

*Describes the 20-slide deck built by `scripts/build_ppt.py`. If the deck
changes, this file needs the same edit — Part 3 is positional. Commands in
Part 6 were run and verified on 2026-08-19.*
