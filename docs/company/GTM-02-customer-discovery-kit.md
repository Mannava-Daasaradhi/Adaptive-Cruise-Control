# GTM-02 — Customer discovery kit

> **Summary.** Everything needed to run 10 discovery conversations in two
> weeks: the hypotheses being tested, who to contact and how to find them,
> outreach messages ready to personalize, a 30-minute interview script that
> avoids leading questions, and a log template that turns calls into
> decisions. Nothing here is sent automatically — you send it.

## 1. Hypotheses under test

| id | hypothesis | confirmed if | killed if |
|---|---|---|---|
| H1 | ADAS/V2X teams have **no routine string-stability sign-off** for longitudinal controllers | ≥ 5 of 10 say it is checked ad hoc or not at all | most have an established method they are happy with |
| H2 | V2X vendors are **asked by OEMs** what latency/loss does to controllers | ≥ 2 of 4 V2X people describe such a request | none recall it |
| H3 | Drive logs (10 Hz speed + gap) **are available** and can be shared under NDA | ≥ 3 would share logs for a pilot | logs never leave the building and on-prem is refused too |
| H4 | A **2–4 week audit at €6k–15k** fits a team-lead budget | ≥ 2 say "I could buy that" without procurement committee | everyone needs a 6-month vendor process |
| H5 | Regulatory attention (JRC OpenACC, UNECE) **creates urgency** | unprompted mentions of regulation / traffic impact | nobody cares until it is mandated |
| H6 | **FMU** is the format controllers can leave the org in | ≥ 3 mention FMU/Simulink export | a different interface dominates (e.g. C library, ROS) |

## 2. Who to contact

| segment | roles to search | example organizations (public knowledge; verify current teams) |
|---|---|---|
| V2X vendors | "C-V2X" + product manager / systems engineer / applications engineer | Qualcomm, Commsignia, Cohda Wireless, Harman (Savari), Danlaw |
| ADAS Tier-1s | "adaptive cruise control" / "longitudinal control" / "ADAS validation" engineer, team lead | Bosch, Continental, ZF, Aptiv, Valeo, Denso, Magna, Hyundai Mobis |
| OEM ADAS teams | same keywords | VW Group / CARIAD, Stellantis, Toyota, GM, Ford, Hyundai, Tata Motors, Mahindra |
| Engineering services | ADAS validation practice lead / delivery manager | KPIT Technologies, Tata Elxsi, L&T Technology Services, Capgemini Engineering |
| Simulation tool vendors (channel) | partner manager / product manager, ADAS | IPG Automotive, dSPACE, AVL, Vector, Foretellix, Applied Intuition |
| Convoy / platooning | autonomy / controls lead | Kratos, Forterra, Carnegie Robotics, AV-trucking companies |
| Research & test sites (credibility, data) | researchers on car-following / ACC | JRC (OpenACC authors), TNO, TU/e, AstaZero, ZalaZONE, Mcity |

**Finding people:** LinkedIn search `"adaptive cruise control" AND
(validation OR "longitudinal control")`, filter by company; conference
speaker lists (IEEE ITSC, IEEE IV, ITS World Congress, ADAS & Autonomous
Vehicle Technology Expo); authors of recent string-stability or OpenACC
papers (they know who has the problem). Warm intros beat cold: ask every
call "who else should I talk to?".

## 3. What to have ready before the first call

1. A 2-minute screen recording of the demo (GTM-03 steps 1–4).
2. The P-04 one-page argument, and the README quick start.
3. A calendar link; 30-minute slots.

## 4. Outreach messages (personalize the first line every time)

**A. Cold email — ADAS validation engineer**

> Subject: string stability of ACC — 20 minutes of your experience?
>
> Hi {first name} — I saw {their talk / paper / post on X}. I'm building a
> tool that measures whether an ACC controller amplifies traffic waves
> (string stability) from a single drive log, with a what-if for V2V
> feedforward. JRC's OpenACC tests found every commercial ACC they measured
> string-unstable, and I'm trying to learn how validation teams deal with
> that today.
>
> I'm not selling anything yet — could I ask you about your current process
> for 20 minutes next week? Happy to share what I've found so far.
>
> {name} · {link to repo or 2-minute demo}

**B. Cold email — V2X product manager**

> Subject: what does 100 ms of V2X latency do to a controller?
>
> Hi {first name} — when OEMs ask what your latency and packet-loss figures
> mean for cooperative cruise control, what do you show them? I've built an
> engine that answers that at controller level (string stability vs.
> latency, loss and channel noise, reproducible from one config file) and
> I'm talking to V2X teams to learn whether that evidence would help. Would
> you have 20 minutes for my questions?

**C. LinkedIn connection note (≤ 300 characters)**

> Hi {first name} — I'm researching how ADAS teams validate ACC string
> stability (the JRC OpenACC finding). Your work on {X} is exactly what I'm
> trying to learn about — would you be open to a 20-min chat? Not selling.

**D. Follow-up (5 working days later, once)**

> Hi {first name} — bumping this once in case it got buried. Even 15
> minutes would help; and if someone on your team owns this topic, a
> pointer would be just as welcome.

## 5. Interview script (30 minutes, ask about the past, not the future)

**Opening (2 min).** Purpose: learn how they work today. No pitch until the
end.

**Their world (8 min)**
1. Walk me through the last time you validated a longitudinal controller
   change. What was checked, with which tools, who signed off?
2. What happened the last time a platoon or car-following behavior surprised
   you — in simulation, on track, or from the field?
3. Where does V2X (if at all) enter your controller work today?

**The problem (10 min)**
4. How do you currently judge whether a controller amplifies disturbances
   down a line of cars? When did you last do that?
5. What did it cost (time, track days, people) the last time?
6. What data do you have from vehicles (rate, signals, how long)? Could any
   of it leave the building under NDA, or would it need to stay on-prem?
7. Has anyone — management, an OEM customer, a regulator — asked you about
   traffic impact or string stability? What exactly did they ask?

**Budget and process (5 min)**
8. When your team last bought a tool or an outside study, how did that
   happen? Who approved it, and roughly what size was it?
9. What would have to be true for you to try a 2–4 week pilot?

**Close (5 min)** — only now, show the 2-minute demo; ask:
10. What's the first thing you'd want this to tell you about your car?
11. Who else should I talk to?

**Rules:** no hypotheticals ("would you use…") — ask what they *did*;
count commitments (time, data, intros, money), not compliments; write the
log within one hour.

## 6. Call log template (one per call, keep in a private folder)

```
date / person / role / company / segment
how they validate longitudinal control today:
last surprising platoon / car-following event:
string stability: never | ad hoc | routine  (quote)
V2X relevance: none | future | active  (quote)
data: none | internal only | shareable under NDA ; rate/signals:
who asked about traffic impact / regulation (quote):
buying: last purchase, size, approver:
hypotheses: H1 _ H2 _ H3 _ H4 _ H5 _ H6 _   (+ confirmed, − killed, ? unclear)
commitment offered: none | follow-up | data | intro | pilot
next step + date:
```

After every 3 calls: re-rank segments, update the hypotheses table, and
change the outreach message if a better phrasing of the pain appeared.

## Revision history
- 2026-09-25 — created (D-026 sprint).
