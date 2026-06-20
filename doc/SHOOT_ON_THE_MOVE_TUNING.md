# Shoot-on-the-Move (SOTM) — Understanding & Tuning Guide

This document has three parts:

1. **How it works** — the intuition behind every calculation, in plain language.
2. **At the lab** — a step-by-step checklist to run with the robot in front of you, using
   the **`SOTM Tuner`** opmode (no re-deploys between adjustments).
3. **Reference** — every variable, what it does, and a troubleshooting table.

> **Golden rule:** change **one** variable at a time, take ~5 shots, note what happened.
> SOTM has many knobs and they interact. The `SOTM Tuner` opmode is built so you never
> have to leave the field or re-deploy to change anything.

---

# Part 1 — How it works (the intuition)

## 1.1 The static shot (robot standing still)

Everything starts here. The trajectory model (`Shooter.updateKinematics`) answers one
question: *"From my current distance to the goal, what artifact speed and hood angle land
the ball in the goal?"* It returns a target RPM and a hood angle, looked up from distance.

If the robot never moved, that's all you'd need. Two knobs shape it:

- **`RPM_MULTIPLER`** is the master "how hard do I shoot" scale. Our robot has **no counter
  rollers** (a single flywheel), so an artifact leaves at roughly *half* the wheel's surface
  speed. `RPM_MULTIPLER ≈ 2` is what converts the physics speed into the RPM you actually
  command. If *every* shot (close and far) is short, raise it; if every shot is long, lower it.
- **The hood LUT** (`Shooter.getHoodAngle`) sets the launch arc vs. distance. If close shots
  are good but far shots are wrong (or vice-versa), the LUT's slope is off — that's a code edit.

## 1.2 The core SOTM trick: aim at a "virtual target"

When the robot is moving and you release a ball, **the ball keeps your robot's velocity**
(like dropping a ball while running — it travels forward with you). So if you aim straight
at the goal while strafing right, the ball drifts right and misses.

The fix is a beautifully simple idea. Instead of aiming at the goal, aim at a **virtual
target**:

```
virtual target = goal − (robot velocity) × (time the shot takes)
```

Solve the *static* shot to that virtual point. Then the ball's own velocity carries it the
rest of the way into the **real** goal. One trick handles everything:

- **Direction (lead):** strafing right → virtual target is shifted left → turret aims left
  → ball drifts right into the goal. Works for any direction you move.
- **Speed:** driving *toward* the goal → virtual target is *closer* → the model commands a
  *lower* RPM (because the ball already has your forward speed). Driving away → farther →
  more RPM. Automatic.
- **Standing still:** velocity = 0 → virtual target = goal → it's just the static shot. SOTM
  never hurts you when stopped.

In code this is one line: `Configuration.setAimPointOffset(−vx·t, −vy·t)`, which shifts the
goal that both the turret and the shooter already aim at.

## 1.3 Why "time the shot takes" is the master knob

That `t` above is **flight time + system lag**:

```
t = tof (ball in the air)  +  SHOT_LATENCY (effective system lag: servo speed, feed, processing)
```

`SHOT_LATENCY` is a **tuned** knob, not a derived number. In theory it'd be a tiny actuator
latency, but **on this robot the real effective lag is large — about 0.5–0.8 s** (the turret
servo is slow, etc.). Don't trust the clean theory here; trust the strafe test:

> **Strafe test (the one that matters):** strafe sideways at a steady speed and shoot.
> - Ball lands **in the same direction you're strafing** → aim is **UNDER-leading** → **raise `SHOT_LATENCY`**.
> - Ball lands **opposite your strafe** → **over-leading** → **lower `SHOT_LATENCY`**.
> - Ball lands in the goal → you're dialed.

The lead distance is `velocity × t`, so `SHOT_LATENCY` (and a good static `tof`) is the
master knob for all moving shots — get it right before touching the gains.

## 1.4 Rotation: leading the spin

The turret re-aims at the goal **every loop**, so it doesn't need to "lead" rotation the
way it leads translation — *except* for its own reaction lag. A servo takes ~0.1–0.2 s to
get where it's told. If the robot is spinning at rate ω, by the time the turret arrives the
robot has rotated `ω × latency` further, so the turret is always that far behind.

We cancel it by pre-rotating the turret by `ω × TURRET_LEAD_TIME` (plus a tiny angular-accel
term). Note this uses the **small servo-latency window**, *not* the big feed time — using
the feed time here would massively over-lead. This is the **rotational SOTM**.

## 1.5 Acceleration: predicting the near future

Velocity isn't constant. If you're accelerating, your velocity *at the moment the ball
launches* (`T` from now) is higher than right now. So we extrapolate: `v_launch = v + a·T`,
and use that for the lead. Same idea for spin (angular acceleration).

The catch: **acceleration is a noisy signal** (it's the derivative of velocity, which is
already a derivative of position). So it's filtered and gated, and you can dial it down or
off (`ACCEL_GAIN = 0`) if it causes more jitter than it's worth. Velocity + rotation alone
is already very good; accel is the cherry on top for aggressive driving.

## 1.6 Rapid-fire RPM droop, and hood compensation

Firing 3 balls in a row yanks energy out of the flywheel faster than the motors (even with
a 1.4 kg flywheel) can put it back, so RPM **sags** between shots. A slower wheel shoots
slower → the ball lands **short**.

We don't fix this by over-spinning the wheel (that would overshoot when you're *not*
drooping). Instead we **measure the actual RPM** and, when it's below target, **loft the
hood** toward the angle that needs less speed — so the slower ball still reaches the goal.
The direction is read from the trajectory model itself, so it's always correct.

**Limit to be honest about:** for *far* shots the hood is already near its most-efficient
angle, so lofting can't add much range — a far shot that's short on RPM fundamentally needs
wheel speed. For those, your options are flywheel inertia (already done mechanically) or
gating the shot until RPM recovers (`isUpToSpeed()`, see §Reference).

## 1.7 Gate-3 motor protection (why it burned out)

The third-gate feed motor (`transferMotor2`) pushes balls toward the shooter. If a ball
jams against its wheel, the motor **stalls** — and a stalled motor draws huge current,
heats up, and burns out. The old code only auto-stopped the motor in teleop and **skipped
the check entirely in autonomous** (where it actually burned out).

Now there's a **current-based stall cutoff that runs in every mode**: if the motor draws
more than `GATE3_STALL_CURRENT` amps for `GATE3_STALL_TIME` seconds, it cuts power, treats
it as "ball present," and holds off until you open/close the gate. You tune the threshold by
watching the live current on telemetry.

---

# Part 2 — At the lab (step-by-step)

Run the **`SOTM Tuner`** opmode (group "Debug"). It lets gamepad 1 drive and fire while
gamepad 2 scrolls a cursor through every parameter and nudges it live.

### Controls

```
GAMEPAD 1 — DRIVER (move + fire)
  Left stick      drive / strafe
  Right stick X   rotate
  Right bumper    FIRE (hold = open gate, release = close)
  A / B           intake on / off
  D-pad Left      relocalize to the alliance test pose (repeatable start spot)
  Back            shooter on / off (toggle)

GAMEPAD 2 — TUNER (live, no re-deploy)
  D-pad Up/Down     select previous / next parameter (cursor ">" on telemetry)
  D-pad Right/Left  increase / decrease selected by 1 step
  Right/Left bumper increase / decrease by 10 steps (coarse)
  Y                 ROTATION layer on/off   (ROTATION_GAIN 0 <-> last value)
  B                 FLIP rotation sign      ← do this if the turret leads the WRONG way
  X                 ACCEL layer on/off      (ACCEL_GAIN 0 <-> 1)
  A                 master predictive ENABLED on/off
  Start             toggle alliance RED / BLUE
```

> ⚠️ **Live edits are NOT saved.** The parameters are in-memory only and reset when the
> opmode restarts. When you're happy with a value, **write it down**, and after the session
> copy the finals into the source defaults (`AimController`, `Shooter`, `Configuration`,
> `Transfer`). See §3 for which file each one lives in.

### Pre-flight (before you touch a stick)
- [ ] Battery fresh (RPM droop and servo speed both depend on voltage — a sagging battery
      will lie to you while tuning).
- [ ] Several artifacts staged; clear space downrange; know exactly **which goal** you're
      shooting at.
- [ ] On gamepad 2, press **Start** until **Alliance** on telemetry matches your goal.
- [ ] Select `SOTM Tuner`, press INIT, then START.

### Step 0 — Localization sanity check (do NOT skip)
SOTM aims using the robot's odometry pose. If the pose is wrong, every shot is wrong and
you'll waste an hour tuning the wrong thing.
1. Drive a lap around the field. Watch the pose on telemetry / Dashboard.
2. Confirm X, Y, and heading roughly match where the robot actually is.
3. Park on your known relocalize spot and tap **G1 D-pad Left** — pose should snap to the
   expected coordinates. ✅ when pose tracks reality. ❌ then fix odometry/Pinpoint first.

### Step 1 — Static shot (stationary)
The predictive layers (rotation, accel) are **ON by default**. To isolate the basic case
while tuning the foundation, toggle them OFF with **G2 Y** (rotation) and **G2 X** (accel)
— or **G2 A** for the master switch — then turn them back on for Steps 3–4.
1. Park at a **known mid distance**. Hold **G1 A** to intake a ball, then **G1 RB** to fire.
2. Cursor to **`RPM_MULTIPLER`** (G2 D-pad Up/Down). Nudge with **G2 D-pad Right/Left** until
   shots are centered front-to-back: short → increase, long → decrease.
3. **Arc too high / too loopy?** Cursor to **`HOOD_TRIM`** and make it **negative**. That
   flattens the entry angle, and the trajectory solve **automatically raises the RPM** —
   exactly "less arc, more RPM, sharper shot." Then re-center distance with `RPM_MULTIPLER`.
4. If shots are biased **left/right** while stationary, cursor to **`Mech TURRET_OFFSET`** and
   nudge it out.
5. Re-test at a **close** and a **far** distance. If you can center mid but close/far are off
   in opposite ways, the **hood LUT** slope needs a code change (note it; can't tune live).
- ✅ Done when stationary shots are consistent, centered, and at an arc you like.

### Step 2 — Driving shots (translation)
Leave prediction off (velocity SOTM works without it). The virtual-target lead is automatic;
you're really just dialing the **time** knob.
1. Pick a steady speed. Drive **straight at** the goal and fire; then straight **away** and
   fire; then **strafe** left/right and fire; then diagonally.
2. Cursor to **`SHOT_LATENCY`** and run the **strafe test**:
   - Ball lands **in the strafe direction** (under-lead) → **increase** `SHOT_LATENCY`.
   - Ball lands **opposite** the strafe (over-lead) → **decrease** it.
   Default 0.70 s; expect ~0.5–0.8 s. (Driving straight toward/away mostly changes RPM via the
   virtual-target distance — that's automatic; focus the lead tuning on strafing.)
3. Re-check stationary still works (it should — the knob scales with velocity).
- ✅ Done when shots land in the goal while translating at a constant speed in any direction.
- The direction of the lead should already be correct; if a *strafe* misses sideways but
  driving toward/away is fine, suspect the **heading/pose** (back to Step 0), not a gain.

### Step 3 — Spinning shots (rotation)
Rotation lead is **ON by default** (`ROTATION_GAIN = +1`, sign confirmed on-robot). Watch
**`Turret lead (deg)`**, **`omega`**, and **`Turret in range`** on telemetry while you spin.
1. **Slow** spin: the turret should hold the goal. ✅ if it does.
2. **Fast** spin: if the turret freezes mid-spin and snaps to the goal when you stop, read
   **`Turret in range`** during the freeze to see which limit you hit:
   - **`= false`** → the goal swung past the turret's reach. The default range is only about
     **±108°** from robot-forward, set by **`MIN_SERVO_POS` / `MAX_SERVO_POS`** (now in the
     param list). If your turret can physically rotate further, **widen them live**: select
     one, nudge it out 0.02 at a time, and watch the turret reach further — **STOP if the
     servo buzzes/strains** at the end (that's the real mechanical stop). `[0.04, 0.93]` ≈
     ±155–165°. If it's already at the hard stop, it's geometric — keep the goal in front.
   - **`= true`** but lagging → the **servo slew-rate limit**: the turret can't counter-
     rotate as fast as you're spinning, and a feed-forward lead can't beat that. Mitigate by
     spinning slower while aiming, raising **`TURRET_LEAD_TIME`** (more anticipation — helps
     the transient/heading-lag part), or a faster turret servo (mechanical).
   The lead clamp is 45° and ω is fairly responsive to push this as far as the hardware allows.
3. Once the direction is right: if the turret still **lags**, cursor to **`TURRET_LEAD_TIME`**
   (or `ROTATION_GAIN`) and increase. If it **over-leads / wobbles**, decrease.
4. If the turret **jitters while you're NOT spinning**, raise **`ANG_VEL_DEADBAND`** (or lower
   `OMEGA_LOWPASS`).
- ✅ Done when the turret holds the goal through a steady spin, both directions.

### Step 4 — Accelerating shots (the refinement)
Accel is **ON by default** (`ACCEL_GAIN = 1`). It's the noisiest signal, so if shots jitter
while driving hard, toggle it OFF with **G2 X** to compare, and lower `ACCEL_LOWPASS` if you
keep it on.
1. Do a "jab" shot: start from a stop, **accelerate hard and fire almost immediately**; also
   try coming off a fast drive and firing as you brake.
2. Compare against accel-off (press X again). Keep it on only if it **clearly** helps.
3. If the aim **jitters while driving**, lower **`ACCEL_LOWPASS`** (smoother) or raise
   **`ACCEL_DEADBAND`**; if that doesn't settle it, just leave **`ACCEL_GAIN` = 0**.
- ✅ Done when hard accel/decel shots are at least as good as constant-speed, with no jitter.

### Step 5 — Rapid fire (RPM droop)
1. Load 3 artifacts. Fire all three as fast as the gate allows while watching **Target / Cur
   RPM** on telemetry — you'll see Current RPM **sag** below Target on balls 2 and 3.
2. If the later balls land **short**, cursor to **`HOOD_DROOP_GAIN`** and increase a little.
   Re-test. If they start landing **long**, back off.
3. Remember the limit: at **far** range the hood can't fully recover a big RPM sag. If far
   rapid-fire is the problem, consider re-enabling the up-to-speed gate (§3) so the robot
   waits a beat between far shots.
- ✅ Done when all three rapid shots group acceptably at your fighting distance.

### Step 6 — Gate-3 motor protection (safety)
Watch the **`Gate3 motor (A)`** value the whole time here.
1. Run the intake normally (**G1 A**) with the path clear — note the **free-running** current
   (small).
2. Carefully let a ball sit against the gate-3 wheel so the motor loads up — note the
   **stalled** current (much higher). *Don't hold it long.*
3. Cursor to **`GATE3_STALL_CURRENT`** and set it **between** those two readings (closer to
   the stall value). Set **`GATE3_STALL_TIME`** so a normal shot's brief load doesn't trip it
   (~0.4 s is a good start).
4. Verify: jam a ball → **`Gate3 stalled` flips to true** and the motor cuts within your
   `GATE3_STALL_TIME`. Fire normally → it does **not** trip.
- ✅ Done when a real jam cuts the motor quickly but normal intake/shooting never trips.

### Wrap-up
- [ ] Write down every final value (telemetry shows the full list under "ALL PARAMS").
- [ ] Copy them into the source defaults so they survive a restart (see §3 for files).
- [ ] Re-deploy and do one confirmation run of each step.

---

# Part 3 — Reference

### Where each variable lives

| Variable | File | Tuned by |
|---|---|---|
| `RPM_MULTIPLER`, `SHOOTER_HEIGHT_TO_GOAL` | `Configuration` | SOTM Tuner / code |
| `SHOT_LATENCY` (translation lead), `ROTATION_GAIN`, `ACCEL_GAIN`, `ENABLED`, `TURRET_LEAD_TIME`, deadbands, low-passes, clamps | `AimController` | SOTM Tuner / code |
| `HOOD_TRIM` (arc), hood LUT (`getHoodAngle`), `RPM_TOLERANCE`, `isUpToSpeed`, droop knobs (`HOOD_DROOP_GAIN`, `MAX_HOOD_DROOP_COMP`, `DROOP_RPM_DEADBAND`, …) | `Shooter` | SOTM Tuner / code |
| `GATE3_STALL_CURRENT`, `GATE3_STALL_TIME` | `Transfer` | SOTM Tuner / code |
| mechanical `TURRET_OFFSET` (per-opmode) | each opmode | SOTM Tuner (live) / code |

### Timing
| Variable | Meaning | Typical |
|---|---|---|
| `SHOT_LATENCY` (`T`) | Master translation-lead window. Total lead = `tof + SHOT_LATENCY`. Empirically large on this bot (slow servo etc.); tune by the strafe test. | 0.5–0.8 s |
| `TURRET_LEAD_TIME` (`Tr`) | Turret servo/control latency — the rotation-lead window only. | 0.10–0.20 s |
| `HOOD_TRIM` | Global entry-angle offset. **Negative = flatter arc + more RPM** (sharper shot). In `Shooter`. | 0 (− to flatten) |
| `FALLBACK_TOF` / `MAX_TOF` | Lead before a valid flight time exists / clamp on it. | 0.3 / 1.5 |

### Rotation
| Variable | Effect | Symptom → fix |
|---|---|---|
| `ROTATION_GAIN` | Scales the rotational lead; sign sets direction. | Lags spinning → raise. Over-leads/wobbles → lower. **Wrong way → negate (G2 B).** |
| `TURRET_LEAD_TIME` | The lead window. | Set to real servo latency, trim with gain. |
| `OMEGA_LOWPASS` | Smoothing on measured spin rate. | Jittery lead → lower; sluggish → raise. |
| `ANGULAR_VELOCITY_DEADBAND` | Ignores small spin-rate noise. | Turret jitters at rest → raise. |
| `MAX_TURRET_LEAD_DEG` | Clamp on the lead. | Leave ~20° unless you spin very fast. |

### Acceleration
| Variable | Effect | Symptom → fix |
|---|---|---|
| `ACCEL_GAIN` | Scales all accel terms; **0 disables**. | Start 0.5 → 1.0 only if it helps. |
| `ACCEL_LOWPASS` | Smoothing on linear accel. | Jitter while driving → lower. Sluggish → raise. |
| `ACCEL_DEADBAND` / `MAX_ACCEL` | Ignore small accel / clamp. | Noise leaks at constant speed → raise deadband. |

### Translation
| Variable | Effect | Symptom → fix |
|---|---|---|
| `VELOCITY_DEADBAND` | Ignores tiny velocity so noise doesn't move the aim at rest. | Aim twitches stopped → raise. Slow motions ignored → lower. |
| `MAX_AIM_OFFSET` | Clamp on the virtual-target offset. | Leave unless aim snaps at extreme speed. |

### Rapid-fire droop (`Shooter`)
| Variable | Effect | Symptom → fix |
|---|---|---|
| `DROOP_COMP_ENABLED` | Master on/off for hood droop comp. | — |
| `HOOD_DROOP_GAIN` | Hood degrees added per RPM of deficit. | 2nd/3rd ball short → raise; long → lower. |
| `MAX_HOOD_DROOP_COMP` | Clamp (deg). | ~6°. |
| `DROOP_RPM_DEADBAND` | Deficit below which no comp. | Set above steady-state RPM error. |
| `DROOP_MIN_RPM_FRACTION` | Don't compensate below this fraction of target (ignores spin-up). | ~0.6. |
| `isUpToSpeed()` | Currently hardcoded `true`. Restore the commented RPM-tolerance check to **gate** firing until the wheel recovers (helps far rapid-fire). | — |

### Gate-3 motor protection (`Transfer`)
| Variable | Effect | How to set |
|---|---|---|
| `GATE3_STALL_CURRENT` | Amps above which (sustained) the motor cuts. | Between free-run and stall draw from `Gate3 motor (A)` telemetry. |
| `GATE3_STALL_TIME` | Seconds of overcurrent before cutting. | ~0.4 s — past the startup spike and a normal shot's load. |

> Tip: if motor-current reads look laggy or cost loop rate, enable
> `LynxModule.BulkCachingMode.AUTO` in opmode init.

### Troubleshooting quick reference
| Symptom | Most likely knob |
|---|---|
| Everything short/long, even stationary | `RPM_MULTIPLER` (or hood LUT) |
| Arc too high / too loopy, want it sharper | `HOOD_TRIM` negative (flattens + adds RPM) |
| Ball drifts in the SAME direction you strafe | `SHOT_LATENCY` too small (under-lead) — **increase** |
| Ball drifts OPPOSITE your strafe | `SHOT_LATENCY` too big (over-lead) — decrease |
| RPM drops as you strafe/drive toward goal | normal — the virtual target got closer; not a bug |
| Turret freezes during a *fast* spin, catches up when stopped | `Turret in range`=false → goal past reach (default ±108°; widen `MIN/MAX_SERVO_POS` if the servo allows); =true → servo slew limit (spin slower / raise `TURRET_LEAD_TIME`) |
| Turret pins to one side and never follows the goal | goal is outside the turret range from your pose/heading (`Turret in range`=false): face the goal, or widen `MIN/MAX_SERVO_POS` |
| Strafe misses sideways, radial is fine | pose/heading wrong (Step 0), not a gain |
| Turret lags when spinning | `ROTATION_GAIN`/`TURRET_LEAD_TIME` up (check sign!) |
| Turret leads wrong way when spinning | **negate `ROTATION_GAIN` (G2 B)**, then reduce |
| Aim jitters at a standstill | `VELOCITY_DEADBAND` / `ANGULAR_VELOCITY_DEADBAND` up |
| Aim jitters only while driving hard | `ACCEL_GAIN` down (toggle G2 X) or `ACCEL_LOWPASS` down |
| 2nd/3rd rapid ball short | `HOOD_DROOP_GAIN` up (close/mid); gate `isUpToSpeed()` for far |
| Gate-3 motor cuts during a normal shot | `GATE3_STALL_TIME` / `GATE3_STALL_CURRENT` up |
| Gate-3 motor never cuts on a jam | `GATE3_STALL_CURRENT` down toward free-run draw |

### Bring-up sequence (TL;DR)
1. **Localization** sane (drive a lap, relocalize).
2. Static shot (`RPM_MULTIPLER`, `HOOD_TRIM` for arc, mech offset). *(Predictive is off by default.)*
3. `SHOT_LATENCY` → strafe + radial driving shots.
4. Rotation **on (G2 Y)** → **sign check (G2 B)**, then `TURRET_LEAD_TIME`.
5. Accel **on** (0.5→1.0) → accel filters; back to 0 if it hurts.
6. Rapid fire → `HOOD_DROOP_GAIN` (+ `isUpToSpeed` gate for far).
7. `GATE3_STALL_CURRENT` / `GATE3_STALL_TIME` from the live current readout.
