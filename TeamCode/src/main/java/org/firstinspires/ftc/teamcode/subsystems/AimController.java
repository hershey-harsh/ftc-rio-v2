package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Configuration;

/**
 * Centralized shoot-on-the-move (SOTM) aiming, shared by every teleop opmode so the
 * competition opmodes and the FLL demo behave identically (no copy-paste drift).
 *
 * <h2>Method: predictive, acceleration-based virtual target</h2>
 * The lead a moving robot needs is the drift of the shot over the time it is "uncorrectable":
 * the <em>flight time</em> {@code tof} plus a system-lag term {@code T}
 * ({@link #SHOT_LATENCY}). In a clean model {@code T} would just be small actuator latency,
 * but EMPIRICALLY this robot needs {@code T} ≈ 0.5–0.8 s — the effective lag (turret servo
 * speed, feed, processing) is much larger than a tidy model predicts. So treat {@code T} as
 * a TUNED knob, not a derived constant: total translation lead = {@code tof + SHOT_LATENCY}.
 * Too small → shots under-lead (drift in the travel direction); too big → over-lead.
 *
 * <p>Let the robot have field-frame velocity {@code v} (Pedro's getVelocity is field-frame),
 * acceleration {@code a}, heading {@code h}, angular velocity {@code w}, and angular
 * acceleration {@code al}. Translation uses the lead window {@code T} ({@link #SHOT_LATENCY},
 * ~0.5–0.8 s); rotation uses the smaller turret-latency window {@code Tr}
 * ({@link #TURRET_LEAD_TIME}, ~0.15 s).
 * <ul>
 *   <li>Velocity at launch:        {@code v_L = v + a*T}</li>
 *   <li>Turret-relevant heading drift: {@code dH = w*Tr + 0.5*al*Tr^2}</li>
 * </ul>
 *
 * <h3>Translation (incl. acceleration) — folded into the goal aim point</h3>
 * The artifact, once airborne, carries {@code v_L}, so to land in the goal we aim a
 * static shot (solved from the predicted launch position) at the virtual target
 * {@code goal - v_L*tof}. Re-expressed as an offset on the goal (because the turret /
 * shooter still measure from the <em>current</em> pose), this collapses to:
 * <pre>   offset = -(v_L * tof) - (v*T + 0.5*a*T^2)</pre>
 * which feeds {@link Configuration#setAimPointOffset(double, double)} → both the turret
 * aim and the shooter distance/RPM. With {@code a = 0} this is exactly
 * {@code -v*(tof + T)}, i.e. the previous velocity-only behavior — so this is a strict
 * superset of what already worked.
 *
 * <h3>Rotation — turret angle feed-forward</h3>
 * The turret tracks the goal every loop, so it does NOT freeze for the whole feed time —
 * it only lags the target by its own actuator/control latency {@code Tr}
 * ({@link #TURRET_LEAD_TIME}, ~0.15 s), independent of the much larger feed time. A robot
 * spinning at {@code w} therefore leaves the turret behind by
 * {@code dH = w*Tr + 0.5*al*Tr^2}; we pre-rotate it by {@code -dH} (in
 * {@link Configuration#TURRET_PREDICTIVE_OFFSET}, which {@code Turret} adds on top of the
 * mechanical offset). Using the full feed time here would massively over-lead.
 *
 * <h2>Robustness</h2>
 * {@code w}, {@code al}, and {@code a} are derived by finite-differencing the pose /
 * velocity (no dependency on a specific localizer API) with low-pass filtering,
 * deadbands, and clamps. The whole predictive layer is gated by {@link #ROTATION_GAIN}
 * and {@link #ACCEL_GAIN}; set both to 0 for plain velocity-only SOTM, or
 * {@link #ENABLED} = false to disable prediction entirely.
 */
public final class AimController {
    private AimController() {}

    // ---- Master gates ------------------------------------------------------
    /** Master switch for the predictive (accel + rotation) layer. False = velocity-only SOTM. */
    public static boolean ENABLED = true;
    /** Scales the linear-acceleration contribution (predictive). On by default. */
    public static double ACCEL_GAIN = 1.0;
    /** Scales the rotational lead. Sign verified POSITIVE on-robot (turret leads the correct
     *  way at slow rotation). NOTE: at *fast* rotation the turret can hit its servo slew-rate
     *  limit and lag no matter how big this is — that's mechanical, not a gain. */
    public static double ROTATION_GAIN = 1.0;

    // ---- Timing ------------------------------------------------------------
    /** Lead time (s) used before the kinematics have produced a valid time-of-flight. */
    public static double FALLBACK_TOF = 0.3;
    /** Clamp on time-of-flight (s) so a bad solve can't fling the aim. */
    public static double MAX_TOF = 1.5;
    /** Turret actuator/control latency (s) — the window for the rotational lead. Much
     *  smaller than the feed time because the turret re-aims every loop. */
    public static double TURRET_LEAD_TIME = 0.15;
    /** Translation lead time, ADDED to flight time: total lead = (tof + SHOT_LATENCY). The
     *  master "how far ahead do I aim while moving" knob. EMPIRICALLY ~0.5–0.8 s on this bot
     *  (the effective system lag — servo speed, feed, processing — is much larger than a
     *  clean "actuator latency" model predicts). Tune with the STRAFE TEST: if the ball lands
     *  in the SAME direction you strafe → aim is UNDER-leading → RAISE this; if it lands
     *  OPPOSITE the strafe → over-leading → lower it. */
    public static double SHOT_LATENCY = 0.70;

    // ---- Noise handling ----------------------------------------------------
    /** Ignore translational velocity below this (in/s) so odometry noise — and the slow
     *  velocity decay right after the robot stops — doesn't keep moving the aim. Also gates the
     *  accel terms. Raise it if the turret still wanders/oscillates for a beat after stopping;
     *  lower it if slow driving shots stop getting any lead. */
    public static double VELOCITY_DEADBAND = 4.0;
    /** Ignore linear acceleration below this (in/s^2). */
    public static double ACCEL_DEADBAND = 5.0;
    /** Ignore angular velocity below this (rad/s). */
    public static double ANGULAR_VELOCITY_DEADBAND = 0.10;
    /** Ignore angular acceleration below this (rad/s^2). */
    public static double ANGULAR_ACCEL_DEADBAND = 0.50;

    /** Low-pass gains (0..1, higher = more responsive / noisier) for the differentiated signals. */
    public static double ACCEL_LOWPASS = 0.15;
    public static double OMEGA_LOWPASS = 0.45;   // fairly responsive so the lead engages quickly when a spin starts
    public static double ALPHA_LOWPASS = 0.15;

    // ---- Safety clamps -----------------------------------------------------
    /** Clamp on the magnitude of the goal aim-point offset (inches). */
    public static double MAX_AIM_OFFSET = 60.0;
    /** Clamp on filtered linear acceleration magnitude per axis (in/s^2). */
    public static double MAX_ACCEL = 200.0;
    /** Clamp on the predictive turret lead (degrees). Raised to 45 so the lead can keep
     *  growing during fast spins instead of saturating at 20 (it was clamping mid-spin). */
    public static double MAX_TURRET_LEAD_DEG = 45.0;
    /** Loop gaps longer than this (s) re-initialize the differentiators (no spurious spikes). */
    public static double MAX_DT = 0.10;

    // ---- Internal differentiator state ------------------------------------
    private static long lastTimeNs = 0;
    private static double lastVx = 0, lastVy = 0, lastHeading = 0;
    private static double ax = 0, ay = 0, omega = 0, alpha = 0;

    // ---- Exposed for telemetry / tuning -----------------------------------
    public static double lastOffsetX = 0, lastOffsetY = 0, lastTof = 0;
    public static double lastOmega = 0, lastAlpha = 0, lastAccelX = 0, lastAccelY = 0;
    public static double lastTurretLeadDeg = 0;

    /**
     * Clears the predictive turret lead and re-arms the differentiators. Call this on the
     * static (non-SOTM) code paths so a stale lead from a previous moving shot can't
     * corrupt a stationary shot.
     */
    public static void clearPrediction() {
        Configuration.TURRET_PREDICTIVE_OFFSET = 0;
        Configuration.TURRET_ZONE_OFFSET = 0;
        lastTurretLeadDeg = 0;
        lastTimeNs = 0; // forces a differentiator re-init on the next call
    }

    /**
     * Static (non-SOTM) aim update. Applies the SAME zone logic as
     * {@link #updateShootOnMove} — the far-shoot-zone fixed goal pose plus its turret offset
     * and RPM bonus, and the offset-zone turret offset — but with NO predictive velocity /
     * acceleration / rotation lead. Use on autos that shoot while stopped so they still hit
     * the tuned zones and goal pose without leading a moving shot. Call once per loop in
     * odometry mode; apply any per-opmode mechanical {@code TURRET_OFFSET} / RPM tweak after.
     */
    public static void updateStaticAim() {
        Shooter s = Shooter.INSTANCE;
        Pose pose = Configuration.CURRENT_POSE;
        boolean red = Configuration.ALLIANCE == Configuration.Alliance.RED;

        // Far-shoot zone: aim at the fixed far-shoot goal pose with the far turret offset and
        // a hotter RPM (identical to the SOTM path's far-shoot branch, just no lead).
        if (Configuration.inFarShootZone(pose.getX(), pose.getY())) {
            Pose g = Configuration.farShootGoal();
            Pose base = Configuration.allianceGoal();
            Configuration.setAimPointOffset(g.getX() - base.getX(), g.getY() - base.getY());
            Configuration.TURRET_ZONE_OFFSET = red
                    ? Configuration.FAR_SHOOT_TURRET_OFFSET        // RED
                    : -Configuration.FAR_SHOOT_TURRET_OFFSET;      // BLUE
            Configuration.TURRET_PREDICTIVE_OFFSET = 0;
            lastOffsetX = Configuration.X_GOAL_OFFSET;
            lastOffsetY = Configuration.Y_GOAL_OFFSET;
            lastTurretLeadDeg = 0;
            lastTimeNs = 0;   // re-arm the differentiators (no stale lead if SOTM resumes)
            s.updateKinematics(s.GOAL_DISTANCE, Math.toRadians(s.HOOD_ANGLE));
            s.TARGET_RPM = s.getKinematicRPMGoal()
                    * (Configuration.RPM_MULTIPLER + Configuration.FAR_SHOOT_RPM_BONUS);
            return;
        }

        // Otherwise: aim straight at the alliance goal (no velocity lead), but still honor the
        // offset-zone turret nudge if we're inside it.
        Configuration.setAimPointOffset(0, 0);
        Configuration.TURRET_ZONE_OFFSET = Configuration.inOffsetZone(pose.getX(), pose.getY())
                ? (red ? -Configuration.OFFSET_ZONE_TURRET_OFFSET : Configuration.OFFSET_ZONE_TURRET_OFFSET)
                : 0.0;
        Configuration.TURRET_PREDICTIVE_OFFSET = 0;
        lastOffsetX = 0;
        lastOffsetY = 0;
        lastTurretLeadDeg = 0;
        lastTimeNs = 0;
        s.updateKinematics(s.GOAL_DISTANCE, Math.toRadians(s.HOOD_ANGLE));
        s.TARGET_RPM = s.getKinematicRPMGoal() * Configuration.RPM_MULTIPLER;
    }

    /**
     * Updates the virtual-target aim offset, the predictive turret lead, and the shooter
     * target RPM for this loop. Call once per loop while shooting in odometry mode; apply
     * any per-opmode mechanical turret offset / close-range RPM tweak afterwards (the
     * mechanical offset and this predictive lead are separate fields and compose).
     *
     * @param vxInPerSec robot field-frame X velocity, inches/sec (follower.getVelocity().getXComponent())
     * @param vyInPerSec robot field-frame Y velocity, inches/sec (follower.getVelocity().getYComponent())
     */
    public static void updateShootOnMove(double vxInPerSec, double vyInPerSec) {
        Shooter s = Shooter.INSTANCE;
        Pose pose = Configuration.CURRENT_POSE;
        boolean red = Configuration.ALLIANCE == Configuration.Alliance.RED;

        // --- SOTM zones (zone shapes/goal live in Configuration; the decision is made here) ---
        // Far-shoot zone (tiny triangle): aim at the fixed far-shoot point with NO lead and NO
        // zone offset. Implemented as a fixed goal-aim-point offset (= farGoal - allianceGoal),
        // so the turret/shooter aim straight at it. Returns early — skips the moving-shot math.
        if (Configuration.inFarShootZone(pose.getX(), pose.getY())) {
            Pose g = Configuration.farShootGoal();
            Pose base = Configuration.allianceGoal();
            Configuration.setAimPointOffset(g.getX() - base.getX(), g.getY() - base.getY());
            Configuration.TURRET_ZONE_OFFSET = red
                    ? Configuration.FAR_SHOOT_TURRET_OFFSET        // RED: 2deg right
                    : -Configuration.FAR_SHOOT_TURRET_OFFSET;      // BLUE: 2deg left
            Configuration.TURRET_PREDICTIVE_OFFSET = 0;
            lastOffsetX = Configuration.X_GOAL_OFFSET;
            lastOffsetY = Configuration.Y_GOAL_OFFSET;
            lastTurretLeadDeg = 0;
            lastTimeNs = 0;   // re-arm the differentiators for when we leave the zone
            s.updateKinematics(s.GOAL_DISTANCE, Math.toRadians(s.HOOD_ANGLE));
            s.TARGET_RPM = s.getKinematicRPMGoal()
                    * (Configuration.RPM_MULTIPLER + Configuration.FAR_SHOOT_RPM_BONUS);  // hotter for the long shot
            return;
        }
        // Offset zone: -3 (RED) / +3 (BLUE) added to the turret on top of the normal
        // moving-shot aim below. RED was over-aiming LEFT, so it gets the negative
        // (right) and BLUE the positive (left).
        Configuration.TURRET_ZONE_OFFSET = Configuration.inOffsetZone(pose.getX(), pose.getY())
                ? (red ? -Configuration.OFFSET_ZONE_TURRET_OFFSET : Configuration.OFFSET_ZONE_TURRET_OFFSET)
                : 0.0;

        // --- Differentiate pose/velocity into accel (a) and angular vel/accel (w, al) ---
        long now = System.nanoTime();
        double heading = Configuration.CURRENT_POSE.getHeading();
        double dt = (now - lastTimeNs) / 1e9;
        boolean haveDeriv = ENABLED && lastTimeNs != 0 && dt > 1e-4 && dt < MAX_DT;

        if (haveDeriv) {
            ax = lowpass((vxInPerSec - lastVx) / dt, ax, ACCEL_LOWPASS);
            ay = lowpass((vyInPerSec - lastVy) / dt, ay, ACCEL_LOWPASS);

            double dHeading = AngleUnit.normalizeRadians(heading - lastHeading); // shortest signed delta
            double newOmega = lowpass(dHeading / dt, omega, OMEGA_LOWPASS);
            alpha = lowpass((newOmega - omega) / dt, alpha, ALPHA_LOWPASS);
            omega = newOmega;
        } else {
            ax = ay = omega = alpha = 0; // re-init: no prediction this loop
        }
        lastVx = vxInPerSec;
        lastVy = vyInPerSec;
        lastHeading = heading;
        lastTimeNs = now;

        // --- Condition the predictive signals (deadband, clamp, gain) ---
        double vx = Math.abs(vxInPerSec) < VELOCITY_DEADBAND ? 0 : vxInPerSec;
        double vy = Math.abs(vyInPerSec) < VELOCITY_DEADBAND ? 0 : vyInPerSec;

        // Gate the acceleration terms on actually moving / rotating. When the robot stops, the
        // odometry velocity decays noisily for ~1-2s; differentiating that produces a ringing
        // acceleration that keeps shoving the aim point around, so the turret oscillates while
        // settling. Zeroing accel once (essentially) stopped lets the aim settle immediately.
        boolean translating = Math.hypot(vxInPerSec, vyInPerSec) >= VELOCITY_DEADBAND;
        double omegaE = deadband(omega, ANGULAR_VELOCITY_DEADBAND);
        double axE = translating ? clamp(deadband(ax, ACCEL_DEADBAND), MAX_ACCEL) * ACCEL_GAIN : 0.0;
        double ayE = translating ? clamp(deadband(ay, ACCEL_DEADBAND), MAX_ACCEL) * ACCEL_GAIN : 0.0;
        double alphaE = (omegaE != 0.0) ? deadband(alpha, ANGULAR_ACCEL_DEADBAND) * ACCEL_GAIN : 0.0;

        double T = SHOT_LATENCY;   // actuator latency, NOT the feed time (turret re-aims every loop)
        double tof = s.getTof();
        tof = (Double.isNaN(tof) || tof <= 0) ? FALLBACK_TOF : Math.min(tof, MAX_TOF);
        lastTof = tof;

        // --- Translation: virtual target folded into the goal offset ---
        double vLx = vx + axE * T;                         // velocity at launch
        double vLy = vy + ayE * T;
        double offX = -(vLx * tof) - (vx * T + 0.5 * axE * T * T);
        double offY = -(vLy * tof) - (vy * T + 0.5 * ayE * T * T);

        double mag = Math.hypot(offX, offY);
        if (mag > MAX_AIM_OFFSET) {
            double k = MAX_AIM_OFFSET / mag;
            offX *= k;
            offY *= k;
        }
        lastOffsetX = offX;
        lastOffsetY = offY;
        lastAccelX = axE;
        lastAccelY = ayE;
        Configuration.setAimPointOffset(offX, offY);

        // --- Rotation: pre-rotate the turret by the heading change over its latency window ---
        // Uses TURRET_LEAD_TIME (actuator latency), NOT T, because the turret re-aims every loop.
        double Tr = TURRET_LEAD_TIME;
        double dH = omegaE * Tr + 0.5 * alphaE * Tr * Tr;  // predicted heading change (rad)
        double turretLead = clamp(-Math.toDegrees(dH) * ROTATION_GAIN, MAX_TURRET_LEAD_DEG);
        lastOmega = omegaE;
        lastAlpha = alphaE;
        lastTurretLeadDeg = turretLead;
        Configuration.TURRET_PREDICTIVE_OFFSET = turretLead;

        // --- Solve trajectory to the (virtual-target) distance & set RPM ---
        s.updateKinematics(s.GOAL_DISTANCE, Math.toRadians(s.HOOD_ANGLE));
        s.TARGET_RPM = s.getKinematicRPMGoal() * Configuration.RPM_MULTIPLER;
    }

    private static double lowpass(double raw, double prev, double gain) {
        return gain * raw + (1 - gain) * prev;
    }

    private static double deadband(double v, double db) {
        return Math.abs(v) < db ? 0 : v;
    }

    private static double clamp(double v, double limit) {
        return Math.max(-limit, Math.min(limit, v));
    }
}
