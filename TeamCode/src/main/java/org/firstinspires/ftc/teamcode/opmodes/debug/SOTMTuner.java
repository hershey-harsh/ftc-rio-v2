package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleConsumer;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.impl.MotorEx;

import static dev.nextftc.bindings.Bindings.button;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AimController;
import org.firstinspires.ftc.teamcode.subsystems.Light;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * One-stop shoot-on-the-move tuner. Drive (gamepad 1) and shoot while a tuner (gamepad 2)
 * scrolls a cursor through every SOTM/droop/stall parameter and nudges it live — no
 * re-deploys between adjustments.
 *
 * See doc/SHOOT_ON_THE_MOVE_TUNING.md for the full procedure. Controls are also printed to
 * telemetry at the bottom.
 *
 * <pre>
 * GAMEPAD 1 — DRIVER (move + fire)
 *   Left stick      drive / strafe
 *   Right stick X   rotate
 *   Right bumper    FIRE (hold = open gate, release = close)
 *   A / B           intake on / off
 *   D-pad Left      relocalize to the test pose (repeatable start spot)
 *   Back            shooter on / off (toggle)
 *
 * GAMEPAD 2 — TUNER (adjust, no re-deploy)
 *   D-pad Up/Down   select previous / next parameter
 *   D-pad Right/Left  increase / decrease selected by 1 step
 *   Right/Left bmpr   increase / decrease by 10 steps (coarse)
 *   Y               ROTATION layer on/off          (ROTATION_GAIN 0 <-> +/-1)
 *   B               FLIP rotation sign             (do this if the turret leads the WRONG way)
 *   X               ACCEL layer on/off             (ACCEL_GAIN 0 <-> 1)
 *   A               master predictive ENABLED on/off
 *   Start           toggle alliance RED / BLUE
 * </pre>
 */
@Disabled
@TeleOp(name = "SOTM Tuner", group = "Debug")
public class SOTMTuner extends NextFTCOpMode {

    private final MotorEx frontLeft  = new MotorEx(Configuration.LEFT_FRONT_MOTOR).reversed();
    private final MotorEx frontRight = new MotorEx(Configuration.RIGHT_FRONT_MOTOR);
    private final MotorEx backLeft   = new MotorEx(Configuration.LEFT_REAR_MOTOR).reversed();
    private final MotorEx backRight  = new MotorEx(Configuration.RIGHT_REAR_MOTOR);
    private DriverControlledCommand driverControlled;

    // MUST match how the robot is physically placed, or odometry (and the turret) will be
    // off by that error. We start at field center facing heading 270° (down the -Y axis) —
    // that's the physical start spot. Both alliance goals sit at ~135° relative from here,
    // which is inside the widened ±144° turret range. Place the robot here and press D-pad
    // Left to re-sync odometry to this spot.
    private static final Pose TEST_POSE = new Pose(72, 72, Math.toRadians(270));

    private double X_VELOCITY = 0, Y_VELOCITY = 0;
    private double mechTurretOffset = 0;          // the mechanical turret offset, tunable here
    private double rotationGainMag = 1.0;         // remembered magnitude for the Y on/off toggle

    private double LOOP_TIME = 0;
    private final ElapsedTime LOOP_TIMER = new ElapsedTime();

    // --- Tunable parameter table (ordered to match the guide's bring-up sequence) ---
    private static class Param {
        final String name; final DoubleSupplier get; final DoubleConsumer set;
        final double step; final String fmt;
        Param(String name, DoubleSupplier get, DoubleConsumer set, double step, String fmt) {
            this.name = name; this.get = get; this.set = set; this.step = step; this.fmt = fmt;
        }
    }
    private final List<Param> params = new ArrayList<>();
    private int sel = 0;

    public SOTMTuner() {
        addComponents(
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Light.INSTANCE),
                new SubsystemComponent(Limelight.INSTANCE),
                new SubsystemComponent(Shooter.INSTANCE),
                new SubsystemComponent(Transfer.INSTANCE),
                new SubsystemComponent(Turret.INSTANCE)
        );
    }

    @Override
    public void onInit() {
        for (MotorEx m : new MotorEx[]{frontLeft, frontRight, backLeft, backRight}) {
            m.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        PedroComponent.follower().setStartingPose(TEST_POSE);
        Configuration.CURRENT_POSE = PedroComponent.follower().getPose();
        Configuration.SHOOTER_HEIGHT_TO_GOAL = 1.1;
        Configuration.ALLIANCE = Configuration.Alliance.RED;
        Limelight.INSTANCE.MODE = Limelight.Mode.LOCALIZATION;
        // The tuner prints its own LIVE block, so keep the subsystem debug dumps off to
        // avoid a wall of telemetry on the driver station (flip these on if you use Dashboard).
        Shooter.INSTANCE.DEBUG_TELEMETRY = false;

        // group 1: static foundation + timing
        params.add(new Param("RPM_MULTIPLER",            () -> Configuration.RPM_MULTIPLER,            v -> Configuration.RPM_MULTIPLER = v,            0.05, "%.3f"));
        params.add(new Param("HOOD_TRIM (deg, - = flat)",() -> Shooter.HOOD_TRIM,                      v -> Shooter.HOOD_TRIM = v,                     1.0,  "%.1f"));
        params.add(new Param("Mech TURRET_OFFSET (deg)", () -> mechTurretOffset,                       v -> mechTurretOffset = v,                      0.5,  "%.2f"));
        params.add(new Param("SHOT_LATENCY (lead, s)",   () -> AimController.SHOT_LATENCY,             v -> AimController.SHOT_LATENCY = v,            0.02, "%.3f"));
        // group 2: rotation
        params.add(new Param("TURRET_LEAD_TIME Tr",      () -> AimController.TURRET_LEAD_TIME,         v -> AimController.TURRET_LEAD_TIME = v,        0.01, "%.3f"));
        params.add(new Param("ROTATION_GAIN",            () -> AimController.ROTATION_GAIN,            v -> AimController.ROTATION_GAIN = v,           0.1,  "%.2f"));
        params.add(new Param("OMEGA_LOWPASS",            () -> AimController.OMEGA_LOWPASS,            v -> AimController.OMEGA_LOWPASS = v,           0.05, "%.2f"));
        params.add(new Param("ANG_VEL_DEADBAND (rad/s)", () -> AimController.ANGULAR_VELOCITY_DEADBAND,v -> AimController.ANGULAR_VELOCITY_DEADBAND=v, 0.02, "%.3f"));
        params.add(new Param("MAX_TURRET_LEAD_DEG",      () -> AimController.MAX_TURRET_LEAD_DEG,      v -> AimController.MAX_TURRET_LEAD_DEG = v,     1.0,  "%.1f"));
        // group 2b: turret travel limits — widen ONLY if the servo can safely reach further
        params.add(new Param("MIN_SERVO_POS (turret end)",() -> Turret.MIN_SERVO_POS,                 v -> Turret.MIN_SERVO_POS = v,                  0.02, "%.3f"));
        params.add(new Param("MAX_SERVO_POS (turret end)",() -> Turret.MAX_SERVO_POS,                 v -> Turret.MAX_SERVO_POS = v,                  0.02, "%.3f"));
        params.add(new Param("FAR_SHOOT_TURRET_OFFSET",  () -> Configuration.FAR_SHOOT_TURRET_OFFSET,  v -> Configuration.FAR_SHOOT_TURRET_OFFSET = v, 0.5,  "%.1f"));
        params.add(new Param("OFFSET_ZONE_TURRET_OFFSET",() -> Configuration.OFFSET_ZONE_TURRET_OFFSET,v -> Configuration.OFFSET_ZONE_TURRET_OFFSET = v,0.5, "%.1f"));
        params.add(new Param("FAR_SHOOT_RPM_BONUS",      () -> Configuration.FAR_SHOOT_RPM_BONUS,      v -> Configuration.FAR_SHOOT_RPM_BONUS = v,     0.05, "%.2f"));
        // group 3: acceleration
        params.add(new Param("ACCEL_GAIN",               () -> AimController.ACCEL_GAIN,               v -> AimController.ACCEL_GAIN = v,              0.1,  "%.2f"));
        params.add(new Param("ACCEL_LOWPASS",            () -> AimController.ACCEL_LOWPASS,            v -> AimController.ACCEL_LOWPASS = v,           0.05, "%.2f"));
        params.add(new Param("ACCEL_DEADBAND (in/s^2)",  () -> AimController.ACCEL_DEADBAND,           v -> AimController.ACCEL_DEADBAND = v,          1.0,  "%.1f"));
        params.add(new Param("VELOCITY_DEADBAND (in/s)", () -> AimController.VELOCITY_DEADBAND,        v -> AimController.VELOCITY_DEADBAND = v,       0.5,  "%.1f"));
        // group 4: rapid-fire droop
        params.add(new Param("HOOD_DROOP_GAIN",          () -> Shooter.HOOD_DROOP_GAIN,                v -> Shooter.HOOD_DROOP_GAIN = v,               0.002,"%.4f"));
        params.add(new Param("MAX_HOOD_DROOP_COMP (deg)",() -> Shooter.MAX_HOOD_DROOP_COMP,            v -> Shooter.MAX_HOOD_DROOP_COMP = v,           1.0,  "%.1f"));
        params.add(new Param("DROOP_RPM_DEADBAND",       () -> Shooter.DROOP_RPM_DEADBAND,             v -> Shooter.DROOP_RPM_DEADBAND = v,            25.0, "%.0f"));
        // group 5: gate-3 motor protection
        params.add(new Param("GATE3_STALL_CURRENT (A)",  () -> Transfer.GATE3_STALL_CURRENT,           v -> Transfer.GATE3_STALL_CURRENT = v,          0.5,  "%.2f"));
        params.add(new Param("GATE3_STALL_TIME (s)",     () -> Transfer.GATE3_STALL_TIME,              v -> Transfer.GATE3_STALL_TIME = v,             0.05, "%.2f"));
    }

    @Override
    public void onStartButtonPressed() {
        driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().negate(),
                !Configuration.FIELD_CENTRIC
        );
        driverControlled.schedule();

        Shooter.INSTANCE.on().schedule();      // spin up + MODE = ODOMETRY
        Transfer.INSTANCE.start().schedule();

        /// ---- GAMEPAD 1 — driver ----
        button(() -> gamepad1.right_bumper)
                .whenTrue(() -> Transfer.INSTANCE.openGate(1.0).schedule())
                .whenBecomesFalse(() -> Transfer.INSTANCE.closeGate().schedule());
        button(() -> gamepad1.a).whenBecomesTrue(() -> Transfer.INSTANCE.intake().schedule());
        button(() -> gamepad1.b).whenBecomesTrue(() -> Transfer.INSTANCE.stop().schedule());
        button(() -> gamepad1.dpad_left).whenBecomesTrue(() -> PedroComponent.follower().setPose(TEST_POSE));
        button(() -> gamepad1.back)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> Shooter.INSTANCE.off().schedule())
                .whenBecomesFalse(() -> Shooter.INSTANCE.on().schedule());

        /// ---- GAMEPAD 2 — tuner ----
        button(() -> gamepad2.dpad_up).whenBecomesTrue(() -> sel = (sel - 1 + params.size()) % params.size());
        button(() -> gamepad2.dpad_down).whenBecomesTrue(() -> sel = (sel + 1) % params.size());
        button(() -> gamepad2.dpad_right).whenBecomesTrue(() -> adjust(1));
        button(() -> gamepad2.dpad_left).whenBecomesTrue(() -> adjust(-1));
        button(() -> gamepad2.right_bumper).whenBecomesTrue(() -> adjust(10));
        button(() -> gamepad2.left_bumper).whenBecomesTrue(() -> adjust(-10));

        button(() -> gamepad2.y).whenBecomesTrue(() -> {
            if (AimController.ROTATION_GAIN == 0) AimController.ROTATION_GAIN = rotationGainMag;
            else { rotationGainMag = AimController.ROTATION_GAIN; AimController.ROTATION_GAIN = 0; }
        });
        button(() -> gamepad2.b).whenBecomesTrue(() -> {
            AimController.ROTATION_GAIN = -AimController.ROTATION_GAIN;
            if (AimController.ROTATION_GAIN != 0) rotationGainMag = AimController.ROTATION_GAIN;
        });
        button(() -> gamepad2.x).whenBecomesTrue(() ->
                AimController.ACCEL_GAIN = (AimController.ACCEL_GAIN == 0) ? 1.0 : 0.0);
        button(() -> gamepad2.a).whenBecomesTrue(() -> AimController.ENABLED = !AimController.ENABLED);
        button(() -> gamepad2.start).whenBecomesTrue(() -> Configuration.ALLIANCE =
                Configuration.ALLIANCE == Configuration.Alliance.RED
                        ? Configuration.Alliance.BLUE : Configuration.Alliance.RED);
    }

    private void adjust(double steps) {
        Param p = params.get(sel);
        p.set.accept(p.get.getAsDouble() + steps * p.step);
    }

    @Override
    public void onUpdate() {
        LOOP_TIMER.reset();
        Configuration.CURRENT_POSE = PedroComponent.follower().getPose();

        if (Shooter.INSTANCE.MODE == Shooter.Mode.ODOMETRY) {
            X_VELOCITY = PedroComponent.follower().getVelocity().getXComponent();
            Y_VELOCITY = PedroComponent.follower().getVelocity().getYComponent();
            AimController.updateShootOnMove(X_VELOCITY, Y_VELOCITY);
            Configuration.TURRET_OFFSET = mechTurretOffset;   // mechanical offset is tuned here
        }

        // ---- Selected parameter (big) ----
        Param p = params.get(sel);
        telemetry.addData(">> PARAM [" + (sel + 1) + "/" + params.size() + "]",
                p.name + " = " + String.format(Locale.US, p.fmt, p.get.getAsDouble())
                        + "   (step " + String.format(Locale.US, p.fmt, p.step) + ")");

        // ---- Layer states ----
        telemetry.addLine();
        telemetry.addData("Master ENABLED", AimController.ENABLED);
        telemetry.addData("ROTATION_GAIN", String.format(Locale.US, "%.2f%s",
                AimController.ROTATION_GAIN, AimController.ROTATION_GAIN < 0 ? "  (flipped)" : ""));
        telemetry.addData("ACCEL_GAIN", String.format(Locale.US, "%.2f", AimController.ACCEL_GAIN));
        telemetry.addData("Alliance", Configuration.ALLIANCE);

        // ---- Live SOTM outputs ----
        telemetry.addLine();
        telemetry.addData("----- LIVE -----", "");
        telemetry.addData("Goal dist (m)", String.format(Locale.US, "%.3f", Shooter.INSTANCE.GOAL_DISTANCE));
        telemetry.addData("Target / Cur RPM", String.format(Locale.US, "%.0f / %.0f",
                Shooter.INSTANCE.TARGET_RPM, Shooter.INSTANCE.CURRENT_RPM));
        telemetry.addData("RPM error", String.format(Locale.US, "%.0f",
                Shooter.INSTANCE.TARGET_RPM - Shooter.INSTANCE.CURRENT_RPM));
        telemetry.addData("Hood angle (deg)", String.format(Locale.US, "%.1f", Shooter.INSTANCE.HOOD_ANGLE));
        telemetry.addData("Turret target (deg)", String.format(Locale.US, "%.1f", Turret.INSTANCE.TARGET_DEGREE));
        telemetry.addData("Turret in range", Turret.INSTANCE.TURRET_IN_RANGE
                + (Turret.INSTANCE.TURRET_IN_RANGE ? "" : "  <-- goal past turret limit; rotate, or widen MIN/MAX_SERVO_POS"));
        telemetry.addData("Zone", Configuration.inFarShootZone(Configuration.CURRENT_POSE.getX(), Configuration.CURRENT_POSE.getY())
                ? "FAR-SHOOT (fixed goal + far turret offset)"
                : Configuration.inOffsetZone(Configuration.CURRENT_POSE.getX(), Configuration.CURRENT_POSE.getY())
                        ? "2-DEG OFFSET" : "none");
        telemetry.addData("Zone turret offset (deg)", String.format(Locale.US, "%.1f", Configuration.TURRET_ZONE_OFFSET));
        telemetry.addData("Aim offset (in)", String.format(Locale.US, "x %.2f  y %.2f",
                AimController.lastOffsetX, AimController.lastOffsetY));
        telemetry.addData("Turret lead (deg)", String.format(Locale.US, "%.2f", AimController.lastTurretLeadDeg));
        telemetry.addData("omega (rad/s)", String.format(Locale.US, "%.3f", AimController.lastOmega));
        telemetry.addData("accel (in/s^2)", String.format(Locale.US, "x %.1f  y %.1f",
                AimController.lastAccelX, AimController.lastAccelY));
        telemetry.addData("vel (in/s)", String.format(Locale.US, "x %.1f  y %.1f", X_VELOCITY, Y_VELOCITY));
        telemetry.addData("Gate3 motor (A)", String.format(Locale.US, "%.2f", Transfer.gate3MotorCurrent));
        telemetry.addData("Gate3 stalled", Transfer.INSTANCE.GATE3_MOTOR_STALLED);

        // ---- All params (compact) ----
        telemetry.addLine();
        telemetry.addData("----- ALL PARAMS -----", "");
        for (int i = 0; i < params.size(); i++) {
            Param q = params.get(i);
            telemetry.addLine((i == sel ? " > " : "   ") + q.name + " = "
                    + String.format(Locale.US, q.fmt, q.get.getAsDouble()));
        }

        telemetry.addLine();
        telemetry.addLine("G1: sticks drive | RB fire | A/B intake | Dpad-L relocalize | Back shooter");
        telemetry.addLine("G2: Dpad U/D select | Dpad L/R +-1 | bumpers +-10 | Y rot | B flip | X accel | A enable | Start alliance");
        telemetry.addData("Loop (hz)", LOOP_TIME > 0 ? String.format(Locale.US, "%.0f", 1000.0 / LOOP_TIME) : "---");

        BindingManager.update();
        driverControlled.setScalar(Configuration.CONTROL_SCALE);
        telemetry.update();
        LOOP_TIME = LOOP_TIMER.milliseconds();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
    }
}
