package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Range;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.impl.MotorEx;

import static dev.nextftc.bindings.Bindings.button;

import java.util.function.Supplier;

import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Light;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.AimController;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Main competition teleop (SOTM + Forza controls). This is the BLUE base; {@link CompetitionRed}
 * extends it and only flips the alliance (everything is alliance-aware, so the X/B buttons can
 * still re-override mid-match and all poses/zones re-mirror).
 *
 * <p>Beyond the Forza control set it adds two things requested for competition:
 * <ul>
 *   <li><b>Operator turret offset trim</b> (gamepad 2 D-pad L/R): a persistent ± trim folded
 *       into the mechanical turret offset every loop, so it actually sticks in odometry mode
 *       (unlike {@code Turret.increaseAngle}, which the per-loop aim overwrites). D-pad Up zeroes it.</li>
 *   <li><b>Auto gate run</b> (gamepad 1 D-pad Up): drive to the gate, dwell, then drive back into
 *       the close-shoot zone, auto-opening the transfer gate the instant the robot ENTERS the zone
 *       (rising edge — opens <i>while</i> entering, not after) and closing it {@code GATE_OPEN_SECONDS}
 *       later. D-pad Down breaks the path and disarms (an interrupted run never opens the gate).</li>
 * </ul>
 *
 * <pre>
 * GAMEPAD 1 (Driver)
 *   Right trigger - Left trigger   forward / back        Left stick X   strafe
 *   Right stick X                  rotate
 *   D-pad Up      auto gate run (to gate -> back, auto-opens on close-shoot-zone entry)
 *   D-pad Down    break / cancel automation (disarms the gate run)
 *   D-pad Right   auto-drive to park        D-pad Left   relocalize to alliance human-player pose
 *   X / B         force BLUE / RED alliance         Back   emergency stop (toggle)
 *
 * GAMEPAD 2 (Operator)
 *   D-pad Left / Right   turret offset trim -/+ (persistent)     D-pad Up   reset trim
 *   X / B   intake on / off     Right bumper   FIRE (hold = open gate)     Back   emergency stop
 * </pre>
 */
@TeleOp(name = "Competition Blue", group = "Competition")
public class CompetitionBlue extends NextFTCOpMode {
    private double X_VELOCITY = 0;
    private double Y_VELOCITY = 0;
    public double TRUE_TARGET_DEGREE = 0;

    double LOOP_TIME = 0;
    ElapsedTime LOOP_TIMER = new ElapsedTime();

    private final MotorEx frontLeftMotor = new MotorEx(Configuration.LEFT_FRONT_MOTOR).reversed();
    private final MotorEx frontRightMotor = new MotorEx(Configuration.RIGHT_FRONT_MOTOR);
    private final MotorEx backLeftMotor = new MotorEx(Configuration.LEFT_REAR_MOTOR).reversed();
    private final MotorEx backRightMotor = new MotorEx(Configuration.RIGHT_REAR_MOTOR);
    private DriverControlledCommand driverControlled;

    private boolean automatedDrive = false;     // park-path drive (resumes manual when done)
    private Supplier<PathChain> parkPath;

    // ---- Operator turret trim (gamepad 2) -------------------------------------------------
    public static double TURRET_TRIM_STEP = 0.5;     // degrees per press
    private double operatorTurretTrim = 0.0;

    // ---- Auto gate run + close-shoot-zone auto-open ---------------------------------------
    /** Return-to-shoot pose for the gate run (RED; BLUE = mirror), from RT2GA18's launch pose. */
    private static final Pose SHOOT_RED = new Pose(81.501, 76.116, Math.toRadians(-45));
    /** Seconds to dwell at the gate before driving back. */
    public static double GATE_DWELL = 0.7;
    /** How long the transfer gate stays auto-opened after entering the close-shoot zone (2–3 s). */
    public static double GATE_OPEN_SECONDS = 2.5;

    private enum GateRun { IDLE, TO_GATE, AT_GATE, TO_SHOOT }
    private GateRun gateRun = GateRun.IDLE;
    private final ElapsedTime gateTimer = new ElapsedTime();
    private boolean gateArmed = false;     // D-pad Up pressed and the run hasn't been broken
    private boolean gateOpened = false;    // transfer gate currently auto-opened this run
    private boolean wasInZone = false;     // previous-loop zone membership (for the entry edge)
    private final ElapsedTime gateOpenTimer = new ElapsedTime();

    /** Overridden by {@link CompetitionRed}; sets the alliance in onInit. */
    public void setupOpMode() {
        Configuration.ALLIANCE = Configuration.Alliance.BLUE;
    }

    public CompetitionBlue() {
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

    private boolean red() { return Configuration.ALLIANCE == Configuration.Alliance.RED; }
    private Pose gateTarget() { return red() ? Configuration.GATE_OPEN_RED : Configuration.GATE_OPEN_BLUE; }
    private Pose gateCtrl()   { return red() ? Configuration.GATE_OPEN_RED_2 : Configuration.GATE_OPEN_BLUE_2; }
    private Pose shootPose()  { return red() ? SHOOT_RED : SHOOT_RED.mirror(); }
    private boolean inCloseShootZone() {
        Pose p = Configuration.CURRENT_POSE;
        return Configuration.inCloseShootZone(p.getX(), p.getY());
    }

    private PathChain buildToGate() {
        Pose cur = PedroComponent.follower().getPose();
        return PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(cur, gateCtrl(), gateTarget()))
                .setLinearHeadingInterpolation(cur.getHeading(), gateTarget().getHeading())
                .build();
    }

    private PathChain buildGateToShoot() {
        Pose cur = PedroComponent.follower().getPose();
        return PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(cur, gateCtrl(), shootPose()))
                .setLinearHeadingInterpolation(cur.getHeading(), shootPose().getHeading())
                .build();
    }

    /** Break the follower, hand back manual control, and disarm the gate run. */
    private void cancelAutomation() {
        PedroComponent.follower().breakFollowing();
        PedroComponent.follower().startTeleopDrive();
        if (gateOpened) Transfer.INSTANCE.closeGate().schedule();
        gateArmed = false;
        gateOpened = false;
        gateRun = GateRun.IDLE;
        automatedDrive = false;
    }

    @Override
    public void onInit() {
        frontLeftMotor.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightMotor.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftMotor.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightMotor.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        PedroComponent.follower().setStartingPose(Configuration.CURRENT_POSE);
        Configuration.CURRENT_POSE = PedroComponent.follower().getPose();

        Configuration.SHOOTER_HEIGHT_TO_GOAL = 1.1;
        setupOpMode();   // alliance (BLUE here; RED in the subclass)

        Limelight.INSTANCE.MODE = Limelight.Mode.LOCALIZATION;
        Transfer.INSTANCE.GATE12_DEBOUNCE_THRESHOLD = 3;
    }

    @Override
    public void onStartButtonPressed() {
        Range leftTrigger = Gamepads.gamepad1().leftTrigger();
        driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().rightTrigger().map(rt -> rt - leftTrigger.get()),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().negate(),
                !Configuration.FIELD_CENTRIC
        );

        driverControlled.schedule();
        Transfer.INSTANCE.intake().schedule();
        Shooter.INSTANCE.on().schedule();

        // Lazy park path — built from the robot's current pose when the button is pressed.
        parkPath = () -> {
            Pose target = red() ? Configuration.RED_PARK_BR : Configuration.BLUE_PARK_BR;
            return PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(PedroComponent.follower().getPose(), target))
                    .setLinearHeadingInterpolation(PedroComponent.follower().getPose().getHeading(), target.getHeading())
                    .build();
        };

        /// ---- Gamepad 1 (Driver) ---- ///

        // Back → Emergency Stop (toggle)
        button(() -> gamepad1.back)
                .toggleOnBecomesTrue()
                .whenTrue(() -> {
                    Shooter.INSTANCE.emergencyStop().schedule();
                    Transfer.INSTANCE.emergencyStopAll().schedule();
                    Turret.INSTANCE.emergencyStop().schedule();
                })
                .whenBecomesFalse(() -> {
                    Shooter.INSTANCE.start().schedule();
                    Transfer.INSTANCE.start().schedule();
                    Turret.INSTANCE.start().schedule();
                });

        // X → force BLUE alliance, B → force RED alliance
        button(() -> gamepad1.x).whenTrue(() -> Configuration.ALLIANCE = Configuration.Alliance.BLUE);
        button(() -> gamepad1.b).whenTrue(() -> Configuration.ALLIANCE = Configuration.Alliance.RED);

        // D-Pad Up → ARM + start the auto gate run (only from idle). Arming enables the
        //            close-shoot-zone auto-open on the way back.
        button(() -> gamepad1.dpad_up).whenBecomesTrue(() -> {
            if (gateRun == GateRun.IDLE) {
                PedroComponent.follower().followPath(buildToGate());
                gateRun = GateRun.TO_GATE;
                gateArmed = true;
                gateOpened = false;
            }
        });

        // D-Pad Down → break the path + disarm (gate will NOT auto-open after this)
        button(() -> gamepad1.dpad_down).whenBecomesTrue(this::cancelAutomation);

        // D-Pad Right → auto-drive to park
        button(() -> gamepad1.dpad_right).whenBecomesTrue(() -> {
            PedroComponent.follower().followPath(parkPath.get());
            automatedDrive = true;
        });

        // D-Pad Left → relocalize to alliance human-player pose
        button(() -> gamepad1.dpad_left).whenTrue(() -> PedroComponent.follower().setPose(
                red() ? Configuration.RED_LOCALIZATION_POSE : Configuration.BLUE_LOCALIZATION_POSE));

        /// ---- Gamepad 2 (Operator) ---- ///

        // D-Pad Left/Right → persistent turret offset trim; D-Pad Up → reset it
        button(() -> gamepad2.dpad_right).whenBecomesTrue(() -> operatorTurretTrim += TURRET_TRIM_STEP);
        button(() -> gamepad2.dpad_left).whenBecomesTrue(() -> operatorTurretTrim -= TURRET_TRIM_STEP);
        button(() -> gamepad2.dpad_up).whenBecomesTrue(() -> operatorTurretTrim = 0.0);

        // Back → Emergency Stop (toggle)
        button(() -> gamepad2.back)
                .toggleOnBecomesTrue()
                .whenTrue(() -> {
                    Shooter.INSTANCE.emergencyStop().schedule();
                    Transfer.INSTANCE.emergencyStopAll().schedule();
                    Turret.INSTANCE.emergencyStop().schedule();
                })
                .whenBecomesFalse(() -> {
                    Shooter.INSTANCE.start().schedule();
                    Transfer.INSTANCE.start().schedule();
                    Turret.INSTANCE.start().schedule();
                });

        // B → Intake Off, X → Intake On
        button(() -> gamepad2.b).whenTrue(() -> Transfer.INSTANCE.stop().schedule());
        button(() -> gamepad2.x).whenTrue(() -> Transfer.INSTANCE.intake().schedule());

        // Right Bumper → manual gate open while held, close when released
        button(() -> gamepad2.right_bumper)
                .whenTrue(() -> Transfer.INSTANCE.openGate(1.0).schedule())
                .whenBecomesFalse(() -> Transfer.INSTANCE.closeGate().schedule());
    }

    @Override
    public void onUpdate() {
        LOOP_TIMER.reset();

        Configuration.CURRENT_POSE = PedroComponent.follower().getPose();

        // --- Auto gate-run DRIVE state machine (driving only; the transfer gate is owned by
        //     the armed-zone auto-open logic below). ---
        switch (gateRun) {
            case TO_GATE:
                if (!PedroComponent.follower().isBusy()) {
                    gateTimer.reset();
                    gateRun = GateRun.AT_GATE;
                }
                break;
            case AT_GATE:
                if (gateTimer.seconds() > GATE_DWELL) {
                    PedroComponent.follower().followPath(buildGateToShoot());
                    gateRun = GateRun.TO_SHOOT;
                }
                break;
            case TO_SHOOT:
                if (!PedroComponent.follower().isBusy()) {
                    PedroComponent.follower().startTeleopDrive();
                    gateRun = GateRun.IDLE;
                    if (!gateOpened) gateArmed = false;   // ended without opening -> don't open later
                }
                break;
            default:
                break;
        }

        // --- Armed zone auto-open: open the gate the instant the robot ENTERS the close-shoot
        //     zone (rising edge), only while armed. Closes after GATE_OPEN_SECONDS. ---
        boolean inZone = inCloseShootZone();
        if (gateArmed) {
            if (!gateOpened && inZone && !wasInZone) {
                Transfer.INSTANCE.openGate().schedule();
                gateOpened = true;
                gateOpenTimer.reset();
            } else if (gateOpened && gateOpenTimer.seconds() > GATE_OPEN_SECONDS) {
                Transfer.INSTANCE.closeGate().schedule();
                gateOpened = false;
                gateArmed = false;
            }
        }
        wasInZone = inZone;

        // Robot light priority: YELLOW (following path) > AZURE (3 balls) > GREEN (1 ball) > alliance color
        if (PedroComponent.follower().isBusy()) {
            Light.INSTANCE.setColor(Light.YELLOW, Light.Target.ROBOT).schedule();
        } else if (Transfer.INSTANCE.ALL_STOPPED) {
            Light.INSTANCE.setColor(Light.AZURE, Light.Target.ROBOT).schedule();
        } else if (Transfer.INSTANCE.GATE12_BALL_PRESENT) {
            Light.INSTANCE.setColor(Light.GREEN, Light.Target.ROBOT).schedule();
        } else {
            Light.INSTANCE.setColor(red() ? Light.RED : Light.BLUE, Light.Target.ROBOT).schedule();
        }

        // Park path: resume manual when it finishes (gate run manages its own resume above).
        if (automatedDrive && gateRun == GateRun.IDLE && !PedroComponent.follower().isBusy()) {
            automatedDrive = false;
            PedroComponent.follower().startTeleopDrive();
        }

        BindingManager.update();
        driverControlled.setScalar(Configuration.CONTROL_SCALE);

        if (Shooter.INSTANCE.MODE == Shooter.Mode.ODOMETRY) {
            X_VELOCITY = PedroComponent.follower().getVelocity().getXComponent();
            Y_VELOCITY = PedroComponent.follower().getVelocity().getYComponent();

            // Shoot-on-the-move: virtual-target lead (direction + speed in one step; zero at rest).
            AimController.updateShootOnMove(X_VELOCITY, Y_VELOCITY);

            double base;
            if (Configuration.CURRENT_POSE.getY() < 36) {
                // Close range: mechanical aim offset (mirrored by alliance) + slightly hotter wheel.
                base = red() ? -2.0 : 2.0;
                Shooter.INSTANCE.TARGET_RPM = Shooter.INSTANCE.getKinematicRPMGoal() * (Configuration.RPM_MULTIPLER + 0.15);
            } else {
                // Far: small mechanical aim offset only — the lead is handled by AimController.
                base = red() ? -0.5 : 0.5;
            }
            // Operator trim composes on top and persists across loops.
            Configuration.TURRET_OFFSET = base + operatorTurretTrim;
        }

        boolean RPM_READY = Shooter.INSTANCE.TARGET_RPM > 0
                && Math.abs(Shooter.INSTANCE.TARGET_RPM - Shooter.INSTANCE.CURRENT_RPM) <= Shooter.RPM_TOLERANCE;
        boolean VELOCITY_LOW = Math.abs(X_VELOCITY) < 5 && Math.abs(Y_VELOCITY) < 5;
        if (Turret.INSTANCE.TURRET_IN_RANGE && RPM_READY && VELOCITY_LOW) {
            Light.INSTANCE.setColor(red() ? Light.RED : Light.BLUE, Light.Target.TURRET).schedule();
        } else {
            Light.INSTANCE.setColor(Light.YELLOW, Light.Target.TURRET).schedule();
        }

        telemetry.addData("Alliance", Configuration.ALLIANCE);
        telemetry.addData("Gate run", gateRun);
        telemetry.addData("Gate armed / open", gateArmed + " / " + gateOpened);
        telemetry.addData("In close-shoot zone", inZone);
        telemetry.addData("Turret trim (deg)", String.format(java.util.Locale.US, "%.1f", operatorTurretTrim));
        telemetry.addData("X", Configuration.CURRENT_POSE.getX());
        telemetry.addData("Y", Configuration.CURRENT_POSE.getY());
        telemetry.addData("Z", Math.toDegrees(Configuration.CURRENT_POSE.getHeading()));
        telemetry.addData("Target / Cur RPM", String.format(java.util.Locale.US, "%.0f / %.0f",
                Shooter.INSTANCE.TARGET_RPM, Shooter.INSTANCE.CURRENT_RPM));
        telemetry.addData("Loop (hz)", LOOP_TIME > 0 ? (1000 / LOOP_TIME) : 0);
        telemetry.update();

        LOOP_TIME = LOOP_TIMER.milliseconds();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
    }
}
