package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AimController;
import org.firstinspires.ftc.teamcode.subsystems.Light;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Teleop with a one-button "gate run": from wherever the robot is, drive to the gate, open
 * it, then drive back into the close-shoot zone — all while you keep driving/shooting the
 * rest of the time. Poses are taken from RT2GA18 (RED) and mirrored for BLUE.
 *
 * <pre>
 *   Left stick / Right stick X   drive / strafe / rotate
 *   Right bumper                 FIRE (hold = open gate, release = close)
 *   A / B                        intake on / off
 *   Y                            ARM + START gate run (drive to gate; auto-opens the gate on
 *                                zone entry, closes after GATE_OPEN_SECONDS, drives back)
 *   X                            BREAK follower (interrupt the path) + DISARM — after this the
 *                                gate will NOT auto-open even though Y was pressed
 *   D-pad Left                   relocalize to the alliance launch pose
 *   D-pad Up                     toggle alliance (poses re-mirror)
 * </pre>
 *
 * The automated drive uses raw {@code follower.followPath(...)} (not a command) so it
 * coexists with the scheduled driver-control command, exactly like the competition opmodes.
 */
@Disabled
@TeleOp(name = "Gate Run SOTM", group = "Red")
public class GateRunSOTM extends NextFTCOpMode {

    private final MotorEx frontLeft  = new MotorEx(Configuration.LEFT_FRONT_MOTOR).reversed();
    private final MotorEx frontRight = new MotorEx(Configuration.RIGHT_FRONT_MOTOR);
    private final MotorEx backLeft   = new MotorEx(Configuration.LEFT_REAR_MOTOR).reversed();
    private final MotorEx backRight  = new MotorEx(Configuration.RIGHT_REAR_MOTOR);
    private DriverControlledCommand driverControlled;

    private double X_VELOCITY = 0, Y_VELOCITY = 0;
    private double LOOP_TIME = 0;
    private final ElapsedTime LOOP_TIMER = new ElapsedTime();

    // ---- Gate-run poses (RED; BLUE = pose.mirror()), straight from RT2GA18 ----
    private static final Pose GATE_RED      = new Pose(131.000, 58.598, Math.toRadians(28.16));
    private static final Pose GATE_CTRL_RED = new Pose(105.784, 55.717);                  // curve control point
    private static final Pose SHOOT_RED     = new Pose(81.501, 76.116, Math.toRadians(-45));

    /** How long to dwell at the gate while it opens (seconds). */
    public static double GATE_DWELL = 0.7;

    /** How long the transfer gate stays auto-opened after entering the gate zone (2–3 s). */
    public static double GATE_OPEN_SECONDS = 2.5;

    // The "Close Shoot Zone" lives in Configuration (shared with the competition teleop):
    // Configuration.CLOSE_SHOOT_ZONE_RED / _BLUE + Configuration.inCloseShootZone(x, y).

    private enum GateRun { IDLE, TO_GATE, AT_GATE, TO_SHOOT }
    private GateRun gateRun = GateRun.IDLE;
    private final ElapsedTime gateTimer = new ElapsedTime();

    // Armed-zone auto-open state.
    private boolean gateArmed = false;     // Y pressed and the run hasn't been broken
    private boolean gateOpened = false;    // transfer gate currently auto-opened this run
    private boolean wasInZone = false;     // previous-loop zone membership (for the entry edge)
    private final ElapsedTime gateOpenTimer = new ElapsedTime();

    public GateRunSOTM() {
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
    private Pose gatePose()  { return red() ? GATE_RED : GATE_RED.mirror(); }
    private Pose gateCtrl()  { return red() ? GATE_CTRL_RED : GATE_CTRL_RED.mirror(); }
    private Pose shootPose() { return red() ? SHOOT_RED : SHOOT_RED.mirror(); }
    private boolean inCloseShootZone() {
        Pose p = Configuration.CURRENT_POSE;
        return Configuration.inCloseShootZone(p.getX(), p.getY());
    }

    @Override
    public void onInit() {
        for (MotorEx m : new MotorEx[]{frontLeft, frontRight, backLeft, backRight}) {
            m.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        PedroComponent.follower().setStartingPose(new Pose(72, 72, Math.toRadians(270)));
        Configuration.CURRENT_POSE = PedroComponent.follower().getPose();
        Configuration.SHOOTER_HEIGHT_TO_GOAL = 1.1;
        Configuration.ALLIANCE = Configuration.Alliance.RED;
        Limelight.INSTANCE.MODE = Limelight.Mode.LOCALIZATION;
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
        Shooter.INSTANCE.on().schedule();
        Transfer.INSTANCE.intake().schedule();

        // Manual fire + intake
        button(() -> gamepad1.right_bumper)
                .whenTrue(() -> Transfer.INSTANCE.openGate(1.0).schedule())
                .whenBecomesFalse(() -> Transfer.INSTANCE.closeGate().schedule());
        button(() -> gamepad1.a).whenBecomesTrue(() -> Transfer.INSTANCE.intake().schedule());
        button(() -> gamepad1.b).whenBecomesTrue(() -> Transfer.INSTANCE.stop().schedule());

        // Y -> ARM + start the gate run (only from idle). Arming enables the zone auto-open.
        button(() -> gamepad1.y).whenBecomesTrue(() -> {
            if (gateRun == GateRun.IDLE) {
                PedroComponent.follower().followPath(buildToGate());
                gateRun = GateRun.TO_GATE;
                gateArmed = true;
                gateOpened = false;
            }
        });
        // X -> BREAK the follower (interrupt the path) and DISARM, so the gate will NOT
        //      auto-open even though Y was pressed. Hands manual control back.
        button(() -> gamepad1.x).whenBecomesTrue(this::breakFollower);

        button(() -> gamepad1.dpad_left).whenBecomesTrue(() -> PedroComponent.follower().setPose(
                red() ? Configuration.RED_LOCALIZATION_POSE : Configuration.BLUE_LOCALIZATION_POSE));
        button(() -> gamepad1.dpad_up).whenBecomesTrue(() -> Configuration.ALLIANCE =
                red() ? Configuration.Alliance.BLUE : Configuration.Alliance.RED);
    }

    private PathChain buildToGate() {
        Pose cur = PedroComponent.follower().getPose();
        return PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(cur, gateCtrl(), gatePose()))
                .setLinearHeadingInterpolation(cur.getHeading(), gatePose().getHeading())
                .build();
    }

    private PathChain buildGateToShoot() {
        return PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(gatePose(), gateCtrl(), shootPose()))
                .setLinearHeadingInterpolation(gatePose().getHeading(), shootPose().getHeading())
                .build();
    }

    /** Interrupt the auto path and disarm the zone auto-open (the "pathing interrupted" path). */
    private void breakFollower() {
        PedroComponent.follower().breakFollowing();
        PedroComponent.follower().startTeleopDrive();
        if (gateOpened) Transfer.INSTANCE.closeGate().schedule();
        gateArmed = false;
        gateOpened = false;
        gateRun = GateRun.IDLE;
    }

    @Override
    public void onUpdate() {
        LOOP_TIMER.reset();
        Configuration.CURRENT_POSE = PedroComponent.follower().getPose();

        // --- Gate-run DRIVE state machine (driving only; the transfer gate is owned by the
        //     armed-zone auto-open logic below). ---
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
                    PedroComponent.follower().startTeleopDrive();   // resume manual driving
                    gateRun = GateRun.IDLE;
                    // Ended without ever opening -> don't auto-open later under manual control.
                    // If it DID open, the close-timer below finishes the open + disarms.
                    if (!gateOpened) gateArmed = false;
                }
                break;
            default:
                break;
        }

        // --- Armed zone auto-open: fire when the robot ENTERS the close-shoot zone (rising
        //     edge), only after Y (gateArmed). The robot usually starts inside the zone, so the
        //     edge means it opens on the RETURN entry, not at the Y press. Opens the transfer
        //     gate, then closes after GATE_OPEN_SECONDS. A follower break (X) clears gateArmed,
        //     so an interrupted run never opens. ---
        boolean inZone = inCloseShootZone();
        if (gateArmed) {
            if (!gateOpened && inZone && !wasInZone) {
                Transfer.INSTANCE.openGate().schedule();
                gateOpened = true;
                gateOpenTimer.reset();
            } else if (gateOpened && gateOpenTimer.seconds() > GATE_OPEN_SECONDS) {
                Transfer.INSTANCE.closeGate().schedule();
                gateOpened = false;
                gateArmed = false;   // one auto-open per Y press
            }
        }
        wasInZone = inZone;

        // --- Shoot-on-the-move aiming (turret + RPM track the goal the whole time) ---
        if (Shooter.INSTANCE.MODE == Shooter.Mode.ODOMETRY) {
            X_VELOCITY = PedroComponent.follower().getVelocity().getXComponent();
            Y_VELOCITY = PedroComponent.follower().getVelocity().getYComponent();
            AimController.updateShootOnMove(X_VELOCITY, Y_VELOCITY);
            // Mechanical baseline offset (mirror: RED -0.5 / BLUE +0.5); zone offsets compose on top.
            Configuration.TURRET_OFFSET = red() ? -0.5 : 0.5;
        }

        telemetry.addData("Alliance", Configuration.ALLIANCE);
        telemetry.addData("Gate run", gateRun);
        telemetry.addData("Armed (Y)", gateArmed);
        telemetry.addData("Gate auto-open", gateOpened);
        telemetry.addData("In close-shoot zone", inZone);
        telemetry.addData("Follower busy", PedroComponent.follower().isBusy());
        telemetry.addData("X", String.format(java.util.Locale.US, "%.1f", Configuration.CURRENT_POSE.getX()));
        telemetry.addData("Y", String.format(java.util.Locale.US, "%.1f", Configuration.CURRENT_POSE.getY()));
        telemetry.addData("Heading", String.format(java.util.Locale.US, "%.1f", Math.toDegrees(Configuration.CURRENT_POSE.getHeading())));
        telemetry.addData("Target / Cur RPM", String.format(java.util.Locale.US, "%.0f / %.0f",
                Shooter.INSTANCE.TARGET_RPM, Shooter.INSTANCE.CURRENT_RPM));
        telemetry.addLine("Y: arm+gate run | X: break+disarm | RB: fire | A/B: intake | Dpad-L: relocalize | Dpad-U: alliance");

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
