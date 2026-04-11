package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.driving.DriverControlledCommand;

import static dev.nextftc.bindings.Bindings.*;
import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name = "Shot", group = "Debug")
public class ShooterDebug extends NextFTCOpMode {
    double LOOP_TIME = 0;
    ElapsedTime LOOP_TIMER = new ElapsedTime();

    private DriverControlledCommand driverControlled;

    private static final double RPM_STEP = 100.0;
    private static final double HOOD_STEP = 1.0;

    private enum DebugMode { MANUAL, LUT_TEST, KINEMATIC }
    private DebugMode debugMode = DebugMode.MANUAL;

    public ShooterDebug() {
        addComponents(
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Shooter.INSTANCE),
                new SubsystemComponent(Transfer.INSTANCE),
                new SubsystemComponent(Turret.INSTANCE)
        );
    }

    @Override
    public void onInit() {
        Configuration.ALLIANCE = Configuration.Alliance.RED;
        PedroComponent.follower().setStartingPose(new com.pedropathing.geometry.Pose(72, 72, Math.toRadians(270)));
        Configuration.CURRENT_POSE = PedroComponent.follower().getPose();

        Shooter.INSTANCE.MODE = Shooter.Mode.MANUAL;
        Turret.INSTANCE.mode = Turret.Mode.odometry;
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

        Shooter.INSTANCE.start().schedule();
        Turret.INSTANCE.start().schedule();

        // DPad Up — increase flywheel RPM by 100
        button(() -> gamepad1.dpad_up)
                .whenBecomesTrue(() -> {
                    Shooter.INSTANCE.TARGET_RPM += RPM_STEP;
                });

        // DPad Down — decrease flywheel RPM by 100
        button(() -> gamepad1.dpad_down)
                .whenBecomesTrue(() -> {
                    Shooter.INSTANCE.TARGET_RPM = Math.max(0, Shooter.INSTANCE.TARGET_RPM - RPM_STEP);
                });

        // DPad Right — increase hood angle by 1°
        button(() -> gamepad1.dpad_right)
                .whenBecomesTrue(() -> {
                    Shooter.INSTANCE.setHoodAngle(Shooter.INSTANCE.HOOD_ANGLE + HOOD_STEP);
                });

        // DPad Left — decrease hood angle by 1°
        button(() -> gamepad1.dpad_left)
                .whenBecomesTrue(() -> {
                    Shooter.INSTANCE.setHoodAngle(Shooter.INSTANCE.HOOD_ANGLE - HOOD_STEP);
                });

        // Right Bumper — open gate (hold), close on release
        button(() -> gamepad1.right_bumper)
                .whenBecomesTrue(() -> Transfer.INSTANCE.openGate().schedule())
                .whenBecomesFalse(() -> Transfer.INSTANCE.closeGate().schedule());

        // A — run intake
        button(() -> gamepad1.a)
                .whenBecomesTrue(() -> Transfer.INSTANCE.intake().schedule());

        // B — stop intake
        button(() -> gamepad1.b)
                .whenBecomesTrue(() -> Transfer.INSTANCE.stop().schedule());

        // Y — cycle debug mode: MANUAL → LUT_TEST → KINEMATIC → MANUAL
        button(() -> gamepad1.y)
                .whenBecomesTrue(() -> {
                    if (debugMode == DebugMode.MANUAL) {
                        debugMode = DebugMode.LUT_TEST;
                    } else if (debugMode == DebugMode.LUT_TEST) {
                        debugMode = DebugMode.KINEMATIC;
                    } else {
                        debugMode = DebugMode.MANUAL;
                    }
                    Shooter.INSTANCE.MODE = Shooter.Mode.MANUAL;
                });
    }

    @Override
    public void onUpdate() {
        LOOP_TIMER.reset();

        Configuration.CURRENT_POSE = PedroComponent.follower().getPose();

        double goalDistInches = Shooter.INSTANCE.GOAL_DISTANCE * 39.37;
        double velocityMS = Shooter.INSTANCE.rpmToArtifactMSVelocity(Shooter.INSTANCE.TARGET_RPM);

        // Apply LUT values when in LUT test mode
//        Shooter.ShotParameters lutParams = Shooter.getShooterValues(Shooter.INSTANCE.GOAL_DISTANCE);
//        if (debugMode == DebugMode.LUT_TEST) {
//            Shooter.INSTANCE.TARGET_RPM = Shooter.artifactVelocityMStoRPM(lutParams.ARTIFACT_VELOCITY);
//            Shooter.INSTANCE.setHoodAngle(lutParams.HOOD_ANGLE);
//        }

        // Apply kinematic RPM when in KINEMATIC mode
        if (debugMode == DebugMode.KINEMATIC) {

            Shooter.INSTANCE.updateKinematics(
                    Shooter.INSTANCE.GOAL_DISTANCE,
                    Math.toRadians(Shooter.INSTANCE.HOOD_ANGLE)
            );
            Shooter.INSTANCE.TARGET_RPM = Shooter.INSTANCE.KINEMATIC_RPM_GOAL * Configuration.RPM_MULTIPLER;
        }

        // --- Telemetry ---
        telemetry.addData("Loop (ms)", LOOP_TIME);
        telemetry.addData("Loop (hz)", LOOP_TIME > 0 ? (1000.0 / LOOP_TIME) : 0);
        telemetry.addData("Mode", debugMode.name());

        telemetry.addLine();
        telemetry.addData("=== Goal ===", "");
        telemetry.addData("Distance (in)", "%.2f", goalDistInches);
        telemetry.addData("Distance (m)", "%.3f", Shooter.INSTANCE.GOAL_DISTANCE);

        telemetry.addLine();
        telemetry.addData("=== Flywheel ===", "");
        telemetry.addData("Target RPM", Shooter.INSTANCE.TARGET_RPM);
//        telemetry.addData("Read RPM", Shooter.INSTANCE.READ_RPM);
        telemetry.addData("Motor Right", Shooter.velocityToRPM((Shooter.INSTANCE.flywheelMotor1.getVelocity())));
        telemetry.addData("Motor Left", Shooter.velocityToRPM((Shooter.INSTANCE.flywheelMotor2.getVelocity())));
        telemetry.addData("Velocity (m/s)", "%.3f", velocityMS);

        telemetry.addLine();
        telemetry.addData("=== Hood ===", "");
        telemetry.addData("Hood Angle (deg)", Shooter.INSTANCE.HOOD_ANGLE);
        telemetry.addData("Hood Position", "%.4f", Shooter.INSTANCE.HOOD_POSITION);

        telemetry.addLine();
        telemetry.addData("=== LUT Output ===", "");
//        telemetry.addData("LUT Distance (m)", "%.3f", lutParams.DISTANCE);
//        telemetry.addData("LUT Velocity (m/s)", "%.3f", lutParams.ARTIFACT_VELOCITY);
//        telemetry.addData("LUT RPM", "%.0f", Shooter.artifactVelocityMStoRPM(lutParams.ARTIFACT_VELOCITY));
//        telemetry.addData("LUT Hood Angle (deg)", "%.1f", lutParams.HOOD_ANGLE);
//        telemetry.addData("LUT Time of Flight (s)", "%.3f", lutParams.TIME_OF_FLIGHT);

        telemetry.addLine();
        telemetry.addData("=== Kinematic Output ===", "");
        telemetry.addData("Kinematic RPM Goal", "%.0f", Shooter.INSTANCE.KINEMATIC_RPM_GOAL);

        telemetry.addLine();
        telemetry.addLine("--- Controls ---");
        telemetry.addLine("DPad Up/Down: RPM +/- 100");
        telemetry.addLine("DPad Left/Right: Hood -/+ 1°");
        telemetry.addLine("RB: Open gate (hold)");
        telemetry.addLine("A: Intake | B: Stop intake");
        telemetry.addLine("Y: Cycle mode (MANUAL→LUT→KINEMATIC)");

        BindingManager.update();
        telemetry.update();

        LOOP_TIME = LOOP_TIMER.milliseconds();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
    }
}
