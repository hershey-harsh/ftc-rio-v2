package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.pedropathing.geometry.Pose;
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

@TeleOp(name = "Shooter", group = "Debug")
public class ShooterDebug extends NextFTCOpMode {
    double LOOP_TIME = 0;
    ElapsedTime LOOP_TIMER = new ElapsedTime();

    private DriverControlledCommand driverControlled;

    private static final double HOOD_STEP = 1.0; // degrees per dpad press
    private double userBaselineHoodAngle = 56.5; // tracks dpad-controlled baseline separately

    // Logged data entries (appended on each A press)
    private final StringBuilder logEntries = new StringBuilder();
    private int logCount = 0;

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
        PedroComponent.follower().setStartingPose(new Pose(72, 72, Math.toRadians(270)));
        Configuration.CURRENT_POSE = PedroComponent.follower().getPose();

        // Shooter stays in MANUAL mode so periodic() does NOT auto-set hood
        // RPM will be driven from onUpdate using the math model
        Shooter.INSTANCE.mode = Shooter.Mode.manual;

        // Turret tracks goal by default
        Turret.INSTANCE.mode = Turret.Mode.odometry;

        // Enable hood servos and set a sensible starting angle
        Shooter.INSTANCE.hoodServo1.getServo().getController().pwmEnable();
        Shooter.INSTANCE.hoodServo2.getServo().getController().pwmEnable();
        Shooter.INSTANCE.setHoodAngle(56.5);
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

        // Start shooter (sets started=true so periodic() runs, keeps manual mode so hood isn't auto-overridden)
        Shooter.INSTANCE.start().schedule();

        // Intake runs by default on start
        Transfer.INSTANCE.intake().schedule();

        // Turret servos on
        Turret.INSTANCE.start().schedule();

        // D-Pad Up — increase hood angle manually
        button(() -> gamepad1.dpad_up)
                .whenBecomesTrue(() -> {
                    userBaselineHoodAngle = Math.min(userBaselineHoodAngle + HOOD_STEP, Shooter.INSTANCE.MAX_HOOD_ANGLE);
                    Shooter.INSTANCE.setHoodAngle(userBaselineHoodAngle);
                });

        // D-Pad Down — decrease hood angle manually
        button(() -> gamepad1.dpad_down)
                .whenBecomesTrue(() -> {
                    userBaselineHoodAngle = Math.max(userBaselineHoodAngle - HOOD_STEP, Shooter.INSTANCE.MIN_HOOD_ANGLE);
                    Shooter.INSTANCE.setHoodAngle(userBaselineHoodAngle);
                });

        // Left Bumper — close gate
        button(() -> gamepad1.left_bumper)
                .whenBecomesTrue(() -> Transfer.INSTANCE.closeGate().schedule());

        // Right Bumper — open gate
        button(() -> gamepad1.right_bumper)
                .whenBecomesTrue(() -> Transfer.INSTANCE.openGate().schedule())
                .whenBecomesFalse(() -> Transfer.INSTANCE.closeGate().schedule());

        // A — log: distance (m), hood angle (deg), hood servo position
        button(() -> gamepad1.a)
                .whenBecomesTrue(() -> {
                    logCount++;
                    double distM = Shooter.INSTANCE.GOAL_DISTANCE * 0.0254;
                    logEntries.append(String.format(
                            "[%d] dist=%.3fm  hoodAngle=%.1f°  hoodPos=%.4f\n",
                            logCount, distM,
                            Shooter.INSTANCE.HOOD_ANGLE,
                            Shooter.INSTANCE.HOOD_POSITION
                    ));
                });

        // B — toggle hood compensation
        button(() -> gamepad1.b)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> Shooter.INSTANCE.hoodCompensationEnabled = true)
                .whenBecomesFalse(() -> Shooter.INSTANCE.hoodCompensationEnabled = false);
    }

    @Override
    public void onUpdate() {
        LOOP_TIMER.reset();

        Configuration.CURRENT_POSE = PedroComponent.follower().getPose();

        // Drive RPM from math model every loop (hood stays wherever user set it)
        double distMeters = Shooter.INSTANCE.GOAL_DISTANCE * 0.0254;
        double hoodRad = Math.toRadians(userBaselineHoodAngle);
        Shooter.INSTANCE.updateKinematics(distMeters, hoodRad);
        Shooter.INSTANCE.targetRPM = Shooter.INSTANCE.getKinematicRPMGoal() * 2.15;

        // Hood compensation: adjust hood based on actual vs target RPM
        double baselineHood = userBaselineHoodAngle;
        double compensatedHood = baselineHood;
        if (Shooter.INSTANCE.hoodCompensationEnabled) {
            Shooter.INSTANCE.setPlannedShot(
                    Shooter.INSTANCE.GOAL_DISTANCE,
                    Shooter.INSTANCE.targetRPM,
                    baselineHood
            );
            compensatedHood = Shooter.INSTANCE.physicsCompensatedHoodDeg(
                    Shooter.INSTANCE.GOAL_DISTANCE,
                    Shooter.INSTANCE.targetRPM,
                    Shooter.INSTANCE.readRPM,
                    baselineHood
            );
            Shooter.INSTANCE.setHoodAngle(compensatedHood);
        }

        // --- Telemetry ---
        telemetry.addData("Loop Time (ms)", LOOP_TIME);
        telemetry.addData("Loop Time (hz)", (1000 / LOOP_TIME));

        telemetry.addLine();
        telemetry.addData("=== Turret ===", "");
        telemetry.addData("Mode", Turret.INSTANCE.mode);
        telemetry.addData("Angle (deg)", Turret.INSTANCE.TURRET_ANGLE);
        telemetry.addData("Position", Turret.INSTANCE.TURRET_POSITION);

        telemetry.addLine();
        telemetry.addData("=== Goal ===", "");
        telemetry.addData("Distance (in)", Shooter.INSTANCE.GOAL_DISTANCE);
        telemetry.addData("Distance (m)", distMeters);

        telemetry.addLine();
        telemetry.addData("=== Hood ===", "");
        telemetry.addData("Hood Angle (deg)", Shooter.INSTANCE.HOOD_ANGLE);
        telemetry.addData("Hood Position", Shooter.INSTANCE.HOOD_POSITION);
        telemetry.addData("Math Model Angle", Shooter.INSTANCE.getHoodAngle(distMeters));

        telemetry.addLine();
        telemetry.addData("=== Shooter ===", "");
        telemetry.addData("Target RPM (model)", Shooter.INSTANCE.targetRPM);
        telemetry.addData("Read RPM", Shooter.INSTANCE.readRPM);

        telemetry.addLine();
        telemetry.addData("=== Hood Compensation ===", "");
        telemetry.addData("Enabled", Shooter.INSTANCE.hoodCompensationEnabled);
        telemetry.addData("Baseline Hood", baselineHood);
        telemetry.addData("Compensated Hood", compensatedHood);
        telemetry.addData("Delta (deg)", Shooter.INSTANCE.lastDeltaDeg);

        telemetry.addLine();
        telemetry.addData("=== Log (press A to record) ===", "");
        telemetry.addData("Entries", logCount);
        telemetry.addLine(logEntries.toString());

        telemetry.addLine();
        telemetry.addLine("--- Controls ---");
        telemetry.addLine("DPad Up/Down: Hood +/- 1°");
        telemetry.addLine("LB: Close gate  RB: Open gate");
        telemetry.addLine("A: Log distance + hood");
        telemetry.addLine("B: Toggle hood compensation");

        BindingManager.update();
        telemetry.update();

        LOOP_TIME = LOOP_TIMER.milliseconds();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
    }
}
