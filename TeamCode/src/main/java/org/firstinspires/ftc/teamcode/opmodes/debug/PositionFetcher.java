package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;

import static dev.nextftc.bindings.Bindings.*;

import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

@Disabled
@TeleOp(name = "Position Fetcher (Debug)", group = "Debug")
public class PositionFetcher extends NextFTCOpMode {
    double LOOP_TIME = 0;
    ElapsedTime LOOP_TIMER = new ElapsedTime();
//    private static final double START_X = 72;
//    private static final double START_Y = 72;
//    private static final double START_HEADING = 270;

    private static final double START_X = 25.985;
    private static final double START_Y = 126.629;
    private static final double START_HEADING = 136;

    // Saved positions list
    private final List<Pose> savedPositions = new ArrayList<>();

    public PositionFetcher() {
        addComponents(
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Transfer.INSTANCE)
        );
    }

    @Override
    public void onInit() {
        // Register gamepad axes (Bindings system)
        Gamepads.gamepad1().leftStickY();
        Gamepads.gamepad1().leftStickX();
        Gamepads.gamepad1().rightStickX();

        // Use whatever the current configured pose is as starting pose so the follower reports location
        Pose startPose = new Pose(START_X, START_Y, Math.toRadians(START_HEADING));
        PedroComponent.follower().setStartingPose(startPose);

        telemetry.addLine("=== Position Fetcher (Debug) ===");
        telemetry.addLine("Use the sticks to drive the robot.");
        telemetry.addLine("Press A on gamepad1 to save a position.");
        telemetry.addLine("Press B on gamepad1 to clear all saved positions.");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // Create driver controlled command so you can drive with joysticks during teleop
        PedroDriverControlled driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().negate(),
                !Configuration.FIELD_CENTRIC
        );

        driverControlled.schedule();

        button(() -> gamepad1.a)
                .whenTrue(() -> {
                    Pose p = PedroComponent.follower().getPose();
                    savedPositions.add(new Pose(p.getX(), p.getY(), p.getHeading()));
                });

        button(() -> gamepad1.b)
                .whenTrue(() -> savedPositions.clear());
    }

    @Override
    public void onUpdate() {
        LOOP_TIMER.reset();

        // Update follower so we have current pose
        PedroComponent.follower().update();

        Pose currentPose = PedroComponent.follower().getPose();
        double headingDeg = Math.toDegrees(currentPose.getHeading());


        // Telemetry
        telemetry.addLine("=== Current Position ===");
        telemetry.addData("X", String.format(Locale.US, "%.3f", currentPose.getX()));
        telemetry.addData("Y", String.format(Locale.US, "%.3f", currentPose.getY()));
        telemetry.addData("Heading (deg)", String.format(Locale.US, "%.2f", headingDeg));
        telemetry.addData("Gate", Transfer.INSTANCE.gate1And2Sensor.getState());
        telemetry.addLine("");
        telemetry.addLine("Press A to save current position");
        telemetry.addLine("Press B to clear saved positions");
        telemetry.addLine("");

        telemetry.addLine("=== Saved Positions (" + savedPositions.size() + ") ===");
        for (int i = 0; i < savedPositions.size(); i++) {
            Pose p = savedPositions.get(i);
            telemetry.addLine(String.format(Locale.US, "#%d: X=%.3f, Y=%.3f, H=%.2f°",
                    i + 1, p.getX(), p.getY(), Math.toDegrees(p.getHeading())));
        }

        telemetry.addLine("");
        telemetry.addData("Loop Time (ms)", String.format(Locale.US, "%.2f", LOOP_TIME));
        telemetry.addData("Loop Rate (hz)", LOOP_TIME > 0 ? String.format(Locale.US, "%.1f", 1000.0 / LOOP_TIME) : "---");

        telemetry.update();

        LOOP_TIME = LOOP_TIMER.milliseconds();
    }

    @Override
    public void onStop() {
        BindingManager.reset();

        telemetry.setAutoClear(false);
        telemetry.addLine("=== Final Position ===");
        telemetry.addData("X", String.format(Locale.US, "%.3f", PedroComponent.follower().getPose().getX()));
        telemetry.addData("Y", String.format(Locale.US, "%.3f", PedroComponent.follower().getPose().getY()));
        telemetry.addData("Heading (deg)", String.format(Locale.US, "%.2f", Math.toDegrees(PedroComponent.follower().getPose().getHeading())));
        telemetry.addLine("");

        telemetry.addLine("=== All Saved Positions ===");
        for (int i = 0; i < savedPositions.size(); i++) {
            Pose p = savedPositions.get(i);
            telemetry.addLine(String.format(Locale.US, "#%d: X=%.3f, Y=%.3f, H=%.2f°",
                    i + 1, p.getX(), p.getY(), Math.toDegrees(p.getHeading())));
        }

        telemetry.addLine("");
        telemetry.addLine("=== Copy-Paste Java Code ===");
        for (int i = 0; i < savedPositions.size(); i++) {
            Pose p = savedPositions.get(i);
            telemetry.addLine(String.format(Locale.US, "new Pose(%.3f, %.3f, Math.toRadians(%.2f))",
                    p.getX(), p.getY(), Math.toDegrees(p.getHeading())));
        }

        telemetry.update();
    }
}
