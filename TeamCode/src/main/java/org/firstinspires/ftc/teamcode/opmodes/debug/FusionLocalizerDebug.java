package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

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
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

@Disabled
@TeleOp(name = "Fusion Localizer", group = "Debug")
public class FusionLocalizerDebug extends NextFTCOpMode {
    double LOOP_TIME = 0;
    ElapsedTime LOOP_TIMER = new ElapsedTime();

    private DriverControlledCommand driverControlled;

    private static final double START_X = 72;
    private static final double START_Y = 72;
    private static final double START_HEADING = 270;

    private int updateCount = 0;

    public FusionLocalizerDebug() {
        addComponents(
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFusionFollower),
                new SubsystemComponent(Limelight.INSTANCE)
        );
    }

    @Override
    public void onInit() {
        Configuration.ALLIANCE = Configuration.Alliance.RED;

        Pose startPose = new Pose(START_X, START_Y, Math.toRadians(START_HEADING));
        PedroComponent.follower().setStartingPose(startPose);
        Configuration.CURRENT_POSE = startPose;

        Limelight.INSTANCE.DEBUG_TELEMETRY = true;

        telemetry.addLine("=== Fusion Localizer Debug ===");
        telemetry.addLine("Uses Pinpoint + Limelight fusion.");
        telemetry.addLine("Press Start to begin driving.");
        telemetry.update();
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

        // A — toggle Limelight auto-update
        button(() -> gamepad1.a)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> Limelight.INSTANCE.enableAutoUpdate().schedule())
                .whenBecomesFalse(() -> Limelight.INSTANCE.disableAutoUpdate().schedule());

        // B — manually trigger a single Limelight update (when auto-update is off)
        button(() -> gamepad1.b)
                .whenBecomesTrue(() -> {
                    Limelight.INSTANCE.update().schedule();
                    updateCount++;
                });

        // X — reset pose to start
        button(() -> gamepad1.x)
                .whenBecomesTrue(() -> {
                    Pose startPose = new Pose(START_X, START_Y, Math.toRadians(START_HEADING));
                    PedroComponent.follower().setStartingPose(startPose);
                    Configuration.CURRENT_POSE = startPose;
                });

        // Y — pause/resume Limelight hardware
        button(() -> gamepad1.y)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> Limelight.INSTANCE.pause().schedule())
                .whenBecomesFalse(() -> Limelight.INSTANCE.start().schedule());
    }

    @Override
    public void onUpdate() {
        LOOP_TIMER.reset();

        Configuration.CURRENT_POSE = PedroComponent.follower().getPose();
        Pose pose = Configuration.CURRENT_POSE;
        double headingDeg = Math.toDegrees(pose.getHeading());

        // --- Telemetry ---
        telemetry.addData("Loop Time (ms)", String.format(Locale.US, "%.2f", LOOP_TIME));
        telemetry.addData("Loop Rate (hz)", LOOP_TIME > 0 ? String.format(Locale.US, "%.1f", 1000.0 / LOOP_TIME) : "---");

        telemetry.addLine();
        telemetry.addData("X (in)", String.format(Locale.US, "%.3f", pose.getX()));
        telemetry.addData("Y (in)", String.format(Locale.US, "%.3f", pose.getY()));
        telemetry.addData("Heading (deg)", String.format(Locale.US, "%.2f", headingDeg));

        telemetry.addLine();
        telemetry.addLine("--- Controls ---");
        telemetry.addData("Sticks", "Drive");
        telemetry.addData("A", "Toggle LL Auto-Update");
        telemetry.addData("B", "Manual LL Update (single)");
        telemetry.addData("X", "Reset Pose to Start");
        telemetry.addData("Y", "Pause/Resume LL Hardware");

        BindingManager.update();
        telemetry.update();

        LOOP_TIME = LOOP_TIMER.milliseconds();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
    }
}
