package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.pedropathing.geometry.Pose;
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

@Deprecated
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
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Limelight.INSTANCE)
        );
    }

    @Override
    public void onInit() {
        Configuration.ALLIANCE = Configuration.Alliance.RED;

        Pose startPose = new Pose(START_X, START_Y, Math.toRadians(START_HEADING));
        PedroComponent.follower().setStartingPose(startPose);
        Configuration.CURRENT_POSE = startPose;

        // Start with auto-update enabled
        Limelight.INSTANCE.autoUpdateEnabled = true;

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
        telemetry.addData("=== Fused Pose (output) ===", "");
        telemetry.addData("X (in)", String.format(Locale.US, "%.3f", pose.getX()));
        telemetry.addData("Y (in)", String.format(Locale.US, "%.3f", pose.getY()));
        telemetry.addData("Heading (deg)", String.format(Locale.US, "%.2f", headingDeg));

        // --- Limelight Pipeline Counters ---
        telemetry.addLine();
        telemetry.addData("=== LL Pipeline ===", "");
        telemetry.addData("Auto-Update", Limelight.INSTANCE.autoUpdateEnabled);
        telemetry.addData("Polls", Limelight.INSTANCE.pollCount);
        telemetry.addData("Valid Results", Limelight.INSTANCE.validResultCount);
        telemetry.addData("Botpose Null", Limelight.INSTANCE.botposeNullCount);
        telemetry.addData("Measurements Sent", Limelight.INSTANCE.measurementSentCount);
        telemetry.addData("Manual Updates", updateCount);
        telemetry.addData("April Tag Position", Limelight.INSTANCE.botpose3D);

        // --- Raw LL Result ---
        telemetry.addLine();
        telemetry.addData("=== LL Raw Result ===", "");
        if (Limelight.INSTANCE.limelightResult == null) {
            telemetry.addData("Result", "NULL");
        } else {
            telemetry.addData("Valid", Limelight.INSTANCE.limelightResult.isValid());
            telemetry.addData("Staleness (ns)", String.format(Locale.US, "%d", Limelight.INSTANCE.limelightResult.getStaleness()));
            telemetry.addData("Staleness (ms)", String.format(Locale.US, "%.1f", Limelight.INSTANCE.limelightResult.getStaleness() / 1e6));
        }

        // --- Raw LL Botpose ---
        telemetry.addLine();
        telemetry.addData("=== LL Raw Botpose ===", "");
        if (Limelight.INSTANCE.lastRawBotpose != null) {
            Pose3D bp = Limelight.INSTANCE.lastRawBotpose;
            telemetry.addData("X (m)", String.format(Locale.US, "%.4f", bp.getPosition().x));
            telemetry.addData("Y (m)", String.format(Locale.US, "%.4f", bp.getPosition().y));
            telemetry.addData("Z (m)", String.format(Locale.US, "%.4f", bp.getPosition().z));
            telemetry.addData("Yaw (deg)", String.format(Locale.US, "%.2f", bp.getOrientation().getYaw()));
            telemetry.addData("Pitch (deg)", String.format(Locale.US, "%.2f", bp.getOrientation().getPitch()));
            telemetry.addData("Roll (deg)", String.format(Locale.US, "%.2f", bp.getOrientation().getRoll()));
        } else {
            telemetry.addData("Botpose", "NEVER RECEIVED");
        }

        // --- Converted Pedro Pose (what gets sent to fusion) ---
        telemetry.addLine();
        telemetry.addData("=== LL -> Pedro Pose ===", "");
        if (Limelight.INSTANCE.lastPedroPose != null) {
            Pose pp = Limelight.INSTANCE.lastPedroPose;
            telemetry.addData("X (in)", String.format(Locale.US, "%.3f", pp.getX()));
            telemetry.addData("Y (in)", String.format(Locale.US, "%.3f", pp.getY()));
            telemetry.addData("Heading (deg)", String.format(Locale.US, "%.2f", Math.toDegrees(pp.getHeading())));

            // Delta between fused and LL measurement
            double dx = pose.getX() - pp.getX();
            double dy = pose.getY() - pp.getY();
            double dh = Math.toDegrees(pose.getHeading() - pp.getHeading());
            telemetry.addData("Delta X (fused-LL)", String.format(Locale.US, "%.3f", dx));
            telemetry.addData("Delta Y (fused-LL)", String.format(Locale.US, "%.3f", dy));
            telemetry.addData("Delta H (fused-LL)", String.format(Locale.US, "%.2f°", dh));
        } else {
            telemetry.addData("Pedro Pose", "NEVER CONVERTED");
        }

        // --- Fusion Localizer State ---
        telemetry.addLine();
        telemetry.addData("=== Fusion Localizer ===", "");
        if (Configuration.fusionLocalizer != null) {
            telemetry.addData("Active", true);
            telemetry.addData("isNaN", Configuration.fusionLocalizer.isNAN());

            // Timestamp sanity check
            long now = System.nanoTime();
            long lastTs = Limelight.INSTANCE.lastMeasurementTimestamp;
            if (lastTs > 0) {
                long ageNs = now - lastTs;
                telemetry.addData("Last Meas Age (ms)", String.format(Locale.US, "%.1f", ageNs / 1e6));
                telemetry.addData("Last Meas Timestamp", String.format(Locale.US, "%d", lastTs));
                telemetry.addData("System.nanoTime()", String.format(Locale.US, "%d", now));
            } else {
                telemetry.addData("Last Meas", "NONE");
            }
        } else {
            telemetry.addData("Active", "NULL — NOT CREATED!");
        }

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
