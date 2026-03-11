package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Configuration;

import static org.firstinspires.ftc.teamcode.subsystems.Light.AZURE;
import static org.firstinspires.ftc.teamcode.subsystems.Light.BLUE;

public class Limelight implements Subsystem {
    public static final Limelight INSTANCE = new Limelight();

    private Limelight3A limelight;
    public LLResult limelightResult;
    private int pipeline;
    public boolean autoUpdateEnabled = true;

    // Limelight position relative to robot center when turret angle = 0.
    // Positive X = forward, Positive Y = left/right depending on your field coord setup.
    // Tune these to match your robot.
    public static double LIMELIGHT_OFFSET_X_METERS = 0.0;
    public static double LIMELIGHT_OFFSET_Y_METERS = 0.0;

    // Extra fixed angular offset if camera is not perfectly aligned with turret zero
    public static double LIMELIGHT_TURRET_YAW_OFFSET_DEG = 0.0;

    // --- Debug diagnostics ---
    public int pollCount = 0;
    public int validResultCount = 0;
    public int botposeNullCount = 0;
    public int measurementSentCount = 0;
    public long lastMeasurementTimestamp = 0;

    public Pose lastPedroPose = null;
    public Pose3D lastRawBotpose = null;

    // corrected robot-frame values before conversion
    public double lastCorrectedX = 0.0;
    public double lastCorrectedY = 0.0;
    public double lastCorrectedHeadingDeg = 0.0;

    private Limelight() {}

    @Override
    public void initialize() {
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, Configuration.LIMELIGHT);

        if (Configuration.ALLIANCE == Configuration.ALLIANCE.RED) {
            pipeline = Configuration.RED_LIMELIGHT_PIPELINE;
        } else {
            pipeline = Configuration.BLUE_LIMELIGHT_PIPELINE;
        }

        limelight.pipelineSwitch(pipeline);
        limelight.start();
    }

    @Override
    public void periodic() {
        if (autoUpdateEnabled) {
            Light.INSTANCE.setBlinkingColor(AZURE, 500, Light.Target.ROBOT).schedule();
            updateLocalization();
        }
    }

    private void updateLocalization() {
        limelightResult = limelight.getLatestResult();
        pollCount++;

        if (limelightResult != null && limelightResult.isValid()) {
            validResultCount++;

            Pose3D botpose3D = limelightResult.getBotpose();
            if (botpose3D != null) {
                lastRawBotpose = botpose3D;

                // Raw Limelight pose (this is effectively the camera/turret pose)
                double camX = botpose3D.getPosition().x;
                double camY = botpose3D.getPosition().y;
                double camYawDeg = botpose3D.getOrientation().getYaw();

                // Turret angle relative to robot
                double turretDeg = Turret.INSTANCE.TURRET_ANGLE + LIMELIGHT_TURRET_YAW_OFFSET_DEG;
                double turretRad = Math.toRadians(turretDeg);

                // Rotate the camera offset by turret angle
                double rotatedOffsetX =
                        LIMELIGHT_OFFSET_X_METERS * Math.cos(turretRad)
                                - LIMELIGHT_OFFSET_Y_METERS * Math.sin(turretRad);

                double rotatedOffsetY =
                        LIMELIGHT_OFFSET_X_METERS * Math.sin(turretRad)
                                + LIMELIGHT_OFFSET_Y_METERS * Math.cos(turretRad);

                // Convert camera pose -> robot pose
                double robotX = camX - rotatedOffsetX;
                double robotY = camY - rotatedOffsetY;
                double robotYawDeg = normalizeDegrees(camYawDeg - turretDeg);

                lastCorrectedX = robotX;
                lastCorrectedY = robotY;
                lastCorrectedHeadingDeg = robotYawDeg;

                Pose2D correctedBotpose2D = new Pose2D(
                        DistanceUnit.METER,
                        robotX,
                        robotY,
                        AngleUnit.DEGREES,
                        robotYawDeg
                );

                Pose ftcPose = PoseConverter.pose2DToPose(correctedBotpose2D, InvertedFTCCoordinates.INSTANCE);
                Pose pedroPose = ftcPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
                lastPedroPose = pedroPose;

                long timestamp = System.nanoTime() - limelightResult.getStaleness();
                lastMeasurementTimestamp = timestamp;
                measurementSentCount++;

                Configuration.fusionLocalizer.addMeasurement(pedroPose, timestamp);

                if (!autoUpdateEnabled) {
                    Light.INSTANCE.setColor(BLUE, Light.Target.ROBOT).schedule();
                }
            } else {
                botposeNullCount++;
            }
        }
    }

    private double normalizeDegrees(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    public Command start() {
        return new InstantCommand(() -> limelight.start());
    }

    public Command stop() {
        return new InstantCommand(() -> limelight.stop());
    }

    public Command enableAutoUpdate() {
        return new InstantCommand(() -> autoUpdateEnabled = true);
    }

    public Command disableAutoUpdate() {
        Light.INSTANCE.setColor(BLUE, Light.Target.ROBOT).schedule();
        return new InstantCommand(() -> autoUpdateEnabled = false);
    }

    public Command update() {
        return new InstantCommand(() -> {
            if (autoUpdateEnabled) return;
            Light.INSTANCE.setColor(AZURE, Light.Target.ROBOT).schedule();
            updateLocalization();
        });
    }

    public Command pause() {
        return new InstantCommand(() -> limelight.pause());
    }
}