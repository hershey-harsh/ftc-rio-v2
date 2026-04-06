package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Configuration;


public class Limelight implements Subsystem {
    public static final Limelight INSTANCE = new Limelight();

    private Limelight3A limelight;
    public LLResult limelightResult;
    private int pipeline;
    public boolean autoUpdateEnabled = true;

    // Limelight position relative to robot center when turret angle = 0 (in inches).
    // Positive X = forward, Positive Y = left/right depending on your field coord setup.
    // Tune these to match your robot.
    public static double LIMELIGHT_OFFSET_X_INCHES = 0.0;
    public static double LIMELIGHT_OFFSET_Y_INCHES = 0.0;

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

    public Pose3D botpose3D;

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
            updateLocalization();
        }
    }

    private void updateLocalization() {
        // Feed robot orientation + turret angle for MegaTag 2
        double robotHeadingDeg = Math.toDegrees(Configuration.CURRENT_POSE.getHeading());
        double turretDeg = Turret.INSTANCE.TURRET_ANGLE + LIMELIGHT_TURRET_YAW_OFFSET_DEG;
        limelight.updateRobotOrientation(robotHeadingDeg + turretDeg);

        limelightResult = limelight.getLatestResult();
        pollCount++;

        if (limelightResult != null && limelightResult.isValid()) {
            validResultCount++;

            botpose3D = limelightResult.getBotpose_MT2();
            if (botpose3D == null) {
                botpose3D = limelightResult.getBotpose();
            }

            if (botpose3D != null) {
                lastRawBotpose = botpose3D;

                // Convert Limelight meters to inches
                double xInches = botpose3D.getPosition().x * 39.3701;
                double yInches = botpose3D.getPosition().y * 39.3701;
                double yawRad = botpose3D.getOrientation().getYaw(AngleUnit.RADIANS) + Math.PI / 2;

                // Account for turret-mounted camera offset
                double turretRad = Math.toRadians(turretDeg);
                double rotatedOffsetX =
                        LIMELIGHT_OFFSET_X_INCHES * Math.cos(turretRad)
                                - LIMELIGHT_OFFSET_Y_INCHES * Math.sin(turretRad);
                double rotatedOffsetY =
                        LIMELIGHT_OFFSET_X_INCHES * Math.sin(turretRad)
                                + LIMELIGHT_OFFSET_Y_INCHES * Math.cos(turretRad);

                double robotX = xInches - rotatedOffsetX;
                double robotY = yInches - rotatedOffsetY;

                lastCorrectedX = robotX;
                lastCorrectedY = robotY;
                lastCorrectedHeadingDeg = Math.toDegrees(yawRad);

                Pose pedroPose = new Pose(
                        robotX,
                        robotY,
                        yawRad,
                        InvertedFTCCoordinates.INSTANCE
                ).getAsCoordinateSystem(PedroCoordinates.INSTANCE);

                lastPedroPose = pedroPose;

                long timestamp = System.nanoTime() - limelightResult.getStaleness();
                lastMeasurementTimestamp = timestamp;
                measurementSentCount++;

                if (Configuration.fusionLocalizer != null) {
                    Configuration.fusionLocalizer.addMeasurement(pedroPose, timestamp);
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
        return new InstantCommand(() -> autoUpdateEnabled = false);
    }

    public Command update() {
        return new InstantCommand(() -> {
            if (autoUpdateEnabled) return;
            updateLocalization();
        });
    }

    public Command pause() {
        return new InstantCommand(() -> limelight.pause());
    }
}