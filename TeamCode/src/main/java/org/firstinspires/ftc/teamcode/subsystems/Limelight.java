package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.List;
import java.util.Locale;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Configuration;


public class Limelight implements Subsystem {
    public static final Limelight INSTANCE = new Limelight();

    private Limelight3A limelight;
    private IMU imu;
    public LLResult LIMELIGHT_RESULT;

    public enum Mode {
        LOCALIZATION,
        MOTIF_DETECTION
    }

    public Mode MODE = Mode.LOCALIZATION, LAST_MODE = null;
    public double tx, ty, ta;
    public int POLL_COUNT = 0, VALID_RESULT_COUNT = 0, BOT_POSE_NULL_COUNT = 0, MEASUREMENT_SENT_COUNT = 0, LAST_MEASURED_TIMESTAMP = 0;
    public Pose3D LAST_RAW_POST_POSE = null;
    public Pose LAST_PEDRO_POSE = null;
    public boolean DEBUG_TELEMETRY = false;
    public boolean autoUpdateEnabled = false;

    private Limelight() {}

    /** Always returns the correct localization pipeline for the current alliance. */
    private int getLocalizationPipeline() {
        return Configuration.ALLIANCE == Configuration.Alliance.RED
                ? Configuration.RED_LIMELIGHT_PIPELINE
                : Configuration.BLUE_LIMELIGHT_PIPELINE;
    }

    @Override
    public void initialize() {
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, Configuration.LIMELIGHT);
        imu = ActiveOpMode.hardwareMap().get(IMU.class, "imu");

        // Reset singleton state for fresh run
        LAST_MODE = null;
        POLL_COUNT = 0;
        VALID_RESULT_COUNT = 0;
        BOT_POSE_NULL_COUNT = 0;
        MEASUREMENT_SENT_COUNT = 0;
        LAST_MEASURED_TIMESTAMP = 0;
        LAST_RAW_POST_POSE = null;
        LAST_PEDRO_POSE = null;

        limelight.start();
    }

    @Override
    public void periodic() {
        Telemetry t = ActiveOpMode.telemetry();
        LLStatus status = limelight.getStatus();

        t.addLine();
        t.addData("----- Limelight Status -----", "");
        t.addData("LL Connected Status", limelight.isConnected());
        t.addData("LL Running Status", limelight.isRunning());

        if (DEBUG_TELEMETRY) {
            t.addLine();
            t.addData("----- Limelight Debug -----", "");
            t.addData("LL Name", "%s", status.getName());
            t.addData("LL State", "Temp: %.1fC, CPU: %.1f%%, FPS: %d", status.getTemp(), status.getCpu(), (int) status.getFps());
            t.addData("Pipeline", "Index: %d, Type: %s", status.getPipelineIndex(), status.getPipelineType());
            t.addData("LL Mode", MODE.name());
        }

        if (MODE != LAST_MODE) {
            switch (MODE) {
                case LOCALIZATION:
                    limelight.pipelineSwitch(getLocalizationPipeline());
                    break;
                case MOTIF_DETECTION:
                    limelight.pipelineSwitch(Configuration.MOTIF_LIMELIGHT_PIPELINE);
                    break;
            }
            LAST_MODE = MODE;
        }

        if (MODE == Mode.LOCALIZATION) {
            limelight.updateRobotOrientation((Configuration.CURRENT_POSE.getHeading() + 90) % 360);
            t.addLine();
            t.addData("Yaw:", (Configuration.CURRENT_POSE.getHeading() + 90) % 360);
        }

        LIMELIGHT_RESULT = limelight.getLatestResult();
        POLL_COUNT++;

        if (LIMELIGHT_RESULT != null) {

            double captureLatency = LIMELIGHT_RESULT.getCaptureLatency();
            double targetingLatency = LIMELIGHT_RESULT.getTargetingLatency();

            switch (MODE) {
                case LOCALIZATION:
                    if (LIMELIGHT_RESULT.isValid()) {
                        VALID_RESULT_COUNT++;
                        double in_ = 39.37007874;
                        Pose3D mt1Pose = LIMELIGHT_RESULT.getBotpose();
                        Pose3D mt2Pose = LIMELIGHT_RESULT.getBotpose_MT2();

                        if (mt1Pose != null && mt2Pose != null) {
                            LAST_RAW_POST_POSE = mt2Pose;

                            Pose pedro1 = new Pose(
                                    (mt1Pose.getPosition().x * in_) - 4.0,
                                    (mt1Pose.getPosition().y * in_) - 4.0,
                                    mt1Pose.getOrientation().getYaw(AngleUnit.RADIANS),
                                    InvertedFTCCoordinates.INSTANCE
                            ).getAsCoordinateSystem(PedroCoordinates.INSTANCE);

                            Pose pedro2 = new Pose(
                                    (mt2Pose.getPosition().x * in_) - 4.0,
                                    (mt2Pose.getPosition().y * in_) - 4.0,
                                    mt2Pose.getOrientation().getYaw(AngleUnit.RADIANS),
                                    InvertedFTCCoordinates.INSTANCE
                            ).getAsCoordinateSystem(PedroCoordinates.INSTANCE);

                            pedro1 = new Pose(pedro1.getX(), pedro1.getY(), Double.NaN);
                            LAST_PEDRO_POSE = pedro1;

                            if (DEBUG_TELEMETRY) {
                                double parseLatency = LIMELIGHT_RESULT.getParseLatency();

                                tx = LIMELIGHT_RESULT.getTx();
                                ty = LIMELIGHT_RESULT.getTy();
                                ta = LIMELIGHT_RESULT.getTa();

                                t.addLine();
                                t.addData("----- Limelight Target -----", "");
                                t.addData("Target X", tx);
                                t.addData("Target Y", ty);
                                t.addData("Target Area", ta);

                                t.addLine();
                                t.addData("----- Limelight Poses -----", "");
                                t.addData("MT1 Pose", mt1Pose);
                                t.addData("MT2 Pose", mt2Pose);
                                t.addData("MT1 Pedro Pose", pedro1);
                                t.addData("MT2 Pedro", pedro2);
                                t.addData("Last Pedro", LAST_PEDRO_POSE);

                                t.addLine();
                                t.addData("----- Limelight Measurements -----", "");
                                t.addData("Measurements Sent", MEASUREMENT_SENT_COUNT);
                                t.addData("Valid Results", VALID_RESULT_COUNT);
                                t.addData("Polls", POLL_COUNT);
                                t.addData("Botpose Nulls", BOT_POSE_NULL_COUNT);

                                t.addLine();
                                t.addData("----- Limelight Latency -----", "");
                                t.addData("Parse Latency (ms)", String.format(Locale.US, "%.1f", parseLatency * 1e3));
                                t.addData("Staleness (ms)", String.format(Locale.US, "%.1f", LIMELIGHT_RESULT.getStaleness() / 1e6));
                                t.addData("Capture Latency (ms)", String.format(Locale.US, "%.1f", captureLatency * 1e3));
                                t.addData("Targeting Latency (ms)", String.format(Locale.US, "%.1f", targetingLatency * 1e3));
                                t.addData("Total Latency (ms)", String.format(Locale.US, "%.1f", (captureLatency + targetingLatency) * 1e3));
                            }

                            long captureTime = System.nanoTime() - (long) ((captureLatency + targetingLatency) * 1e9);
                            if (Configuration.fusionLocalizer != null) {
                                try {
                                    Configuration.fusionLocalizer.addMeasurement(pedro1, captureTime);
                                    MEASUREMENT_SENT_COUNT++;
                                } catch (Exception e) {
                                    if (DEBUG_TELEMETRY) {
                                        t.addData("Fusion Error", e.getMessage());
                                    }
                                }
                            }
                            LAST_MEASURED_TIMESTAMP = (int) captureTime;
                        } else {
                            BOT_POSE_NULL_COUNT++;
                        }
                    }
                    break;
                case MOTIF_DETECTION:
                    // TODO: Implement full motif detection if available
                    List<LLResultTypes.FiducialResult> fiducials = LIMELIGHT_RESULT.getFiducialResults();
                    if (fiducials.size() == 1) {
                        int id = fiducials.get(0).getFiducialId();
                        // TODO: Handle detected fiducial ID
                    }
                    break;
            }
        }
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
//            updateLocalization();
        });
    }

    public Command pause() {
        return new InstantCommand(() -> limelight.pause());
    }
}
