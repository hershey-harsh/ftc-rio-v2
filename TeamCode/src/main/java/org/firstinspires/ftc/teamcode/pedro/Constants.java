package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.localization.FusionLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.drivetrains.Mecanum;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Configuration;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.48155687)
            //TODO: Fine tune kP for predictive braking. Run LineTest and adjust kP to your liking.
            // kP usually ranges from 0.05-0.3. kP changes are harder to notice and have minimal effects.
            // However, tune kP as high as possible so it will give you the most holding strength and accuracy
            // but without jittering the robot. You may also lower kP if you want smoother reactions.
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.25, 0.1413192,0.0013711))
            .headingPIDFCoefficients(new PIDFCoefficients(1.2, 0, 0.08, 0.025)); //TODO: Fine tune these values using automatic tuner
    //.secondaryHeadingPIDFCoefficients((new PIDFCoefficients(1, 0, 0.01, 0.025))) //TODO: Fine tune these values
    //.useSecondaryHeadingPIDF(true)

    public static PathConstraints pathConstraints = new PathConstraints(0.95, 100, 1, 1);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName(Configuration.ODOMETRY_POD)
            .forwardPodY(-3.375)
            .strafePodX(-6)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .xVelocity(79.1942)
            .yVelocity(56.8602)
            .maxPower(1)
            .rightFrontMotorName(Configuration.RIGHT_FRONT_MOTOR)
            .rightRearMotorName(Configuration.RIGHT_REAR_MOTOR)
            .leftRearMotorName(Configuration.LEFT_REAR_MOTOR)
            .leftFrontMotorName(Configuration.LEFT_FRONT_MOTOR)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE).useBrakeModeInTeleOp(true)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE).useBrakeModeInTeleOp(true)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD).useBrakeModeInTeleOp(true)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD).useBrakeModeInTeleOp(true);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
    public static Follower createFusionFollower(HardwareMap hardwareMap) {
        Configuration.fusionLocalizer = new FusionLocalizer(
                new PinpointLocalizer(hardwareMap, localizerConstants),
                // Accuracy of initial position
                // x (inches), y (inches), angle (radians)
                new Pose(0.25, 0.25),   // P: initial covariance
                // Error accumulation of localizer over time
                // x (in^2/s^2), y (in^2/s^2), angle (rad^2/s^s)
                new Pose(0.0005, 0.0005),    // Q: process variance
                // Accuracy of measurement to be fused (usually vision)
                // x (inches), y (inches), angle (radians)
                new Pose(1.0, 1.0),   // R: measurement variance
                // Path history points to retain. You must retain enough
                // history to account for the delay of vision detections.
                // The # of points is dependent on the speed of your control
                // loop and the latency of your computer vision system.
                // For a 50Hz loop, this would give you 2s of history
                100                              // bufferSize
        );

        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .setLocalizer(Configuration.fusionLocalizer)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}