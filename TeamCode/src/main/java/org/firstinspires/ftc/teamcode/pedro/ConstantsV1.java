package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.control.FilteredPIDFCoefficients;
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
import com.pedropathing.geometry.BezierCurve;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Configuration;

public class ConstantsV1 {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.48932355625)
            .forwardZeroPowerAcceleration(-25.398630567541016)
            .lateralZeroPowerAcceleration(-70.3958742327889)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.03, 0, 0.05, 0.07))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.02, 0.07))
            .headingPIDFCoefficients(new PIDFCoefficients(0.7, 0, 0.03, 0.01))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.02, 0.0175))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2.5, 0, 0.1, 0.0005))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0, 0.00035, 0.6, 0.015))
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .centripetalScaling(0.0005);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(4.84252)
            .strafePodX(-6.49606)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName(Configuration.ODOMETRY_POD)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(Configuration.RIGHT_FRONT_MOTOR)
            .rightRearMotorName(Configuration.RIGHT_REAR_MOTOR)
            .leftRearMotorName(Configuration.LEFT_REAR_MOTOR)
            .leftFrontMotorName(Configuration.LEFT_FRONT_MOTOR)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE).useBrakeModeInTeleOp(true)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE).useBrakeModeInTeleOp(true)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD).useBrakeModeInTeleOp(true)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD).useBrakeModeInTeleOp(true)
            .xVelocity(70.39660692590428)
            .yVelocity(54.335191140963346);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants, followerConstants.predictiveBrakingCoefficients)
                .pinpointLocalizer(localizerConstants)
                .build();
    }

//    public static Follower createFusionFollower(HardwareMap hardwareMap) {
//        Configuration.fusionLocalizer = new FusionLocalizer(
//                new PinpointLocalizer(hardwareMap, localizerConstants),
//                // Error accumulation of localizer over time
//                // x (in^2/s^2), y (in^2/s^2), angle (rad^2/s^s)
//                new double[]{0.1, 0.1, 0.05}, // Q: process variance
//                // Accuracy of measurement to be fused (usually vision)
//                // x (inches), y (inches), angle (radians)
//                new double[]{0.5, 0.5, 0.1},// R: measurement variance
//                // Path history points to retain. You must retain enough
//                // history to account for the delay of vision detections.
//                // The # of points is dependent on the speed of your control
//                // loop and the latency of your computer vision system.
//                // For a 50Hz loop, this would give you 2s of history
//                new double[]{0.5, 0.5, 0.1},
//                100 //Buffer Size
//        );
//
//        return new FollowerBuilder(followerConstants, hardwareMap)
//                .pathConstraints(pathConstraints)
//                .setLocalizer(Configuration.fusionLocalizer)
//                .mecanumDrivetrain(driveConstants, new PredictiveBrakingCoefficients(0.3, 0.086595400785, 0.0025996451))
//                .build();
//    }
}