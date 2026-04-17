package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.teamcode.subsystems.Light;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Autonomous(name = "RT2GA18 - R1 & R2", group = "Red Alliance")
public class RT2GA18 extends NextFTCOpMode {
    double LOOP_TIME = 0;
    ElapsedTime LOOP_TIMER = new ElapsedTime();

    private Paths paths;

    // Starting Position
    private static final double START_X = 115.715;
    private static final double START_Y = 126.628;
    private static final double START_HEADING = 45;
    private double vt = 0;

    private double X_VELOCITY = 0;
    private double Y_VELOCITY = 0;
    public double TRUE_TARGET_DEGREE = 0;

    private boolean WAS_FOLLOWER_BUSY = false;

    public RT2GA18() {
        addComponents(
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Transfer.INSTANCE),
                new SubsystemComponent(Shooter.INSTANCE),
                new SubsystemComponent(Turret.INSTANCE),
                new SubsystemComponent(Light.INSTANCE)
        );
    }

    @Override
    public void onInit() {
        Light.INSTANCE.setColor(Light.VIOLET, Light.Target.ROBOT);
        Configuration.ALLIANCE = Configuration.Alliance.RED;
        Configuration.SHOOTER_HEIGHT_TO_GOAL = 1.1;
        //TODO: times 2.15

        paths = new Paths(PedroComponent.follower());

        Pose startPose = new Pose(START_X, START_Y, Math.toRadians(START_HEADING));
        PedroComponent.follower().setStartingPose(startPose);
        Configuration.CURRENT_POSE = startPose;

        Transfer.INSTANCE.start().schedule();
        Turret.INSTANCE.start().schedule();
        Shooter.INSTANCE.start().schedule();

        telemetry.addData("Alliance:", "Red");
        telemetry.addData("Side:", "Top");
        telemetry.addData("Order:", "Preload, R2, Gate x3, R1");
        telemetry.addData("Gate:", "True");
        telemetry.addData("Solo:", "True");
        telemetry.addData("Total Count:", "18");

        telemetry.update();
        Light.INSTANCE.setBlinkingColor(Light.RED, Light.Target.ROBOT).schedule();
    }

    @Override
    public void onStartButtonPressed() {

        Shooter.INSTANCE.on().schedule();
        Transfer.INSTANCE.intake().schedule();

        new SequentialGroup(

                // TODO: All Path Chains are temporary fixes until Path Skipping is fixed through correction prioritization.
                /// Launch Preloaded Artifacts.
                new FollowPath(paths.PreloadLaunch),
                Transfer.INSTANCE.openGate(),
                new Delay(0.7),

                /// Wait 0.5 seconds and then collect R2 Artifacts and come back to launching position.

                new ParallelGroup(
                        Transfer.INSTANCE.closeGate(),
                        new SequentialGroup(
                                new FollowPath(paths.R2),
                                new Delay(0.25),
                                Transfer.INSTANCE.openGate()
                        )
                ),

                new Delay(0.7),

                /// Wait 0.5 seconds and then collect G1 Artifacts and come back to launching position.

                new ParallelGroup(
                        Transfer.INSTANCE.closeGate(),
                        new SequentialGroup(
                                new FollowPath(paths.Gate1_1),
                                new Delay(1.75),
                                new FollowPath(paths.Gate1_3),
                                new Delay(0.25),
                                Transfer.INSTANCE.openGate()
                        )
                ),

                new Delay(0.7),

                /// Wait 0.5 seconds and then collect G2 Artifacts and come back to launching position.

                new ParallelGroup(
                        Transfer.INSTANCE.closeGate(),
                        new SequentialGroup(
                                new FollowPath(paths.Gate2_1),
                                new Delay(1.5),
                                new FollowPath(paths.Gate2_2),
                                new Delay(0.25),
                                Transfer.INSTANCE.openGate()
                        )
                ),

                new Delay(0.7),

                /// Wait 0.5 seconds and then collect G3 Artifacts and come back to launching position.

                new ParallelGroup(
                        Transfer.INSTANCE.closeGate(),
                        new SequentialGroup(
                                new FollowPath(paths.Gate3_1),
                                new Delay(1.5),
                                new FollowPath(paths.Gate3_2),
                                new Delay(0.25),
                                Transfer.INSTANCE.openGate()
                        )
                ),

                new Delay(0.7),

                /// Wait 0.5 seconds and then collect R1 Artifacts and come back to launching position.

                new ParallelGroup(
                        Transfer.INSTANCE.closeGate(),
                        new SequentialGroup(
                                new FollowPath(paths.R1),
                                new Delay(0.25),
                                Transfer.INSTANCE.openGate()
                        )
                )
        ).schedule();
    }

    @Override
    public void onUpdate() {
        LOOP_TIMER.reset();

        boolean IS_FOLLOWER_BUSY = PedroComponent.follower().isBusy();
        if (IS_FOLLOWER_BUSY != WAS_FOLLOWER_BUSY) {
            if (IS_FOLLOWER_BUSY) {
                Light.INSTANCE.setColor(Light.YELLOW, Light.Target.ROBOT).schedule();
            } else {
                Light.INSTANCE.setColor(Light.RED, Light.Target.ROBOT).schedule();
            }
            WAS_FOLLOWER_BUSY = IS_FOLLOWER_BUSY;
        }

        Configuration.CURRENT_POSE = PedroComponent.follower().getPose();

        X_VELOCITY = PedroComponent.follower().getVelocity().getXComponent();
        Y_VELOCITY = PedroComponent.follower().getVelocity().getYComponent();

        Shooter.INSTANCE.updateKinematics(
                Shooter.INSTANCE.GOAL_DISTANCE,
                Math.toRadians(Shooter.INSTANCE.HOOD_ANGLE)
        );

        Shooter.INSTANCE.TARGET_RPM = Shooter.INSTANCE.KINEMATIC_RPM_GOAL * Configuration.RPM_MULTIPLER;

        TRUE_TARGET_DEGREE = Turret.INSTANCE.TRUE_TARGET_DEGREE;
        double weight;

        if (!Double.isNaN(Shooter.INSTANCE.getTof())) {
            weight = (Shooter.INSTANCE.getTof() + Configuration.ARTIFACT_TRANSFER_TIME) - 0.9;
        } else {
            weight = 0.3;
        }

        Configuration.setAimPointOffset(-X_VELOCITY * weight, -Y_VELOCITY * weight);

        double vyr = ((Y_VELOCITY * 0.0254) * Math.sin(Math.PI / 2 - TRUE_TARGET_DEGREE))
                + ((X_VELOCITY * 0.0254) * Math.sin(TRUE_TARGET_DEGREE));
        double vxr = -((Y_VELOCITY * 0.0254) * Math.cos(Math.PI / 2 - TRUE_TARGET_DEGREE))
                + ((X_VELOCITY * 0.0254) * Math.cos(TRUE_TARGET_DEGREE));

        double additionalCompensation = 0;

        if (Y_VELOCITY < 0 || X_VELOCITY < 0) {
            additionalCompensation = -0.5;
        }

        double vn = Shooter.INSTANCE.shooterVKinematic() + (vyr * (Configuration.VELOCITY_COMPENSATION_WEIGHT + additionalCompensation));
        vt = Math.sqrt((vn * vn) + (vxr * vxr));

        Shooter.INSTANCE.updateKinematics(
                Shooter.INSTANCE.GOAL_DISTANCE,
                Math.toRadians(Shooter.INSTANCE.HOOD_ANGLE)
        );

        Configuration.TURRET_OFFSET = 1;

        Shooter.INSTANCE.TARGET_RPM = (Shooter.artifactVelocityMStoRPM(vt) * Configuration.RPM_MULTIPLER);

        telemetry.addLine("=== Position ===");
        telemetry.addData("X", PedroComponent.follower().getPose().getX());
        telemetry.addData("Y", PedroComponent.follower().getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));

        telemetry.update();

        LOOP_TIME = LOOP_TIMER.milliseconds();
    }

    @Override
    public void onStop() {
        Transfer.INSTANCE.emergencyStopAll().schedule();

        telemetry.setAutoClear(false);
        telemetry.addLine("=== Final Position ===");
        telemetry.addData("Final X", PedroComponent.follower().getPose().getX());
        telemetry.addData("Final Y", PedroComponent.follower().getPose().getY());
        telemetry.addData("Final Heading", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));
        telemetry.update();
    }

    public static class Paths {
        public PathChain PreloadLaunch;
        public PathChain R2;
        public PathChain Gate1_1;
        public PathChain Gate1_2;
        public PathChain Gate1_3;
        public PathChain Gate2_1;
        public PathChain Gate2_2;
        public PathChain Gate3_1;
        public PathChain Gate3_2;
        public PathChain R1;
        public PathChain R3;

        public Paths(Follower follower) {
            PreloadLaunch = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(115.429, 126.629),
                            new Pose(81.501, 76.116)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(44.5), Math.toRadians(-45))
                    .build();

            Gate1_1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(81.501, 76.116),
                            new Pose(105.784, 55.717),
                            new Pose(131.000, 58.598)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(28.159999999999997))
                    .setTValueConstraint(0.95)
                    .setVelocityConstraint(3)
                    .setTranslationalConstraint(0.5)
                    .setHeadingConstraint(Math.toRadians(1.5))
                    .setTimeoutConstraint(1500)
                    .build();

            Gate1_3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(131.000, 58.598),
                            new Pose(105.784, 55.717),
                            new Pose(81.501, 76.116)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(28.159999999999997), Math.toRadians(-45))
                    .build();

            Gate2_1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(81.501, 76.116),
                            new Pose(105.784, 55.717),
                            new Pose(131.000, 58.598)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(28.159999999999997))
                    .setTValueConstraint(0.95)
                    .setVelocityConstraint(3)
                    .setTranslationalConstraint(0.5)
                    .setHeadingConstraint(Math.toRadians(1.5))
                    .setTimeoutConstraint(1500)
                    .build();

            Gate2_2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(131.000, 58.598),
                            new Pose(105.784, 55.717),
                            new Pose(81.501, 76.116)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(28.159999999999997), Math.toRadians(-45))
                    .build();

            Gate3_1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(81.501, 76.116),
                            new Pose(105.784, 55.717),
                            new Pose(131.000, 58.598)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(28.159999999999997))
                    .setTValueConstraint(0.95)
                    .setVelocityConstraint(3)
                    .setTranslationalConstraint(0.5)
                    .setHeadingConstraint(Math.toRadians(1.5))
                    .setTimeoutConstraint(1500)
                    .build();

            Gate3_2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(131.000, 58.598),
                            new Pose(105.784, 55.717),
                            new Pose(81.501, 76.116)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(28.159999999999997), Math.toRadians(15))
                    .build();

            R1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(81.501, 76.116),
                            new Pose(100.421, 82.618),
                            new Pose(119.769, 82.263)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(15), Math.toRadians(0))
                    .addPath(new BezierLine(
                            new Pose(119.769, 82.263),
                            new Pose(82.327, 100.765)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            R2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(81.501, 76.116),
                            new Pose(96.070, 57.499),
                            new Pose(122.300, 58.900)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                    .addPath(new BezierCurve(
                            new Pose(122.300, 58.900),
                            new Pose(96.070, 57.499),
                            new Pose(81.501, 76.116)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                    .build();
        }
    }
}
