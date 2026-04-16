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
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Autonomous(name = "BT2GS21 - All", group = "Blue Alliance")
public class BT2GS21 extends NextFTCOpMode {
    double LOOP_TIME = 0;
    ElapsedTime LOOP_TIMER = new ElapsedTime();

    private Paths paths;

    // Starting Position
    private static final double START_X = 26.700;
    private static final double START_Y = 128.200;
    private static final double START_HEADING = 135;
    private double vt = 0;

    private double X_VELOCITY = 0;
    private double Y_VELOCITY = 0;
    public double TRUE_TARGET_DEGREE = 0;

    public BT2GS21() {
        addComponents(
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Transfer.INSTANCE),
                new SubsystemComponent(Shooter.INSTANCE),
                new SubsystemComponent(Turret.INSTANCE)
        );
    }

    @Override
    public void onInit() {
        Configuration.ALLIANCE = Configuration.Alliance.BLUE;
        Configuration.SHOOTER_HEIGHT_TO_GOAL = 1.1;

        paths = new Paths(PedroComponent.follower());

        Pose startPose = new Pose(START_X, START_Y, Math.toRadians(START_HEADING));
        PedroComponent.follower().setStartingPose(startPose);
        Configuration.CURRENT_POSE = startPose;

        // Start intake and other subsystems early
        Transfer.INSTANCE.start().schedule();
        Turret.INSTANCE.start().schedule();
        Shooter.INSTANCE.start().schedule();

        telemetry.addData("Alliance:", "Blue");
        telemetry.addData("Side:", "Top");
        telemetry.addData("Order:", "Preload, R2, Gate x3, R1, R3");
        telemetry.addData("Gate:", "True");
        telemetry.addData("Solo:", "True");
        telemetry.addData("Total Count:", "21");

        telemetry.update();
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
                new Delay(0.5),

                /// Wait 0.5 seconds and then collect R2 Artifacts and come back to launching position.

                new ParallelGroup(
                        Transfer.INSTANCE.closeGate(),
                        new FollowPath(paths.R2),
                        new SequentialGroup(
                                new Delay(2.5), //TODO: Measure how long it takes from Preload -> R2 Collect then + 1.2
                                Transfer.INSTANCE.openGate()
                        )
                ),

                new Delay(0.25),

                /// Wait 0.5 seconds and then collect G1 Artifacts and come back to launching position.

                new ParallelGroup(
                        Transfer.INSTANCE.closeGate(),
                        new SequentialGroup(
                                new InstantCommand(() -> Configuration.RPM_MULTIPLER = 2),
                                new FollowPath(paths.Gate1_1),
//                                new Delay(0.25),
//                                new FollowPath(paths.Gate1_2),
                                new Delay(1.25),
                                new ParallelGroup(
                                        new FollowPath(paths.Gate1_3),
                                        new SequentialGroup(
                                                new Delay(1), //TODO: Measure how long it takes from R2 Launch -> G1 Collect then + 1.5
                                                Transfer.INSTANCE.openGate()
                                        )
                                )
                        )
                ),

                new Delay(0.25),

                /// Wait 0.5 seconds and then collect G2 Artifacts and come back to launching position.

                new ParallelGroup(
                        Transfer.INSTANCE.closeGate(),
                        new SequentialGroup(
                                new FollowPath(paths.Gate2_1),
                                new Delay(1.4),
                                new ParallelGroup(
                                        new FollowPath(paths.Gate2_2),
                                        new SequentialGroup(
                                                new Delay(1), //TODO: Measure how long it takes from R2 Launch -> G1 Collect then + 1.5
                                                Transfer.INSTANCE.openGate()
                                        )
                                )
                        )
                ),
                new Delay(0.25),

                /// Wait 0.5 seconds and then collect G3 Artifacts and come back to launching position.

                new ParallelGroup(
                        Transfer.INSTANCE.closeGate(),
                        new SequentialGroup(
                                new FollowPath(paths.Gate3_1),
                                new Delay(1.4),
                                new ParallelGroup(
                                        new FollowPath(paths.Gate3_2),
                                        new SequentialGroup(
                                                new Delay(1), //TODO: Measure how long it takes from R2 Launch -> G1 Collect then + 1.5
                                                Transfer.INSTANCE.openGate()
                                        )
                                )
                        )
                ),
                new Delay(0.25),

                /// Wait 0.5 seconds and then collect R1 Artifacts and come back to launching position.

                new ParallelGroup(
                        Transfer.INSTANCE.closeGate(),
                        new FollowPath(paths.R1),
                        new SequentialGroup(
                                new Delay(2.75), //TODO: Measure how long it takes from G3 Launch -> R1 Collect then + 0.95
                                Transfer.INSTANCE.openGate()
                        )
                ),

                new Delay(0.25),

                /// Wait 0.5 seconds and then collect R3 Artifacts and come back to launching + park position.

                new ParallelGroup(
//                        new InstantCommand(() -> Configuration.RPM_MULTIPLER = Configuration.RPM_MULTIPLER + 0.1),
                        Transfer.INSTANCE.closeGate(),
                        new FollowPath(paths.R3),
                        new SequentialGroup(
                                new Delay(3.5), //TODO: Measure how long it takes from Preload -> R2 Collect then + 1.85
                                Transfer.INSTANCE.openGate()
                        )
                )
        ).schedule();
    }

    @Override
    public void onUpdate() {
        LOOP_TIMER.reset();

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

        Configuration.TURRET_OFFSET = -1;

        Shooter.INSTANCE.TARGET_RPM = (Shooter.artifactVelocityMStoRPM(vt) * Configuration.RPM_MULTIPLER) * 1.1;

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
            PreloadLaunch = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(26.700, 128.200),
                            new Pose(61.000, 72.400)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(200))
                    .build();

            R2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(61.000, 72.400),
                            new Pose(54.000, 59.500),
                            new Pose(21.000, 60.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(200), Math.toRadians(180))
                    .addPath(new BezierLine(
                            new Pose(21.000, 60.000),
                            new Pose(59.000, 77.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(195))
                    .build();

            Gate1_1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(59.000, 77.000),
                            new Pose(40.000, 62.000),
                            new Pose(9, 59.065)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(195), Math.toRadians(150.81))
                    .setTValueConstraint(0.95)
                    .setVelocityConstraint(3)
                    .setTranslationalConstraint(0.2)
                    .setHeadingConstraint(Math.toRadians(1.5))
                    .setTimeoutConstraint(1000)
                    .build();

//            Gate1_2 = follower.pathBuilder()
//                    .addPath(new BezierCurve(
//                            new Pose(10, 60.954),
//                            new Pose(17.000, 58.000),
//                            new Pose(12.000, 56.000)
//                    ))
//                    .setLinearHeadingInterpolation(Math.toRadians(151.89), Math.toRadians(125))
//                    .build();

            Gate1_3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(9, 59.065),
                            new Pose(18.300, 60.316),
                            new Pose(21.089, 62.013),
                            new Pose(59.000, 77.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(150.81), Math.toRadians(195))
                    .build();

            Gate2_1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(59.000, 77.000),
                            new Pose(40.000, 62.000),
                            new Pose(9, 60.065)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(195), Math.toRadians(150.81))
                    .setTValueConstraint(0.95)
                    .setVelocityConstraint(3)
                    .setTranslationalConstraint(0.2)
                    .setHeadingConstraint(Math.toRadians(1.5))
                    .setTimeoutConstraint(1000)
                    .build();

            Gate2_2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(9, 60.065),
                            new Pose(18.300, 60.316),
                            new Pose(21.089, 62.013),
                            new Pose(59.000, 77.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(150.81), Math.toRadians(195))
                    .build();

            Gate3_1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(59.000, 77.000),
                            new Pose(40.000, 62.000),
                            new Pose(9, 60.065)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(195), Math.toRadians(150.81))
                    .setTValueConstraint(0.95)
                    .setVelocityConstraint(3)
                    .setTranslationalConstraint(0.2)
                    .setHeadingConstraint(Math.toRadians(1.5))
                    .setTimeoutConstraint(1000)
                    .build();

            Gate3_2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(9, 60.065),
                            new Pose(18.300, 60.316),
                            new Pose(21.089, 62.013),
                            new Pose(59.000, 77.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(150.81), Math.toRadians(195))
                    .build();

            R1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(59.000, 77.000),
                            new Pose(30, 83)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(195), Math.toRadians(180))
                    .addPath(new BezierLine(
                            new Pose(30, 83),
                            new Pose(56, 82)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(245), Math.toRadians(245))
                    .build();

            R3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(56.000, 82.000),
                            new Pose(57.000, 31.000),
                            new Pose(17.000, 36.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(245), Math.toRadians(180))
                    .addPath(new BezierLine(
                            new Pose(17.000, 36.000),
                            new Pose(57.409, 102.145)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(235))
                    .build();
        }
    }
}
