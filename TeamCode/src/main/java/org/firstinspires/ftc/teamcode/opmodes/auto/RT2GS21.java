package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.firstinspires.ftc.teamcode.subsystems.AimController;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Disabled
@Autonomous(name = "RT2GS21 - All", group = "Red Alliance")
public class RT2GS21 extends NextFTCOpMode {
    double LOOP_TIME = 0;
    ElapsedTime LOOP_TIMER = new ElapsedTime();

    private Paths paths;

    // Starting Position (mirror of BT2GS21: x' = 141.5 - x, heading' = 180 - heading)
    private static final double START_X = 115.429;
    private static final double START_Y = 126.629;
    private static final double START_HEADING = 44;
    private double vt = 0;

    private double X_VELOCITY = 0;
    private double Y_VELOCITY = 0;
    public double TRUE_TARGET_DEGREE = 0;

    public RT2GS21() {
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
        Configuration.ALLIANCE = Configuration.Alliance.RED;
        Configuration.SHOOTER_HEIGHT_TO_GOAL = 1.1;

        paths = new Paths(PedroComponent.follower());

        Pose startPose = new Pose(START_X, START_Y, Math.toRadians(START_HEADING));
        PedroComponent.follower().setStartingPose(startPose);
        Configuration.CURRENT_POSE = startPose;

        // Start intake and other subsystems early
        Transfer.INSTANCE.start().schedule();
        Turret.INSTANCE.start().schedule();
        Shooter.INSTANCE.start().schedule();

        telemetry.addData("Alliance:", "Red");
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

                new FollowPath(paths.Preload),
                Transfer.INSTANCE.openGate(),
                new Delay(0.5),

                /// Wait 0.5 seconds and then collect R2 Artifacts and come back to launching position.

                new ParallelGroup(
                        Transfer.INSTANCE.closeGate(),
                        new FollowPath(paths.R2),
                        new SequentialGroup(
                                new Delay(2.8), //TODO: Measure how long it takes from Preload -> R2 Collect then + 1.2
                                Transfer.INSTANCE.openGate()
                        )
                ),

                new Delay(0.25),

                /// Wait 0.5 seconds and then collect G1 Artifacts and come back to launching position.

                new ParallelGroup(
                        Transfer.INSTANCE.closeGate(),
                        new SequentialGroup(
                                //new InstantCommand(() -> Configuration.RPM_MULTIPLER = 2),
                                new FollowPath(paths.Gate1Collect),
                                new Delay(1.25),
                                new ParallelGroup(
                                        new FollowPath(paths.Gate1Launch),
                                        new SequentialGroup(
                                                new Delay(1.8), //TODO: Measure how long it takes from R2 Launch -> G1 Collect then + 1.5
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
                                new FollowPath(paths.Gate2Collect),
                                new Delay(1.4),
                                new ParallelGroup(
                                        new FollowPath(paths.Gate2Launch),
                                        new SequentialGroup(
                                                new Delay(1.8), //TODO: Measure how long it takes from R2 Launch -> G1 Collect then + 1.5
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
                                new FollowPath(paths.Gate3Collect),
                                new Delay(1.4),
                                new ParallelGroup(
                                        new FollowPath(paths.Gate3Launch),
                                        new SequentialGroup(
                                                new Delay(1.8), //TODO: Measure how long it takes from R2 Launch -> G1 Collect then + 1.5
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

                new Delay(0.7),

                /// Wait 0.5 seconds and then collect R3 Artifacts and come back to launching + park position.

                new ParallelGroup(
                        Transfer.INSTANCE.closeGate(),
                        new FollowPath(paths.R3),
                        new SequentialGroup(
                                new Delay(3.5), //TODO: Measure how long it takes from R3 Collect -> R3 Launch then + 1.85
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

        // Shoot-on-the-move via the shared predictive controller (also applies droop comp).
        AimController.updateShootOnMove(X_VELOCITY, Y_VELOCITY);
        Configuration.TURRET_OFFSET = 1;      // mirror of BT2GS21 (blue -1 -> red +1)
        Shooter.INSTANCE.TARGET_RPM *= 1.1;   // GS21 firing-rate RPM bump (preserved)

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
        public PathChain R1;
        public PathChain R2;
        public PathChain Gate1Collect;
        public PathChain Gate1Launch;
        public PathChain Gate2Collect;
        public PathChain Gate2Launch;
        public PathChain Gate3Collect;
        public PathChain Gate3Launch;
        public PathChain R3;
        public PathChain Preload;

        public Paths(Follower follower) {
            R1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(81.501, 76.116),
                            new Pose(91.070, 82.846),
                            new Pose(119.500, 83.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(32), Math.toRadians(0))
                    .addPath(new BezierCurve(
                            new Pose(119.500, 83.000),
                            new Pose(91.070, 82.846),
                            new Pose(81.501, 76.116)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(32))
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

            Gate1Collect = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(81.501, 76.116),
                            new Pose(105.784, 55.717),
                            new Pose(131.000, 58.598)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(28.16))
                    .build();

            Gate1Launch = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(131.000, 58.598),
                            new Pose(105.784, 55.717),
                            new Pose(81.501, 76.116)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(28.16), Math.toRadians(-45))
                    .build();

            Gate2Collect = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(81.501, 76.116),
                            new Pose(105.784, 55.717),
                            new Pose(131.000, 58.598)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(28.16))
                    .build();

            Gate2Launch = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(131.000, 58.598),
                            new Pose(105.784, 55.717),
                            new Pose(81.501, 76.116)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(28.16), Math.toRadians(-45))
                    .build();

            Gate3Collect = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(81.501, 76.116),
                            new Pose(105.784, 55.717),
                            new Pose(131.000, 58.598)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(28.16))
                    .build();

            Gate3Launch = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(131.000, 58.598),
                            new Pose(105.784, 55.717),
                            new Pose(81.501, 76.116)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(28.16), Math.toRadians(-45))
                    .build();

            R3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(81.501, 76.116),
                            new Pose(82.070, 36.431),
                            new Pose(119.500, 36.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                    .addPath(new BezierLine(
                            new Pose(119.500, 36.000),
                            new Pose(87.504, 110.562)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(300), Math.toRadians(300))
                    .build();

            Preload = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(115.429, 126.629),
                            new Pose(81.501, 76.116)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(44.5), Math.toRadians(-45))
                    .build();
        }
    }
}
