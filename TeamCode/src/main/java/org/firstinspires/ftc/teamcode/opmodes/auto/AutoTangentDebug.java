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

@Autonomous(name = "RT", group = "Red Alliance")
public class AutoTangentDebug extends NextFTCOpMode {
    double LOOP_TIME = 0;
    ElapsedTime LOOP_TIMER = new ElapsedTime();

    private Paths paths;

    // Starting Position
    private static final double START_X = 119.273;
    private static final double START_Y = 130.327;
    private static final double START_HEADING = 36.5;

    private static double RPM_SCALE_FACTOR = 2.75;

    public AutoTangentDebug() {
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
        telemetry.addData("Order:", "Preload, R2, Gate x3, R1, R3, Move");
        telemetry.addData("Gate:", "True");
        telemetry.addData("Solo:", "True");
        telemetry.addData("Total Count:", "18");

        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {

        new SequentialGroup(
                // Launch Preload
                new ParallelGroup(
                        Transfer.INSTANCE.intake(),
                        new FollowPath(paths.LaunchPreload)
                ),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.closeGate(),

                // R2 Pickup and Launch
                new FollowPath(paths.R2),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.closeGate(),

                // Gate cycle 1
                new FollowPath(paths.GateOpen1),
                new Delay(1.5),
                new FollowPath(paths.GateLaunch1),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.closeGate()
        ).schedule();
    }

    @Override
    public void onUpdate() {
        LOOP_TIMER.reset();

        Configuration.CURRENT_POSE = PedroComponent.follower().getPose();

        Shooter.INSTANCE.updateKinematics(
                Shooter.INSTANCE.GOAL_DISTANCE,
                Math.toRadians(Shooter.INSTANCE.HOOD_ANGLE)
        );

        Shooter.INSTANCE.TARGET_RPM = Shooter.INSTANCE.KINEMATIC_RPM_GOAL * Configuration.RPM_MULTIPLER;

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
        public PathChain LaunchPreload;
        public PathChain R2;
        public PathChain GateOpen1;
        public PathChain GateLaunch1;

        public Paths(Follower follower) {
            LaunchPreload = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(119.273, 130.327),
                            new Pose(84.673, 83.909)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(-62))
                    .build();

            R2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(84.673, 83.909),
                            new Pose(97.782, 59.382),
                            new Pose(128.636, 59.255)
                    ))
                    .setTangentHeadingInterpolation()
                    .addPath(new BezierCurve(
                            new Pose(128.636, 59.255),
                            new Pose(97.782, 59.382),
                            new Pose(84.636, 83.836)
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            GateOpen1 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(84.636, 83.836),
                            new Pose(99.234, 56.711),
                            new Pose(126.652, 58.711),
                            new Pose(129.868, 60.277)
                    )
            ).setTangentHeadingInterpolation()
                    .build();

            GateLaunch1 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(129.868, 60.277),
                            new Pose(119.452, 54.820),
                            new Pose(93.780, 67.584),
                            new Pose(84.636, 83.836)
                    )
            ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
        }
    }
}
