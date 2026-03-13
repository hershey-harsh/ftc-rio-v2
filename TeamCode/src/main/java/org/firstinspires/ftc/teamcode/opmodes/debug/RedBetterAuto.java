package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name = "Red Better Auto", group = "Red Alliance")
public class RedBetterAuto extends NextFTCOpMode {
    private static double X_VELOCITY = 0;
    private static double Y_VELOCITY = 0;

    double LOOP_TIME = 0;
    ElapsedTime LOOP_TIMER = new ElapsedTime();

    private Paths paths;

    // Starting Position
    private static final double START_X = 119.273;
    private static final double START_Y = 130.327;
    private static final double START_HEADING = 36.5;

    private static double RPM_SCALE_FACTOR = 2.75;

    public RedBetterAuto() {
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
        Transfer.INSTANCE.intake().schedule();
        Turret.INSTANCE.start().schedule();
        Turret.INSTANCE.changeToAuto().schedule();
        Shooter.INSTANCE.on().schedule();

        telemetry.addData("Alliance:", "Red");
        telemetry.addData("Side:", "Better");
        telemetry.addData("Order:", "Preload, R2, Launch, Gate x3, R1, Launch");
        telemetry.addData("Gate:", "True");
        telemetry.addData("Solo:", "True");
        telemetry.addData("Total Count:", "10");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {

        new SequentialGroup(
                // Launch Preload
                new FollowPath(paths.LaunchPreload),
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
                Transfer.INSTANCE.closeGate(),

                // Gate cycle 2
                new FollowPath(paths.GateOpen2),
                new Delay(2),
                new FollowPath(paths.GateLaunch2),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.closeGate(),

                // Gate cycle 3
                new FollowPath(paths.GateOpen3),
                new Delay(2),
                new FollowPath(paths.GateLaunch3),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.closeGate(),

                // R1 Pickup and Launch
                new FollowPath(paths.R1),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.closeGate()
        ).schedule();
    }

    @Override
    public void onUpdate() {
        LOOP_TIMER.reset();

        X_VELOCITY = PedroComponent.follower().getVelocity().getXComponent();
        Y_VELOCITY = PedroComponent.follower().getVelocity().getYComponent();

        double headingDeg = Math.toDegrees(PedroComponent.follower().getPose().getHeading());

        telemetry.addLine("=== Position ===");
        telemetry.addData("X", PedroComponent.follower().getPose().getX());
        telemetry.addData("Y", PedroComponent.follower().getPose().getY());
        telemetry.addData("Heading", headingDeg);

        telemetry.addLine();

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
        public PathChain GateOpen2;
        public PathChain GateLaunch2;
        public PathChain GateOpen3;
        public PathChain GateLaunch3;
        public PathChain R1;
        public PathChain R3;

        public Paths(Follower follower) {
            LaunchPreload = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(119.273, 130.327),
                            new Pose(84.673, 83.909)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(0))
                    .build();

            R2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(84.673, 83.909),
                            new Pose(101.709, 57.345),
                            new Pose(128.636, 59.255)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-52), Math.toRadians(0))
                    .addPath(new BezierCurve(
                            new Pose(128.636, 59.255),
                            new Pose(102.975, 56.666),
                            new Pose(84.636, 83.836)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-48))
                    .build();

            GateOpen1 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(84.636, 83.836),
                            new Pose(103.016, 59.184),
                            new Pose(130.741, 62.604)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(24.34))
                    .build();

            GateLaunch1 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(130.741, 62.604),
                            new Pose(102.489, 59.475),
                            new Pose(84.636, 83.836)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(24.3), Math.toRadians(-48))
                    .build();

            GateOpen2 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(84.636, 83.836),
                            new Pose(103.016, 59.277),
                            new Pose(130.741, 62.604)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(24.3))
                    .build();

            GateLaunch2 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(130.741, 62.604),
                            new Pose(102.489, 59.475),
                            new Pose(84.703, 83.830)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(24.3), Math.toRadians(-48))
                    .build();

            GateOpen3 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(84.703, 83.830),
                            new Pose(103.016, 59.184),
                            new Pose(130.741, 62.604)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(24.3))
                    .build();

            GateLaunch3 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(130.741, 62.604),
                            new Pose(102.489, 59.475),
                            new Pose(84.636, 83.836)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(24.3), Math.toRadians(-48))
                    .build();

            R1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(84.636, 83.836),
                            new Pose(124.273, 83.782)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(new BezierLine(
                            new Pose(124.273, 83.782),
                            new Pose(84.636, 83.836)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            R3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(84.636, 83.836),
                            new Pose(82.497, 33.715),
                            new Pose(129.909, 35.182)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                    .addPath(new BezierLine(
                            new Pose(129.909, 35.182),
                            new Pose(84.673, 84.055)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-50), Math.toRadians(-50))
                    .setReversed()
                    .build();
        }
    }
}
