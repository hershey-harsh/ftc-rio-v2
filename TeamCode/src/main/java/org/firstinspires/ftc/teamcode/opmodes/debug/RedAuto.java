package org.firstinspires.ftc.teamcode.opmodes.debug;

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

@Autonomous(name = "Red Auto", group = "Red Alliance")
public class RedAuto extends NextFTCOpMode {
    private static double X_VELOCITY = 0;
    private static double Y_VELOCITY = 0;

    double LOOP_TIME = 0;
    ElapsedTime LOOP_TIMER = new ElapsedTime();

    private Paths paths;

    // Starting Position
    private static final double START_X = 111.418;
    private static final double START_Y = 135.709;
    private static final double START_HEADING = 0;

    private static double RPM_SCALE_FACTOR = 2.75;

    public RedAuto() {
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

        // Start intake and other subsystems early so they're ready
        Turret.INSTANCE.start().schedule();
        Turret.INSTANCE.changeToAuto().schedule();
        Shooter.INSTANCE.on().schedule();

        telemetry.addData("Alliance:", "Red");
        telemetry.addData("Side:", "Top");
        telemetry.addData("Order:", "Preload, R1, Launch, R2, Launch, Gate, Launch, Gate, Launch, R3, Launch, Move");
        telemetry.addData("Gate:", "True");
        telemetry.addData("Solo:", "True");
        telemetry.addData("Total Count:", "18");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        Transfer.INSTANCE.intake().schedule();

        new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(paths.R1Pickup),
                        new SequentialGroup(
                                new Delay(1.4),
                                Transfer.INSTANCE.openGate(),
                                new Delay(0.8),
                                Transfer.INSTANCE.closeGate()
                        )
                ),

                new FollowPath(paths.R1Launch),
                new Delay(0.2),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.closeGate(),

                // R2 Pickup and Launch
                new FollowPath(paths.R2Pickup),

                new FollowPath(paths.R2Launch),
                new Delay(0.2),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.closeGate(),

                // Gate Pickup 1
                new FollowPath(paths.GatePickup1),
                new Delay(2),
                new FollowPath(paths.GateLaunch1),
                new Delay(0.2),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.closeGate(),

                // Gate Pickup 2
                new FollowPath(paths.GatePickup2),
                new Delay(2),
                new FollowPath(paths.GateLaunch2),
                new Delay(0.2),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.closeGate(),

                // R3 Pickup and Launch
                new FollowPath(paths.R3Pickup),

                new FollowPath(paths.R3Launch),
                new Delay(0.2),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.closeGate(),

                // Move (End)
                new FollowPath(paths.Move)
        ).schedule();
    }

    @Override
    public void onUpdate() {
        LOOP_TIMER.reset();

        telemetry.addData("Loop Time (ms)", LOOP_TIME);
        telemetry.addData("Loop Time (hz)", (1000 / LOOP_TIME));

        PedroComponent.follower().update();

        Configuration.CURRENT_POSE = PedroComponent.follower().getPose();

        X_VELOCITY = PedroComponent.follower().getVelocity().getXComponent();
        Y_VELOCITY = PedroComponent.follower().getVelocity().getYComponent();

        double headingDeg = Math.toDegrees(PedroComponent.follower().getPose().getHeading());

        X_VELOCITY = PedroComponent.follower().getVelocity().getXComponent();
        Y_VELOCITY = PedroComponent.follower().getVelocity().getYComponent();

        Shooter.INSTANCE.updateKinematics(
                Shooter.INSTANCE.GOAL_DISTANCE,
                Math.toRadians(Shooter.INSTANCE.HOOD_ANGLE)
        );

        Shooter.INSTANCE.TARGET_RPM = Shooter.INSTANCE.kinematicRPMGoal * Configuration.RPM_MULTIPLER;

        double weight;

        if (!Double.isNaN(Shooter.INSTANCE.getTof())) {
            weight = Shooter.INSTANCE.getTof() + Configuration.ARTIFACT_TRANSFER_TIME;
        } else {
            weight = 0.3;
        }

        Configuration.setAimPointOffset(-X_VELOCITY * weight, -Y_VELOCITY * weight);

        double vyr = ((Y_VELOCITY * 0.0254) * Math.sin(Math.PI / 2 - Turret.INSTANCE.TRUE_TARGET_DEGREE))
                + ((X_VELOCITY * 0.0254) * Math.sin(Turret.INSTANCE.TRUE_TARGET_DEGREE));
        double vxr = -((Y_VELOCITY * 0.0254) * Math.cos(Math.PI / 2 - Turret.INSTANCE.TRUE_TARGET_DEGREE))
                + ((X_VELOCITY * 0.0254) * Math.cos(Turret.INSTANCE.TRUE_TARGET_DEGREE));

        double additionalCompensation = 0;

        if (Y_VELOCITY < 0 || X_VELOCITY < 0) {
            additionalCompensation = -0.75;
        }

        double vn = Shooter.INSTANCE.shooterVKinematic() + (vyr * (Configuration.VELOCITY_COMPENSATION_WEIGHT + additionalCompensation));
        double vt = Math.sqrt((vn * vn) + (vxr * vxr));

//            double oo = Math.atan((-vxr) / vn);
//
        Configuration.TURRET_OFFSET = 2;
        Shooter.INSTANCE.updateKinematics(
                Shooter.INSTANCE.GOAL_DISTANCE,
                Math.toRadians(Shooter.INSTANCE.HOOD_ANGLE)
        );

        Shooter.INSTANCE.TARGET_RPM = Shooter.artifactVelocityMStoRPM(vt) * Configuration.RPM_MULTIPLER;

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
        public PathChain R1Pickup;
        public PathChain R1Launch;
        public PathChain R2Pickup;
        public PathChain R2Launch;
        public PathChain GatePickup1;
        public PathChain GateLaunch1;
        public PathChain GatePickup2;
        public PathChain GateLaunch2;
        public PathChain R3Pickup;
        public PathChain R3Launch;
        public PathChain Move;

        public Paths(Follower follower) {
            R1Pickup = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(111.418, 135.709),
                            new Pose(75.086, 83.436),
                            new Pose(65.777, 79.764),
                            new Pose(98.759, 84.582),
                            new Pose(131.264, 83.600)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            R1Launch = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(131.264, 83.600),
                            new Pose(84.636, 73.218)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            R2Pickup = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(84.636, 73.218),
                            new Pose(100.060, 57.696),
                            new Pose(95.893, 59.916),
                            new Pose(94.039, 59.461),
                            new Pose(131.409, 59.482)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            R2Launch = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(131.409, 59.482),
                            new Pose(84.636, 73.218)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            GatePickup1 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(84.636, 73.218),
                            new Pose(92.488, 57.326),
                            new Pose(132.323, 62.604)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(24.34))
                    .build();

            GateLaunch1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(132.323, 62.604),
                            new Pose(84.636, 73.218)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(24.34), Math.toRadians(0))
                    .build();

            GatePickup2 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(84.636, 73.218),
                            new Pose(92.488, 57.326),
                            new Pose(132.323, 62.604)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(24.34))
                    .build();

            GateLaunch2 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(132.323, 62.604),
                            new Pose(84.636, 73.218)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(24.34), Math.toRadians(0))
                    .build();

            R3Pickup = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(84.636, 73.218),
                            new Pose(85.147, 29.433),
                            new Pose(82.835, 34.965),
                            new Pose(78.208, 35.963),
                            new Pose(131.710, 35.185)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            R3Launch = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(131.710, 35.185),
                            new Pose(84.636, 73.218)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Move = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(84.636, 73.218),
                            new Pose(97.600, 73.218)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        }
    }
}
