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

@Autonomous(name = "RT2GS18 - R1 & R2", group = "Red Alliance")
public class RT2GA18 extends NextFTCOpMode {
    double LOOP_TIME = 0;
    ElapsedTime LOOP_TIMER = new ElapsedTime();

    private Paths paths;

    // Starting Position
    private static final double START_X = 110.899;
    private static final double START_Y = 135.821;
    private static final double START_HEADING = 0;
    private double vt = 0;

    private double X_VELOCITY = 0;
    private double Y_VELOCITY = 0;
    public double TRUE_TARGET_DEGREE = 0;

    private static double RPM_SCALE_FACTOR = 2.75;

    public RT2GA18() {
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
        Configuration.SHOOTER_HEIGHT_TO_GOAL = 0.95;

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

        Shooter.INSTANCE.on().schedule();

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
                new FollowPath(paths.GateOpenCollect1),
                new Delay(1.5),
                new FollowPath(paths.GateLaunch1),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.closeGate(),

                // Gate cycle 2
                new FollowPath(paths.GateOpenCollect2),
                new Delay(1.5),
                new FollowPath(paths.GateLaunch2),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.closeGate(),

                // Gate cycle 3
                new FollowPath(paths.GateOpenCollect3),
                new Delay(1.5),
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

        Configuration.CURRENT_POSE = PedroComponent.follower().getPose();

        if (Configuration.CURRENT_POSE.getY() < 36) {
            Shooter.INSTANCE.TARGET_RPM = 4200;
            Configuration.TURRET_OFFSET = -2;
        } else {
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
                weight = Shooter.INSTANCE.getTof() + Configuration.ARTIFACT_TRANSFER_TIME;
            } else {
                weight = 0.3;
            }

            double additionalCompensation = 0;
            double weightCompensation = 0;

            if (Y_VELOCITY < 0 || X_VELOCITY < 0) {
                additionalCompensation = -0.35;
                weightCompensation = -0.15;
            }

            Configuration.setAimPointOffset(-X_VELOCITY * (weight + weightCompensation), -Y_VELOCITY * (weight + weightCompensation));

            double vyr = ((Y_VELOCITY * 0.0254) * Math.sin(Math.PI / 2 - TRUE_TARGET_DEGREE))
                    + ((X_VELOCITY * 0.0254) * Math.sin(TRUE_TARGET_DEGREE));
            double vxr = -((Y_VELOCITY * 0.0254) * Math.cos(Math.PI / 2 - TRUE_TARGET_DEGREE))
                    + ((X_VELOCITY * 0.0254) * Math.cos(TRUE_TARGET_DEGREE));

            double vn = Shooter.INSTANCE.shooterVKinematic() + (vyr * (Configuration.VELOCITY_COMPENSATION_WEIGHT + additionalCompensation));
            vt = Math.sqrt((vn * vn) + (vxr * vxr));

            Configuration.TURRET_OFFSET = -2;
            Shooter.INSTANCE.updateKinematics(
                    Shooter.INSTANCE.GOAL_DISTANCE,
                    Math.toRadians(Shooter.INSTANCE.HOOD_ANGLE)
            );

            Shooter.INSTANCE.TARGET_RPM = Shooter.artifactVelocityMStoRPM(vt) * Configuration.RPM_MULTIPLER;
        }

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
        public PathChain GateOpenCollect1;
        public PathChain GateLaunch1;
        public PathChain GateOpenCollect2;
        public PathChain GateLaunch2;
        public PathChain GateOpenCollect3;
        public PathChain GateLaunch3;
        public PathChain R1;

        public Paths(Follower follower) {
            LaunchPreload = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(110.899, 135.821),
                            new Pose(88.118, 83.840)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(42))
                    .build();

            R2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(88.118, 83.840),
                                    new Pose(95.750, 53.703), new Pose(96.734, 60.651), new Pose(103.766, 60.098),
                                    new Pose(133.387, 59.057)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .addPath(
                            new BezierCurve(
                                    new Pose(133.387, 59.057),
                                    new Pose(101.714, 61.227),
                                    new Pose(86.832, 83.992)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(42))
                    .build();

            GateOpenCollect1 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(86.832, 83.992),
                            new Pose(92.488, 57.326),
                            new Pose(133.965, 62.1)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(30))
                    .build();

            GateLaunch1 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(133.965, 62.1),
                            new Pose(101.714, 61.227),
                            new Pose(87.832, 83.992)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(42))
                    .build();

            GateOpenCollect2 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(87.832, 83.992),
                            new Pose(92.488, 57.326),
                            new Pose(133.965, 62.1)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(30))
                    .build();

            GateLaunch2 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(133.965, 62.1),
                            new Pose(101.714, 61.227),
                            new Pose(87.832, 83.992)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(42))
                    .build();

            GateOpenCollect3 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(87.832, 83.992),
                            new Pose(92.488, 57.326),
                            new Pose(133.965, 62.1)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(30))
                    .build();

            GateLaunch3 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(133.965, 62.1),
                            new Pose(101.714, 61.227),
                            new Pose(88.588, 83.992)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
                    .build();

            R1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(88.588, 83.992),
                                    new Pose(120.23712757830404, 83.72268907563024)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(120.23712757830404, 83.72268907563024),
                                    new Pose(85.32818945760118, 102.39343009931243)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                    .build();
        }
    }
}
