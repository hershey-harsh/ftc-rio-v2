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

@Autonomous(name = "BT2GS18 - R1 & R2", group = "Blue Alliance")
public class BT2GA18 extends NextFTCOpMode {
    double LOOP_TIME = 0;
    ElapsedTime LOOP_TIMER = new ElapsedTime();

    private Paths paths;

    // Starting Position
    private static final double START_X = 32.248;
    private static final double START_Y = 134.708;
    private static final double START_HEADING = 180;
    private double vt = 0;

    private double X_VELOCITY = 0;
    private double Y_VELOCITY = 0;
    public double TRUE_TARGET_DEGREE = 0;

    private static double RPM_SCALE_FACTOR = 2.75;

    public BT2GA18() {
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
        Configuration.SHOOTER_HEIGHT_TO_GOAL = 0.95;

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
        telemetry.addData("Order:", "Preload, R2, Gate x3, R1, R3, Move");
        telemetry.addData("Gate:", "True");
        telemetry.addData("Solo:", "True");
        telemetry.addData("Total Count:", "18");
        telemetry.addData("Shooter Height:", Configuration.SHOOTER_HEIGHT_TO_GOAL);

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
                            new Pose(33.101, 135.821),
                            new Pose(55.882, 83.840)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(138))
                    .build();

            R2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(55.882, 83.840),
                                    new Pose(48.250, 53.703),
                                    new Pose(47.266, 60.651),
                                    new Pose(40.234, 60.098),
                                    new Pose(10.613, 59.057)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .addPath(
                            new BezierCurve(
                                    new Pose(10.613, 59.057),
                                    new Pose(42.286, 61.227),
                                    new Pose(57.168, 83.992)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(136))
                    .build();

            GateOpenCollect1 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(57.168, 83.992),
                            new Pose(51.512, 57.326),
                            new Pose(10.035-1.2, 62.100)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(136), Math.toRadians(152))
                    .build();

            GateLaunch1 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(10.035-1.2, 62.100),
                            new Pose(42.286, 61.227),
                            new Pose(56.168, 83.992)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(152), Math.toRadians(136))
                    .build();

            GateOpenCollect2 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(56.168, 83.992),
                            new Pose(51.512, 57.326),
                            new Pose(10.035-1.2, 62.100)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(136), Math.toRadians(152))
                    .build();

            GateLaunch2 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(10.035-1.2, 62.100),
                            new Pose(42.286, 61.227),
                            new Pose(56.168, 83.992)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(152), Math.toRadians(136))
                    .build();

            GateOpenCollect3 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(56.168, 83.992),
                            new Pose(51.512, 57.326),
                            new Pose(10.035-1.2, 62.100)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(136), Math.toRadians(152))
                    .build();

            GateLaunch3 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(10.035-1.2, 62.100),
                            new Pose(42.286, 61.227),
                            new Pose(55.412, 83.992)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(152), Math.toRadians(180))
                    .build();

            R1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(55.412, 83.992),
                                    new Pose(23.763, 83.723)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(23.763, 83.723),
                                    new Pose(58.672, 102.393)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                    .build();
        }
    }
}
