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
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "BT1GS18", group = "Blue Alliance")
public class BT1GS18 extends NextFTCOpMode {
    private static double X_VELOCITY = 0;
    private static double Y_VELOCITY = 0;

    double LOOP_TIME = 0;
    ElapsedTime LOOP_TIMER = new ElapsedTime();

    private Paths paths;

    private static final double START_X = 32.582;
    private static final double START_Y = 135.709;
    private static final double START_HEADING = 180;

    private static double RPM_SCALE_FACTOR = 2.75;

    public BT1GS18() {
        addComponents(
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Transfer.INSTANCE),
                new SubsystemComponent(Shooter.INSTANCE),
                new SubsystemComponent(Turret.INSTANCE)
        );}

    @Override
    public void onInit() {
        Configuration.ALLIANCE = Configuration.Alliance.BLUE;

        paths = new Paths(PedroComponent.follower());

        Pose startPose = new Pose(START_X, START_Y, Math.toRadians(START_HEADING));
        PedroComponent.follower().setStartingPose(startPose);
        Configuration.CURRENT_POSE = startPose;

        telemetry.addData("Alliance:", "Blue");
        telemetry.addData("Side:", "Top");
        telemetry.addData("Total Count:", "18");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        Transfer.INSTANCE.intake().schedule();
        Turret.INSTANCE.start().schedule();
        Turret.INSTANCE.changeToAuto().schedule();
        Shooter.INSTANCE.on().schedule();

        new SequentialGroup(
                new Delay(1),
                new ParallelGroup(
                        new FollowPath(paths.R1Pickup),
                        new SequentialGroup(
                                new Delay(1.4),
                                Transfer.INSTANCE.openGate(),
                                new Delay(0.8),
                                Transfer.INSTANCE.closeGate()
                        )
                ),

                new FollowPath(paths.Launch1),
                new Delay(0.2),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.closeGate(),

                // Second Row (R2)
                new FollowPath(paths.R2Pickup),
                new FollowPath(paths.Launch2),
                new Delay(0.2),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.closeGate(),

                // Gate Pickup 1
                new FollowPath(paths.GatePickup1),
                new Delay(2),
                new FollowPath(paths.Launch3),
                new Delay(0.2),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.closeGate(),

                // Gate Pickup 2
                new FollowPath(paths.GatePickup2),
                new Delay(2),
                new FollowPath(paths.Launch4),
                new Delay(0.2),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.closeGate(),

                // Third Row (R3)
//                new FollowPath(paths.R3Pickup),
//                new FollowPath(paths.Launch5),
//                new Delay(0.2),
//                Transfer.INSTANCE.openGate(),
//                new Delay(Configuration.SHOOTER_TIME),
//                Transfer.INSTANCE.closeGate(),

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

        if (Shooter.INSTANCE.mode == Shooter.Mode.odometry) {
            double distMeters = Shooter.INSTANCE.GOAL_DISTANCE * 0.0254;

            double hoodRad = Math.toRadians(Shooter.INSTANCE.HOOD_ANGLE);
            double odoTarget = Turret.INSTANCE.ODO_TARGET;

            Shooter.INSTANCE.updateKinematics(distMeters, hoodRad);

            double weight = Shooter.INSTANCE.getWeight();
            Configuration.setAimPointOffset(-X_VELOCITY * weight, -Y_VELOCITY * weight);

            double vyr = ((Y_VELOCITY * 0.0254) * Math.sin(Math.PI / 2 - odoTarget))
                    + ((X_VELOCITY * 0.0254) * Math.sin(odoTarget));
            double vxr = -((Y_VELOCITY * 0.0254) * Math.cos(Math.PI / 2 - odoTarget))
                    + ((X_VELOCITY * 0.0254) * Math.cos(odoTarget));

            double vn = Shooter.INSTANCE.shooterVKinematic() + (vyr * Shooter.vcWeight);
            double vt = Math.sqrt((vn * vn) + (vxr * vxr));

            Shooter.INSTANCE.setHoodAngle(Shooter.INSTANCE.HOOD_ANGLE);
            Configuration.TURRET_OFFSET = 0;
            Shooter.INSTANCE.targetRPM = Shooter.INSTANCE.vMSToRPM(vt) * RPM_SCALE_FACTOR;
        }

        telemetry.addLine("=== Position ===");
        telemetry.addData("X", PedroComponent.follower().getPose().getX());
        telemetry.addData("Y", PedroComponent.follower().getPose().getY());
        telemetry.addData("Heading", headingDeg);

        telemetry.addLine();
        telemetry.addData("=== Shooter ===", "");
        telemetry.addData("Target RPM (model)", Shooter.INSTANCE.targetRPM);
        telemetry.addData("Read RPM", Shooter.INSTANCE.readRPM);

        telemetry.update();

        LOOP_TIME = LOOP_TIMER.milliseconds();
    }

    @Override
    public void onStop() {
        Turret.INSTANCE.emergencyStop().schedule();
        Transfer.INSTANCE.emergencyStopAll().schedule();
        Shooter.INSTANCE.emergencyStop().schedule();

        telemetry.setAutoClear(false);
        telemetry.addLine("=== Final Position ===");
        telemetry.addData("Final X", PedroComponent.follower().getPose().getX());
        telemetry.addData("Final Y", PedroComponent.follower().getPose().getY());
        telemetry.addData("Final Heading", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));
        telemetry.update();
    }

    public static class Paths {
        public PathChain R1Pickup;
        public PathChain Launch1;
        public PathChain R2Pickup;
        public PathChain Launch2;
        public PathChain GatePickup1;
        public PathChain Launch3;
        public PathChain GatePickup2;
        public PathChain Launch4;
        public PathChain R3Pickup;
        public PathChain Launch5;
        public PathChain Move;

        public Paths(Follower follower) {
            R1Pickup = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(32.582, 135.709),
                                    new Pose(68.914, 83.436),
                                    new Pose(78.223, 79.764),
                                    new Pose(45.241, 84.582),
                                    new Pose(15.500, 83.600)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Launch1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.500, 83.600),
                                    new Pose(59.364, 73.218)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            R2Pickup = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(59.364, 73.218),
                                    new Pose(43.940, 57.696),
                                    new Pose(48.107, 59.916),
                                    new Pose(49.961, 59.461),
                                    new Pose(15.500, 59.627)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Launch2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.500, 59.627),
                                    new Pose(59.364, 73.218)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            GatePickup1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(59.364, 73.218),
                                    new Pose(51.512, 57.326),
                                    new Pose(8.5, 62.400)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
                    .build();

            Launch3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(8.5, 62.400),
                                    new Pose(59.364, 73.218)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                    .build();

            GatePickup2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(59.364, 73.218),
                                    new Pose(51.512, 57.326),
                                    new Pose(8.5, 62.400)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
                    .build();

            Launch4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(11.000, 62.400),
                                    new Pose(59.364, 73.218)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                    .build();

            R3Pickup = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(59.364, 73.218),
                                    new Pose(42.562, 29.869),
                                    new Pose(53.311, 34.965),
                                    new Pose(46.156, 35.382),
                                    new Pose(15.199, 35.621)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Launch5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.199, 35.621),
                                    new Pose(59.364, 73.218)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Move = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.364, 73.218),
                                    new Pose(46.400, 73.218)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }
}