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

@Autonomous(name = "RB3HA - R3 & Human Player", group = "Red Alliance")
public class RB3HA extends NextFTCOpMode {
    double LOOP_TIME = 0;
    ElapsedTime LOOP_TIMER = new ElapsedTime();

    private Paths paths;

    // Starting Position
    private static final double START_X = 87.613;
    private static final double START_Y = 8.303;
    private static final double START_HEADING = 0;
    private double vt = 0;

    private double X_VELOCITY = 0;
    private double Y_VELOCITY = 0;
    public double TRUE_TARGET_DEGREE = 0;

    private static double RPM_SCALE_FACTOR = 2.75;

    public RB3HA() {
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
        telemetry.addData("Side:", "Bottom");
        telemetry.addData("Order:", "R3, Park x3, Move");
        telemetry.addData("Gate:", "False");
        telemetry.addData("Solo:", "True");
        telemetry.addData("Total Count:", "TBD");

        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {

        new SequentialGroup(
                new Delay(1),
                Transfer.INSTANCE.intake(),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.closeGate(),

                new FollowPath(paths.R3),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.closeGate(),

                // Park cycle 1
                new FollowPath(paths.Park1),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.closeGate(),

                // Park cycle 2
                new FollowPath(paths.Park2),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.closeGate(),

                // Park cycle 3
                new FollowPath(paths.Park3),
                Transfer.INSTANCE.openGate(),
                new Delay(Configuration.SHOOTER_TIME),
                Transfer.INSTANCE.closeGate(),

                // Move
                new FollowPath(paths.Move)
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
        Configuration.TURRET_OFFSET = 0;
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
        public PathChain R3;
        public PathChain Park1;
        public PathChain Park2;
        public PathChain Park3;
        public PathChain Move;

        public Paths(Follower follower) {
            // R3 Pickup + Launch as one PathChain
            R3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(87.613, 8.303),
                                    new Pose(94.174, 45.256), new Pose(92.233, 32.886), new Pose(93.628, 35.609),
                                    new Pose(128.801, 35.621)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(128.801, 35.621),
                                    new Pose(87.613, 8.303)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // Park cycle 1: PrePickup -> Backup -> Pickup -> Launch
            Park1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(87.613, 8.303),
                                    new Pose(137.500, 8.303)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(137.500, 8.303),
                                    new Pose(129.691, 8.303)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(129.691, 8.303),
                                    new Pose(137.500, 8.303)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(137.500, 8.303),
                                    new Pose(87.613, 8.303)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // Park cycle 2: PrePickup -> Backup -> Pickup -> Launch
            Park2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(87.613, 8.303),
                                    new Pose(137.500, 8.303)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(137.500, 8.303),
                                    new Pose(129.691, 8.303)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(129.691, 8.303),
                                    new Pose(137.500, 8.303)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(137.500, 8.303),
                                    new Pose(87.613, 8.303)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // Park cycle 3: PrePickup -> Backup -> Pickup -> Launch
            Park3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(87.613, 8.303),
                                    new Pose(137.500, 8.303)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(137.500, 8.303),
                                    new Pose(129.691, 8.303)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(129.691, 8.303),
                                    new Pose(137.500, 8.303)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(137.500, 8.303),
                                    new Pose(87.613, 8.303)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Move = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.613, 8.303),
                                    new Pose(105.286, 32.714)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        }
    }
}
