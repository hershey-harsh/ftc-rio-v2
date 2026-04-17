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

@Autonomous(name = "RB3AH9 - R3", group = "Red Alliance")
public class RB3AH9 extends NextFTCOpMode {
    double LOOP_TIME = 0;
    ElapsedTime LOOP_TIMER = new ElapsedTime();

    private Paths paths;

    // Starting Position
    private static final double START_X = 86.184;
    private static final double START_Y = 8.017;
    private static final double START_HEADING = 90;
    private double vt = 0;

    private double X_VELOCITY = 0;
    private double Y_VELOCITY = 0;
    public double TRUE_TARGET_DEGREE = 0;

    private boolean WAS_FOLLOWER_BUSY = false;

    public RB3AH9() {
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
        Configuration.SHOOTER_HEIGHT_TO_GOAL = 1.02;

        paths = new Paths(PedroComponent.follower());

        Pose startPose = new Pose(START_X, START_Y, Math.toRadians(START_HEADING));
        PedroComponent.follower().setStartingPose(startPose);
        Configuration.CURRENT_POSE = startPose;

        Transfer.INSTANCE.start().schedule();
        Turret.INSTANCE.start().schedule();
        Shooter.INSTANCE.start().schedule();

        telemetry.addData("Alliance:", "Red");
        telemetry.addData("Side:", "Bottom");
        telemetry.addData("Order:", "R3");
        telemetry.addData("Gate:", "False");
        telemetry.addData("Solo:", "True");
        telemetry.addData("Total Count:", "9");

        telemetry.update();
        Light.INSTANCE.setBlinkingColor(Light.RED, Light.Target.ROBOT).schedule();
    }

    @Override
    public void onStartButtonPressed() {

        Shooter.INSTANCE.on().schedule();
        Transfer.INSTANCE.intake().schedule();

        new SequentialGroup(
                new Delay(3),
                Transfer.INSTANCE.openGate(),
                new Delay(0.7),
                Transfer.INSTANCE.closeGate(),

                /// Go to Human Player zone and collect.
                new FollowPath(paths.R3),
                Transfer.INSTANCE.openGate(),
                new Delay(0.7),
                Transfer.INSTANCE.closeGate(),

                new FollowPath(paths.HumanPlayer1),
                Transfer.INSTANCE.openGate(),
                new Delay(0.7),
                Transfer.INSTANCE.closeGate(),

                new FollowPath(paths.HumanPlayer1),
                Transfer.INSTANCE.openGate(),
                new Delay(0.7),
                Transfer.INSTANCE.closeGate(),

                new FollowPath(paths.HumanPlayer1),
                Transfer.INSTANCE.openGate(),
                new Delay(0.7),
                Transfer.INSTANCE.closeGate(),

                new FollowPath(paths.HumanPlayer1),
                Transfer.INSTANCE.openGate(),
                new Delay(0.7),
                Transfer.INSTANCE.closeGate()

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

        Shooter.INSTANCE.TARGET_RPM = Shooter.INSTANCE.KINEMATIC_RPM_GOAL * (Configuration.RPM_MULTIPLER);

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
        double vt = Math.sqrt((vn * vn) + (vxr * vxr));

        Shooter.INSTANCE.updateKinematics(
                Shooter.INSTANCE.GOAL_DISTANCE,
                Math.toRadians(Shooter.INSTANCE.HOOD_ANGLE)
        );

        if (Configuration.CURRENT_POSE.getY() < 36) {
            Configuration.TURRET_OFFSET = -1.5;
            Shooter.INSTANCE.TARGET_RPM = Shooter.artifactVelocityMStoRPM(vt) * (Configuration.RPM_MULTIPLER + 0.15);
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
        public PathChain R3;
        public PathChain HumanPlayer1;

        public Paths(Follower follower) {
            R3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(86.184, 8.017),
                            new Pose(84.300, 38.805),
                            new Pose(127.583, 35.287)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .addPath(new BezierLine(
                            new Pose(127.583, 35.287),
                            new Pose(91.758, 11.447)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(35))
                    .build();

            HumanPlayer1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(91.758, 11.447),
                            new Pose(124.146, 8.226)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(new BezierLine(
                            new Pose(124.146, 8.226),
                            new Pose(120.908, 8.226)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(new BezierLine(
                            new Pose(120.908, 8.226),
                            new Pose(124.146, 8.226)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(new BezierLine(
                            new Pose(124.146, 8.226),
                            new Pose(82.319, 11.942)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35))
                    .build();
        }
    }
}
