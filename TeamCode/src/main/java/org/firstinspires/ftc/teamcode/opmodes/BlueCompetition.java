package org.firstinspires.ftc.teamcode.opmodes;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Range;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.impl.MotorEx;

import static dev.nextftc.bindings.Bindings.*;

import java.util.function.Supplier;

import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.teamcode.pedro.Constants;
//import org.firstinspires.ftc.teamcode.subsystems.Light;
//import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Light;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name = "Blue Competition SOTM", group = "Blue")
public class BlueCompetition extends NextFTCOpMode {
    private double X_VELOCITY = 0;
    private double Y_VELOCITY = 0;
    public double TRUE_TARGET_DEGREE = 0;

    double LOOP_TIME = 0;
    ElapsedTime LOOP_TIMER = new ElapsedTime();

    private MotorEx frontLeftMotor = new MotorEx(Configuration.LEFT_FRONT_MOTOR).reversed();
    private MotorEx frontRightMotor = new MotorEx(Configuration.RIGHT_FRONT_MOTOR);
    private MotorEx backLeftMotor = new MotorEx(Configuration.LEFT_REAR_MOTOR).reversed();
    private MotorEx backRightMotor = new MotorEx(Configuration.RIGHT_REAR_MOTOR);
    private DriverControlledCommand driverControlled;

    private boolean gamepad2Override = false;
    private boolean automatedDrive = false;
    private Supplier<PathChain> gateOpenPath;


    public BlueCompetition() {
        addComponents(
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Light.INSTANCE),
//                new SubsystemComponent(Limelight.INSTANCE),
                new SubsystemComponent(Shooter.INSTANCE),
                new SubsystemComponent(Transfer.INSTANCE),
                new SubsystemComponent(Turret.INSTANCE)
        );
    }

    @Override
    public void onInit() {
        frontLeftMotor.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightMotor.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftMotor.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightMotor.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        PedroComponent.follower().setStartingPose(Configuration.CURRENT_POSE);
        Configuration.SHOOTER_HEIGHT_TO_GOAL = 1.1;
        Configuration.ALLIANCE = Configuration.Alliance.BLUE;
    }

    @Override
    public void onStartButtonPressed() {

        driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().negate(),
                !Configuration.FIELD_CENTRIC
        );

        driverControlled.schedule();
        Transfer.INSTANCE.intake().schedule();
        Shooter.INSTANCE.on().schedule();

        // Lazy path — built from the robot's current pose at the moment the button is pressed
        gateOpenPath = () -> {
            Pose target = Configuration.ALLIANCE == Configuration.Alliance.BLUE
                    ? Configuration.GATE_OPEN_BLUE
                    : Configuration.GATE_OPEN_RED;
            return PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierLine(
                                    PedroComponent.follower().getPose(),
                                    target
                            )
                    )
                    .setLinearHeadingInterpolation(
                            PedroComponent.follower().getPose().getHeading(),
                            target.getHeading()
                    )
                    .build();
        };

        /// ---- Gamepad 1 (Driver) ---- ///

        // Left D-Pad → Emergency Stop (toggle)
        button(() -> gamepad1.back)
                .toggleOnBecomesTrue()
                .whenTrue(() -> {
                    Shooter.INSTANCE.emergencyStop().schedule();
                    Transfer.INSTANCE.emergencyStopAll().schedule();
                    Turret.INSTANCE.emergencyStop().schedule();
                })
                .whenBecomesFalse(() -> {
                    Shooter.INSTANCE.start().schedule();
                    Transfer.INSTANCE.start().schedule();
                    Turret.INSTANCE.start().schedule();
                });

        // X (Blue) → Blue Alliance Override
        button(() -> gamepad1.x)
                .whenTrue(() -> {
                    Configuration.ALLIANCE = Configuration.Alliance.BLUE;
                });

        // D-Pad Up → Automated drive to gate open position
        button(() -> gamepad1.dpad_up)
                .whenBecomesTrue(() -> {
                    PedroComponent.follower().followPath(gateOpenPath.get());
                    automatedDrive = true;
                });

        // D-Pad Down → Cancel automated drive, resume manual
        button(() -> gamepad1.dpad_down)
                .whenBecomesTrue(() -> {
                    if (automatedDrive) {
                        PedroComponent.follower().breakFollowing();
                        PedroComponent.follower().startTeleopDrive();
                        automatedDrive = false;
                    }
                });

        // X (Red) → Red Alliance Override
        button(() -> gamepad1.b)
                .whenTrue(() -> {
                    Configuration.ALLIANCE = Configuration.Alliance.RED;
                });

        // Y (Yellow) → Limelight is always on (fusion localizer), no toggle needed

        // Left Dpad → Relocalize Human Player
        button(() -> gamepad1.dpad_left)
                .whenTrue(() -> PedroComponent.follower().setPose(
                        Configuration.ALLIANCE == Configuration.Alliance.BLUE
                                ? Configuration.BLUE_LOCALIZATION_POSE
                                : Configuration.RED_LOCALIZATION_POSE
                ));

        /// ---- Gamepad 2 (Operator) ---- ///

//        // D-Pad Left → Turret Offset –
//        button(() -> gamepad2.dpad_left)
//                .whenTrue(() -> Turret.INSTANCE.decreaseAngle().schedule());
//
//        // D-Pad Right → Turret Offset +
//        button(() -> gamepad2.dpad_right)
//                .whenTrue(() -> Turret.INSTANCE.increaseAngle().schedule());
//
//        // D-Pad Up → Reset Turret Offset
//        button(() -> gamepad2.dpad_up)
//                .whenTrue(() -> Turret.INSTANCE.resetAngle().schedule());


        // Back → Emergency Stop (toggle)
        button(() -> gamepad2.back)
                .toggleOnBecomesTrue()
                .whenTrue(() -> {
                    Shooter.INSTANCE.emergencyStop().schedule();
                    Transfer.INSTANCE.emergencyStopAll().schedule();
                    Turret.INSTANCE.emergencyStop().schedule();
                })
                .whenBecomesFalse(() -> {
                    Shooter.INSTANCE.start().schedule();
                    Transfer.INSTANCE.start().schedule();
                    Turret.INSTANCE.start().schedule();
                });

//        // Start → Driver 1 Override (toggle)
//        button(() -> gamepad2.start)
//                .toggleOnBecomesFalse()
//                .whenBecomesTrue(() -> {
//                    gamepad2Override = true;
//                    driverControlled.cancel();
//                    driverControlled = new PedroDriverControlled(
//                            Gamepads.gamepad2().leftStickY().negate(),
//                            Gamepads.gamepad2().leftStickX().negate(),
//                            Gamepads.gamepad2().rightStickX().negate(),
//                            !Configuration.FIELD_CENTRIC
//                    );
//                    driverControlled.schedule();
//                })
//                .whenBecomesFalse(() -> {
//                    gamepad2Override = false;
//                    driverControlled.cancel();
//                    driverControlled = new PedroDriverControlled(
//                            Gamepads.gamepad1().leftStickY().negate(),
//                            Gamepads.gamepad1().leftStickX().negate(),
//                            Gamepads.gamepad1().rightStickX().negate(),
//                            !Configuration.FIELD_CENTRIC
//                    );
//                    driverControlled.schedule();
//                });

        // B (Red) → Intake Off
        button(() -> gamepad2.b)
                .whenTrue(() -> Transfer.INSTANCE.stop().schedule());

        // X (Red) → Intake On
        button(() -> gamepad2.x)
                .whenTrue(() -> Transfer.INSTANCE.intake().schedule());

        button(() -> gamepad2.right_bumper)
                .whenTrue(() -> Transfer.INSTANCE.openGate().schedule())
                .whenBecomesFalse(() -> Transfer.INSTANCE.closeGate().schedule());

        // Right Stick → Intake On (when stick is pushed past deadzone)
        Range intakeRange = Gamepads.gamepad2().rightStickY()
                .deadZone(0.3);
        intakeRange
                .asButton(value -> Math.abs(value) > 0)
                .whenTrue(() -> Transfer.INSTANCE.intake().schedule());
    }

    @Override
    public void onUpdate() {
        LOOP_TIMER.reset();

        // Stop automated path following when done or driver takes over
        if (automatedDrive && !PedroComponent.follower().isBusy()) {
            automatedDrive = false;
            PedroComponent.follower().startTeleopDrive();
        }

        telemetry.addData("Loop Time (ms)", LOOP_TIME);
        telemetry.addData("Loop Time (hz)", (1000/LOOP_TIME));
        telemetry.addData("Automated Drive", automatedDrive);

        Configuration.CURRENT_POSE = PedroComponent.follower().getPose();
        BindingManager.update();

        telemetry.addData("Drive Control", gamepad2Override ? "Gamepad 2" : "Gamepad 1");

        // Gate sensor telemetry
        telemetry.addData("Gate 3 Dist (cm)", Transfer.gate3Distance);
        telemetry.addData("Gate 3 Ball", Transfer.INSTANCE.GATE3_BALL_PRESENT);
        telemetry.addData("Gate 3 Motor Off", Transfer.INSTANCE.GATE3_STOPPED);
        telemetry.addData("G12 Raw (true=clear)", Transfer.INSTANCE.gate1And2Sensor.getState());
        telemetry.addData("G12 Transitions", Transfer.INSTANCE.gate12TransitionCount);
        telemetry.addData("G12 Ball", Transfer.INSTANCE.GATE12_BALL_PRESENT);
        telemetry.addData("All Motors Off", Transfer.INSTANCE.ALL_STOPPED);

//        // AprilTag / Limelight Position
//        if (Limelight.INSTANCE.lastPedroPose != null) {
//            telemetry.addData("AT Pose X", Limelight.INSTANCE.lastPedroPose.getX());
//            telemetry.addData("AT Pose Y", Limelight.INSTANCE.lastPedroPose.getY());
//            telemetry.addData("AT Heading", Math.toDegrees(Limelight.INSTANCE.lastPedroPose.getHeading()));
//        } else {
//            telemetry.addData("AT Pose", "No detection");
//        }
//        if (Limelight.INSTANCE.lastRawBotpose != null) {
//            telemetry.addData("AT Raw X (m)", Limelight.INSTANCE.lastRawBotpose.getPosition().x);
//            telemetry.addData("AT Raw Y (m)", Limelight.INSTANCE.lastRawBotpose.getPosition().y);
//        }
//        telemetry.addData("LL Polls", Limelight.INSTANCE.pollCount);
//        telemetry.addData("LL Valid", Limelight.INSTANCE.validResultCount);
//        telemetry.addData("LL Measurements Sent", Limelight.INSTANCE.measurementSentCount);
//        telemetry.addData("Limelight Auto-Update", Limelight.INSTANCE.autoUpdateEnabled);

        driverControlled.setScalar(Configuration.CONTROL_SCALE);

        if (Shooter.INSTANCE.MODE == Shooter.Mode.ODOMETRY) {
            if (Configuration.CURRENT_POSE.getY() < 36) {
                Configuration.setAimPointOffset(0, 0);
                Shooter.INSTANCE.TARGET_RPM = 4300;
                Configuration.TURRET_OFFSET = 0;
            } else {
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
//
                Configuration.TURRET_OFFSET = Configuration.ALLIANCE == Configuration.Alliance.BLUE ? 1 : -1;
                Shooter.INSTANCE.updateKinematics(
                        Shooter.INSTANCE.GOAL_DISTANCE,
                        Math.toRadians(Shooter.INSTANCE.HOOD_ANGLE)
                );

                Shooter.INSTANCE.TARGET_RPM = Shooter.artifactVelocityMStoRPM(vt) * Configuration.RPM_MULTIPLER;
            }
        }

        telemetry.update();

        LOOP_TIME = LOOP_TIMER.milliseconds();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
    }
}
