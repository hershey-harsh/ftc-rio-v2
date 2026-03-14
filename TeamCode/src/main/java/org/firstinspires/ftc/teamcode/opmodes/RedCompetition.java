package org.firstinspires.ftc.teamcode.opmodes;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Range;
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
import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.teamcode.pedro.Constants;
//import org.firstinspires.ftc.teamcode.subsystems.Light;
//import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Light;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name = "Red Competition SOTM", group = "Red")
public class RedCompetition extends NextFTCOpMode {
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

    private ElapsedTime matchTimer = new ElapsedTime();
    private boolean endGameWarningTriggered = false;
    private static final double MATCH_DURATION = 120.0; // 2 minutes in seconds
    private static final double END_GAME_WARNING = 20.0; // last 20 seconds

    public RedCompetition() {
        addComponents(
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Light.INSTANCE),
                new SubsystemComponent(Limelight.INSTANCE),
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
    }

    @Override
    public void onStartButtonPressed() {
        matchTimer.reset();
        endGameWarningTriggered = false;

        driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().negate(),
                !Configuration.FIELD_CENTRIC
        );

        driverControlled.schedule();
        Transfer.INSTANCE.intake().schedule();
        Shooter.INSTANCE.on().schedule();

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

        button(() -> gamepad1.dpad_up)
                .whenTrue(() -> {
                    new FollowPath(
                        PedroComponent.follower().pathBuilder().addPath(
                            new BezierCurve(
                                new Pose(
                                    Configuration.CURRENT_POSE.getX(),
                                    Configuration.CURRENT_POSE.getY(),
                                    Configuration.CURRENT_POSE.getHeading()
                                ),
                                Configuration.GATE_OPEN_RED
                            )
                        ).build()
                    ).schedule();
                });

        // X (Red) → Red Alliance Override
        button(() -> gamepad1.b)
                .whenTrue(() -> {
                    Configuration.ALLIANCE = Configuration.Alliance.RED;
                });

        // Y (Yellow) → Limelight Localization On / Off (toggle)
        button(() -> gamepad1.y)
                .toggleOnBecomesFalse()
                .whenBecomesTrue(() -> Limelight.INSTANCE.enableAutoUpdate().schedule())
                .whenBecomesFalse(() -> Limelight.INSTANCE.disableAutoUpdate().schedule());

        // Left Dpad → Relocalize Human Player
        button(() -> gamepad1.dpad_left)
                .whenTrue(() -> PedroComponent.follower().setPose(Configuration.MANUAL_LOCALIZATION_POSE));

        /// ---- Gamepad 2 (Operator) ---- ///

        // D-Pad Left → Turret Offset –
        button(() -> gamepad2.dpad_left)
                .whenTrue(() -> Turret.INSTANCE.decreaseAngle().schedule());

        // D-Pad Right → Turret Offset +
        button(() -> gamepad2.dpad_right)
                .whenTrue(() -> Turret.INSTANCE.increaseAngle().schedule());

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

        // Start → Driver 1 Override (toggle)
        button(() -> gamepad2.start)
                .toggleOnBecomesFalse()
                .whenBecomesTrue(() -> {
                    gamepad2Override = true;
                    driverControlled.cancel();
                    driverControlled = new PedroDriverControlled(
                            Gamepads.gamepad2().leftStickY().negate(),
                            Gamepads.gamepad2().leftStickX().negate(),
                            Gamepads.gamepad2().rightStickX().negate(),
                            !Configuration.FIELD_CENTRIC
                    );
                    driverControlled.schedule();
                })
                .whenBecomesFalse(() -> {
                    gamepad2Override = false;
                    driverControlled.cancel();
                    driverControlled = new PedroDriverControlled(
                            Gamepads.gamepad1().leftStickY().negate(),
                            Gamepads.gamepad1().leftStickX().negate(),
                            Gamepads.gamepad1().rightStickX().negate(),
                            !Configuration.FIELD_CENTRIC
                    );
                    driverControlled.schedule();
                });

        // X (Blue) → Hold Position Off
        button(() -> gamepad2.y)
                .whenTrue(() -> {
                    PedroComponent.follower().breakFollowing();
                });

        // A (Green) → Hold Position On
        button(() -> gamepad2.a)
                .whenTrue(() -> {
                    PedroComponent.follower().holdPoint(PedroComponent.follower().getPose());
                });

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

        telemetry.addData("Loop Time (ms)", LOOP_TIME);
        telemetry.addData("Loop Time (hz)", (1000/LOOP_TIME));

        Configuration.CURRENT_POSE = PedroComponent.follower().getPose();

        telemetry.addData("Drive Control", gamepad2Override ? "Gamepad 2" : "Gamepad 1");

        driverControlled.setScalar(Configuration.CONTROL_SCALE);

        if (Shooter.INSTANCE.mode == Shooter.Mode.odometry) {
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
                Configuration.TURRET_OFFSET = -2;
                Shooter.INSTANCE.updateKinematics(
                        Shooter.INSTANCE.GOAL_DISTANCE,
                        Math.toRadians(Shooter.INSTANCE.HOOD_ANGLE)
                );

                Shooter.INSTANCE.TARGET_RPM = Shooter.artifactVelocityMStoRPM(vt) * Configuration.RPM_MULTIPLER;
            }
        }


        // End-game warning: blink red 4 times on both lights in the last 20 seconds
        if (!endGameWarningTriggered && matchTimer.seconds() >= (MATCH_DURATION - END_GAME_WARNING)) {
            endGameWarningTriggered = true;
            Light.INSTANCE.setBlinkingColor(Light.RED, 250, 4, Light.Target.BOTH).schedule();
        }

        BindingManager.update();
        telemetry.update();

        LOOP_TIME = LOOP_TIMER.milliseconds();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
    }
}
