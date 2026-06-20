package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Range;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
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
import org.firstinspires.ftc.teamcode.subsystems.AimController;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Disabled
@TeleOp(name = "(FORZA) Red Competition SOTM (FULL)", group = "Red Forza")
public class ForzaRedCompetitionSOTM extends NextFTCOpMode {
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
    private Supplier<PathChain> parkPath;


    public ForzaRedCompetitionSOTM() {
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

        //PedroComponent.follower().setStartingPose(new com.pedropathing.geometry.Pose(72, 72, Math.toRadians(270)));

        Configuration.CURRENT_POSE = PedroComponent.follower().getPose();

        Configuration.SHOOTER_HEIGHT_TO_GOAL = 1.1;
        Configuration.ALLIANCE = Configuration.Alliance.RED;

        Limelight.INSTANCE.MODE = Limelight.Mode.LOCALIZATION;
        Transfer.INSTANCE.GATE12_DEBOUNCE_THRESHOLD = 3;
    }

    @Override
    public void onStartButtonPressed() {

        Range leftTrigger = Gamepads.gamepad1().leftTrigger();
        driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().rightTrigger().map(rt -> rt - leftTrigger.get()),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().negate(),
                !Configuration.FIELD_CENTRIC
        );

        driverControlled.schedule();
        Transfer.INSTANCE.intake().schedule();
        Shooter.INSTANCE.on().schedule();

        // Lazy path — built from the robot's current pose at the moment the button is pressed
        gateOpenPath = () -> {
            boolean isBlue = Configuration.ALLIANCE == Configuration.Alliance.BLUE;
            Pose target = isBlue ? Configuration.GATE_OPEN_BLUE : Configuration.GATE_OPEN_RED;
            Pose controlPoint = isBlue ? Configuration.GATE_OPEN_BLUE_2 : Configuration.GATE_OPEN_RED_2;
            return PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    PedroComponent.follower().getPose(),
                                    controlPoint,
                                    target
                            )
                    )
                    .setLinearHeadingInterpolation(
                            PedroComponent.follower().getPose().getHeading(),
                            target.getHeading()
                    )
                    .build();
        };

        // Lazy path — built from the robot's current pose at the moment the button is pressed
        parkPath = () -> {
            Pose target = Configuration.RED_PARK_BR;
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
                    // Don't cancel driverControlled — the deferred cancel would call
                    // breakFollowing() AFTER followPath(), destroying the path.
                    // followPath() sets manualDrive=false so teleop vectors are ignored.
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

        // D-Pad Right → Automated drive to park position
        button(() -> gamepad1.dpad_right)
                .whenBecomesTrue(() -> {
                    PedroComponent.follower().followPath(parkPath.get());
                    automatedDrive = true;
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

        // D-Pad Left → Turret Offset –
        button(() -> gamepad2.dpad_left)
                .whenTrue(() -> Turret.INSTANCE.decreaseAngle().schedule());

        // D-Pad Right → Turret Offset +
        button(() -> gamepad2.dpad_right)
                .whenTrue(() -> Turret.INSTANCE.increaseAngle().schedule());
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

        // B → Intake Off
        button(() -> gamepad2.b)
                .whenTrue(() -> Transfer.INSTANCE.stop().schedule());

        // X → Intake On
        button(() -> gamepad2.x)
                .whenTrue(() -> Transfer.INSTANCE.intake().schedule());

        // Right Bumper → Gate Open while held, close when released
        // Use 50% transfer power when close to goal (Y < 36)
        button(() -> gamepad2.right_bumper)
                .whenTrue(() -> {
                    double power = Configuration.CURRENT_POSE.getY() < 36 ? 1.0 : 1.0;
                    Transfer.INSTANCE.openGate(power).schedule();
                })
                .whenBecomesFalse(() -> Transfer.INSTANCE.closeGate().schedule());
    }

    @Override
    public void onUpdate() {
        LOOP_TIMER.reset();

        Configuration.CURRENT_POSE = PedroComponent.follower().getPose();

        // Robot light priority: YELLOW (following path) > AZURE (3 balls) > GREEN (1 ball) > RED (default)
        if (PedroComponent.follower().isBusy()) {
            Light.INSTANCE.setColor(Light.YELLOW, Light.Target.ROBOT).schedule();
        } else if (Transfer.INSTANCE.ALL_STOPPED) {
            Light.INSTANCE.setColor(Light.AZURE, Light.Target.ROBOT).schedule();
        } else if (Transfer.INSTANCE.GATE12_BALL_PRESENT) {
            Light.INSTANCE.setColor(Light.GREEN, Light.Target.ROBOT).schedule();
        } else {
            Light.INSTANCE.setColor(Light.RED, Light.Target.ROBOT).schedule();
        }

        if (automatedDrive && !PedroComponent.follower().isBusy()) {
            automatedDrive = false;
            PedroComponent.follower().startTeleopDrive();
        }

        telemetry.addData("Loop Time (ms)", LOOP_TIME);
        telemetry.addData("Loop Time (hz)", LOOP_TIME > 0 ? (1000 / LOOP_TIME) : 0);

        telemetry.addData("X", Configuration.CURRENT_POSE.getX());
        telemetry.addData("Y", Configuration.CURRENT_POSE.getY());
        telemetry.addData("Z", Math.toDegrees(Configuration.CURRENT_POSE.getHeading()));

        BindingManager.update();
        driverControlled.setScalar(Configuration.CONTROL_SCALE);

        if (Shooter.INSTANCE.MODE == Shooter.Mode.ODOMETRY) {
            X_VELOCITY = PedroComponent.follower().getVelocity().getXComponent();
            Y_VELOCITY = PedroComponent.follower().getVelocity().getYComponent();

            // Shoot-on-the-move: virtual-target lead. Compensates lead direction (any
            // translation, incl. strafing) and shot speed in one step; zero at rest.
            AimController.updateShootOnMove(X_VELOCITY, Y_VELOCITY);

            if (Configuration.CURRENT_POSE.getY() < 36) {
                // Close range: small mechanical aim offset + slightly hotter wheel.
                Configuration.TURRET_OFFSET = -2;
                Shooter.INSTANCE.TARGET_RPM = Shooter.INSTANCE.getKinematicRPMGoal() * (Configuration.RPM_MULTIPLER + 0.15);
            } else {
                // Far: mechanical aim offset only — the lead is handled by AimController.
                Configuration.TURRET_OFFSET = -0.5;
            }
        }

        boolean RPM_READY = Shooter.INSTANCE.TARGET_RPM > 0
                && Math.abs(Shooter.INSTANCE.TARGET_RPM - Shooter.INSTANCE.CURRENT_RPM) <= Shooter.RPM_TOLERANCE;
        boolean VELOCITY_LOW = Math.abs(X_VELOCITY) < 5 && Math.abs(Y_VELOCITY) < 5;
        if (Turret.INSTANCE.TURRET_IN_RANGE && RPM_READY && VELOCITY_LOW) {
            Light.INSTANCE.setColor(Light.RED, Light.Target.TURRET).schedule();
        } else {
            Light.INSTANCE.setColor(Light.YELLOW, Light.Target.TURRET).schedule();
        }

        telemetry.update();

        LOOP_TIME = LOOP_TIMER.milliseconds();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
    }
}
