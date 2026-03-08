package org.firstinspires.ftc.teamcode.opmodes;
import com.pedropathing.geometry.Pose;
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
import dev.nextftc.hardware.controllable.Controllable;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.driving.RobotCentric;
import dev.nextftc.hardware.impl.MotorEx;

import static dev.nextftc.bindings.Bindings.*;
import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.teamcode.pedro.Constants;
//import org.firstinspires.ftc.teamcode.subsystems.Light;
//import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name = "Red Competition")
public class Competition extends NextFTCOpMode {
    private double X_VELOCITY = 0;
    private double Y_VELOCITY = 0;

    double LOOP_TIME = 0;
    ElapsedTime LOOP_TIMER = new ElapsedTime();

    private MotorEx frontLeftMotor = new MotorEx(Configuration.LEFT_FRONT_MOTOR).reversed();
    private MotorEx frontRightMotor = new MotorEx(Configuration.RIGHT_FRONT_MOTOR);
    private MotorEx backLeftMotor = new MotorEx(Configuration.LEFT_REAR_MOTOR).reversed();
    private MotorEx backRightMotor = new MotorEx(Configuration.RIGHT_REAR_MOTOR);
    private DriverControlledCommand driverControlled;

    private boolean gamepad2Override = false;

    public Competition() {
        addComponents(
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
//                new SubsystemComponent(Light.INSTANCE),
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
        PedroComponent.follower().setStartingPose(new Pose(72, 72, Math.toRadians(270)));
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

        // Turret starts tracking goal immediately
        Turret.INSTANCE.mode = Turret.Mode.odometry;
        Turret.INSTANCE.start().schedule();
        Shooter.INSTANCE.on().schedule();

//        button(() -> gamepad1.a && !gamepad2Override)
//                .toggleOnBecomesFalse()
//                .whenBecomesTrue(() -> Limelight.INSTANCE.enableAutoUpdate().schedule())
//                .whenBecomesFalse(() -> Limelight.INSTANCE.disableAutoUpdate().schedule());

//        button(() -> gamepad1.b && !gamepad2Override)
//                .whenTrue(() -> Limelight.INSTANCE.update().schedule());

        button(() -> gamepad1.x && !gamepad2Override)
                .whenTrue(() -> PedroComponent.follower().setPose(Configuration.MANUAL_LOCALIZATION_POSE));

        button(() -> gamepad1.back && !gamepad2Override)
                .toggleOnBecomesTrue()
                .whenTrue(() -> {
//                    Light.INSTANCE.setBlinkingColor(Light.RED, 250).schedule();
                    Shooter.INSTANCE.emergencyStop().schedule();
                    Transfer.INSTANCE.emergencyStopAll().schedule();
                    Turret.INSTANCE.emergencyStop().schedule();
                })
                .whenBecomesFalse(() -> {
//                    Light.INSTANCE.setColor(Light.GREEN, Light.Target.ROBOT).schedule();
                    Shooter.INSTANCE.start().schedule();
                    Transfer.INSTANCE.startAll().schedule();
                    Turret.INSTANCE.start().schedule();
                });

        ///  Gamepad 2 Bindings  ///

//        button(() -> gamepad2.right_bumper)
//                .toggleOnBecomesFalse()
//                .whenBecomesTrue(() -> Turret.INSTANCE.changeToAuto().schedule())
//                .whenBecomesFalse(() -> Turret.INSTANCE.changeToManual().schedule());

        button(() -> gamepad2.left_bumper)
                .toggleOnBecomesFalse()
                .whenBecomesTrue(() -> Shooter.INSTANCE.on().schedule())
                .whenBecomesFalse(() -> Shooter.INSTANCE.off().schedule());

        button(() -> gamepad2.right_bumper)
                .whenTrue(() -> Transfer.INSTANCE.openGate().schedule())
                .whenBecomesFalse(() -> Transfer.INSTANCE.closeGate().schedule());

//        button(() -> gamepad2.b)
//                .toggleOnBecomesFalse()
//                .whenBecomesTrue(() -> Transfer.INSTANCE.gate3Start().schedule())
//                .whenBecomesFalse(() -> Transfer.INSTANCE.gate3Stop().schedule());
//
        button(() -> gamepad2.a)
                .toggleOnBecomesFalse()
                .whenBecomesTrue(() -> Transfer.INSTANCE.intake().schedule())
                .whenBecomesFalse(() -> Transfer.INSTANCE.outtake().schedule());

        button(() -> gamepad2.y)
                .toggleOnBecomesFalse()
                .whenBecomesTrue(() -> Shooter.INSTANCE.changeToAuto().schedule())
                .whenBecomesFalse(() -> Shooter.INSTANCE.changeToManual().schedule());

        button(() -> gamepad2.x)
                .toggleOnBecomesFalse()
                .whenBecomesTrue(() -> Turret.INSTANCE.changeToAuto().schedule())
                .whenBecomesFalse(() -> Turret.INSTANCE.changeToManual().schedule());

        button(() -> gamepad2.dpad_left)
                .whenTrue(() -> Turret.INSTANCE.increaseAngle().schedule());

        button(() -> gamepad2.dpad_right)
                .whenTrue(() -> Turret.INSTANCE.decreaseAngle().schedule());

        button(() -> gamepad2.dpad_up)
                .whenTrue(() -> Shooter.INSTANCE.increaseHood().schedule());

        button(() -> gamepad2.dpad_down)
                .whenTrue(() -> Shooter.INSTANCE.decreaseHood().schedule());

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

        button(() -> gamepad2.back)
                .toggleOnBecomesTrue()
                .whenTrue(() -> {
//                    Light.INSTANCE.setBlinkingColor(Light.RED, 250).schedule();
                    Shooter.INSTANCE.emergencyStop().schedule();
                    Transfer.INSTANCE.emergencyStopAll().schedule();
                    Turret.INSTANCE.emergencyStop().schedule();
                })
                .whenBecomesFalse(() -> {
//                    Light.INSTANCE.setColor(Light.GREEN, Light.Target.ROBOT).schedule();
                    Shooter.INSTANCE.start().schedule();
                    Transfer.INSTANCE.startAll().schedule();
                    Turret.INSTANCE.start().schedule();
                });

        //TODO: Test left stick x on Gamepad 2.

        Range shooterAdjustRange = Gamepads.gamepad2().leftStickX()
                .map(value -> !gamepad2Override ? value : 0)
                .deadZone(0.1);
        shooterAdjustRange
                .asButton(value -> value != 0)
                .whenTrue(() -> Shooter.INSTANCE.adjustShooterRPM(shooterAdjustRange.get()).schedule());
    }

    @Override
    public void onUpdate() {
        LOOP_TIMER.reset();

        telemetry.addData("Loop Time (ms)", LOOP_TIME);
        telemetry.addData("Loop Time (hz)", (1000/LOOP_TIME));

        Configuration.CURRENT_POSE = PedroComponent.follower().getPose();

        telemetry.addData("Drive Control", gamepad2Override ? "Gamepad 2" : "Gamepad 1");

        driverControlled.setScalar(Configuration.CONTROL_SCALE);

        X_VELOCITY = PedroComponent.follower().getVelocity().getXComponent();
        Y_VELOCITY = PedroComponent.follower().getVelocity().getYComponent();

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
            Configuration.TURRET_OFFSET = 2;
            Shooter.INSTANCE.targetRPM = Shooter.INSTANCE.vMSToRPM(vt) * 2.75; //2.75 weight if stationary
//                Shooter.INSTANCE.targetRPM = Shooter.INSTANCE.getKinematicRPMGoal() * 2.75;
        }


//        telemetry.addData("COLOR R:", "%.3f", Transfer.redValues);
//        telemetry.addData("COLOR G:", "%.3f", Transfer.greenValues);
//        telemetry.addData("COLOR B:", "%.3f", Transfer.blueValues);
//        telemetry.addData("Is Purple:", Transfer.INSTANCE.isPurple());
//        telemetry.addData("Is Green:", Transfer.INSTANCE.isGreen());

        telemetry.addData("=== Position ===", "");
        telemetry.addData("Position X:", PedroComponent.follower().getPose().getX());
        telemetry.addData("Position Y:", PedroComponent.follower().getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));

//        telemetry.addData("=== Limelight ===", "");
//        telemetry.addData("LL Auto Update", Limelight.INSTANCE.autoUpdateEnabled);
//        telemetry.addData("LL Localizing", Limelight.INSTANCE.limelightResult != null && Limelight.INSTANCE.limelightResult.isValid());

        telemetry.addData("=== Turret ===", "");
        telemetry.addData("Turret Mode", Turret.INSTANCE.mode);
        telemetry.addData("Turret Angle", Turret.INSTANCE.TURRET_ANGLE);
        telemetry.addData("Turret Position", Turret.INSTANCE.TURRET_POSITION);
//        telemetry.addData("Turret Target", Turret.INSTANCE.TARGET_DEGREE);
//        telemetry.addData("Turret Servo Pos", "%.3f", Turret.INSTANCE.TURRET_POSITION);

        telemetry.addData("=== Shooter ===", "");
//        telemetry.addData("Shooter Mode", Shooter.INSTANCE.mode);
        telemetry.addData("Goal Distance (in)", Shooter.INSTANCE.GOAL_DISTANCE);
        telemetry.addData("Flywheel RPM (measured)", Shooter.INSTANCE.readRPM);
        telemetry.addData("Flywheel RPM Goal (static)", Shooter.INSTANCE.getKinematicRPMGoal());
        telemetry.addData("Flywheel RPM Goal (SOTM corrected)", Shooter.INSTANCE.targetRPM);
        telemetry.addData("Hood Angle (deg)", Shooter.INSTANCE.HOOD_ANGLE);
        telemetry.addData("Hood Position", Shooter.INSTANCE.HOOD_POSITION);
        telemetry.addData("TOF estimate (s)", Shooter.INSTANCE.getTof());
        telemetry.addData("Aim Weight (s)", Shooter.INSTANCE.getWeight());
        telemetry.addData("X Offset", Configuration.X_GOAL_OFFSET);
        telemetry.addData("Y Offset", Configuration.Y_GOAL_OFFSET);
        //telemetry.addData("Hood Angle", Shooter.INSTANCE.HOOD_ANGLE);
        //telemetry.addData("Close Run (ms)", Shooter.INSTANCE.runMs);

        BindingManager.update();
        telemetry.update();

        LOOP_TIME = LOOP_TIMER.milliseconds();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
    }
}