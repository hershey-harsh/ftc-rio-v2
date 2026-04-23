package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Light;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import java.util.function.Supplier;
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
import static dev.nextftc.bindings.Bindings.button;

@TeleOp(name = "Blue FLL Demo", group = "AA FLL Demo")
public class BlueFLLDemoSTOMBase extends NextFTCOpMode {
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
    private boolean automatedDrive = false;
    private Supplier<PathChain> gateOpenPath;
    private Supplier<PathChain> parkPath;

    public void setupOpMode() {
        Configuration.ALLIANCE = Configuration.Alliance.BLUE;
    }

    public BlueFLLDemoSTOMBase() {
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
//        Center
//        PedroComponent.follower().setStartingPose(new com.pedropathing.geometry.Pose(72, 72, Math.toRadians(270)));
//        Tiny Triangle
//        PedroComponent.follower().setStartingPose(new com.pedropathing.geometry.Pose(72, 7.5, Math.toRadians(90)));
        Configuration.CURRENT_POSE = PedroComponent.follower().getPose();
        Configuration.SHOOTER_HEIGHT_TO_GOAL = 1.1;
        setupOpMode();
        Limelight.INSTANCE.MODE = Limelight.Mode.LOCALIZATION;
        Transfer.INSTANCE.GATE12_DEBOUNCE_THRESHOLD = 3;
    }

    @Override
    public void onStartButtonPressed() {
        Transfer.INSTANCE.override = false;
//        driverControlled = new PedroDriverControlled(
//                Gamepads.gamepad1().leftStickY().negate(),
//                Gamepads.gamepad1().leftStickX().negate(),
//                Gamepads.gamepad1().rightStickX().negate(),
//                !Configuration.FIELD_CENTRIC
//        );
        Gamepads.gamepad1().leftStickX().update();
        Gamepads.gamepad1().leftStickY().update();
        Gamepads.gamepad1().rightStickX().update();
        Gamepads.gamepad2().leftStickX().update();
        Gamepads.gamepad2().leftStickY().update();
        Gamepads.gamepad2().rightStickX().update();

        driverControlled = new PedroDriverControlled(
                () -> Gamepads.gamepad1().leftStickY().get() !=0 ? -Gamepads.gamepad1().leftStickY().get() : -Gamepads.gamepad2().leftStickY().get() * 0.5,
                () -> Gamepads.gamepad1().leftStickX().get() !=0 ? -Gamepads.gamepad1().leftStickX().get() : -Gamepads.gamepad2().leftStickX().get() * 0.5,
                () -> Gamepads.gamepad1().rightStickX().get() !=0 ? -Gamepads.gamepad1().rightStickX().get() : -Gamepads.gamepad2().rightStickX().get() * 0.5,
                !Configuration.FIELD_CENTRIC
        );

        driverControlled.schedule();

        Transfer.INSTANCE.intake().schedule();
        Shooter.INSTANCE.on().schedule();

        /// ---- Gamepad 1 & 2 (Driver) ---- ///

        // Back → Emergency Stop (toggle)
        button(() -> gamepad1.back || gamepad2.back)
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

        // B (Red) → Red Alliance Override
        button(() -> gamepad1.b)
                .whenTrue(() -> {
                    Configuration.ALLIANCE = Configuration.Alliance.RED;
                });

        // Left Dpad → Relocalize Tiny Triangle
        button(() -> gamepad1.dpad_left)
                .whenTrue(() -> PedroComponent.follower().setPose(
                        new Pose(72, 7.5, Math.toRadians(90))
//                        Configuration.ALLIANCE == Configuration.Alliance.BLUE
//                                ? Configuration.BLUE_LOCALIZATION_POSE
//                                : Configuration.RED_LOCALIZATION_POSE
                ));

        // Right Bumper → Gate Open while held, close when released
        button(() -> gamepad1.right_bumper || (gamepad2.right_bumper && gamepad1.left_bumper))
                .whenTrue(() -> {
                    Transfer.INSTANCE.openGate(1.0).schedule();
                })
                .whenBecomesFalse(() -> Transfer.INSTANCE.closeGate().schedule());

        // A → Intake Off
        button(() -> gamepad1.a)
                .whenTrue(() -> Transfer.INSTANCE.stop().schedule());

        // Y → Intake On
        button(() -> gamepad1.y)
                .whenTrue(() -> Transfer.INSTANCE.intake().schedule());
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

        telemetry.addData("X", Configuration.CURRENT_POSE.getX());
        telemetry.addData("Y", Configuration.CURRENT_POSE.getY());
        telemetry.addData("Z", Math.toDegrees(Configuration.CURRENT_POSE.getHeading()));

        BindingManager.update();
        driverControlled.setScalar(Configuration.CONTROL_SCALE);

        if (Shooter.INSTANCE.MODE == Shooter.Mode.ODOMETRY) {
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

            Configuration.TURRET_OFFSET = Configuration.ALLIANCE == Configuration.Alliance.BLUE ? -0.5 : -0.5;
            Shooter.INSTANCE.updateKinematics(
                    Shooter.INSTANCE.GOAL_DISTANCE,
                    Math.toRadians(Shooter.INSTANCE.HOOD_ANGLE)
            );

            if (Configuration.CURRENT_POSE.getY() < 36) {
//                Configuration.TURRET_OFFSET = -3.5;
                Configuration.TURRET_OFFSET = -1;
                Shooter.INSTANCE.TARGET_RPM = Shooter.artifactVelocityMStoRPM(vt) * (Configuration.RPM_MULTIPLER + 0.15);
            } else {
                if (Configuration.CURRENT_POSE.getX() >= 62.575 && Configuration.CURRENT_POSE.getY() < 101.762) {
                    Configuration.TURRET_OFFSET = 2;
                } else {
                    Configuration.TURRET_OFFSET = 0.5;
                }
                Shooter.INSTANCE.TARGET_RPM = Shooter.artifactVelocityMStoRPM(vt) * Configuration.RPM_MULTIPLER;
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
