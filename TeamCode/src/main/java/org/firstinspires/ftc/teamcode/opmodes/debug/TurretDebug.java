package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.driving.DriverControlledCommand;

import static dev.nextftc.bindings.Bindings.*;
import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Disabled
@TeleOp(name = "Turret", group = "Debug")
public class TurretDebug extends NextFTCOpMode {
    double LOOP_TIME = 0;
    ElapsedTime LOOP_TIMER = new ElapsedTime();

    private DriverControlledCommand driverControlled;

    // Hood position control
    private double hoodPosition = 0.5;
    private static final double HOOD_INCREMENT = 0.01;

    public TurretDebug() {
        addComponents(
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Turret.INSTANCE),
                new SubsystemComponent(Shooter.INSTANCE)
        );
    }

    @Override
    public void onInit() {
        Gamepads.gamepad1().leftStickY();
        Gamepads.gamepad1().leftStickX();
        Gamepads.gamepad1().rightStickX();

        PedroComponent.follower().setStartingPose(new Pose(72, 72, Math.toRadians(270)));

        // Set turret to manual mode on init (servos are enabled in Turret.initialize())
        Turret.INSTANCE.mode = Turret.Mode.manual;

        // Set shooter to odometry mode - RPM will be calculated from hood position using math model
        Shooter.INSTANCE.mode = Shooter.Mode.odometry;

        // Enable hood servos and set initial position
        Shooter.INSTANCE.hoodServo1.getServo().getController().pwmEnable();
        Shooter.INSTANCE.hoodServo2.getServo().getController().pwmEnable();
        Shooter.INSTANCE.hoodServo.setPosition(hoodPosition);
        Shooter.INSTANCE.HOOD_POSITION = hoodPosition;
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

        // Start turret servos
        Turret.INSTANCE.start().schedule();

        // Enable shooter motors
        Shooter.INSTANCE.flywheelMotor1.getMotor().setMotorEnable();
        Shooter.INSTANCE.flywheelMotor2.getMotor().setMotorEnable();

        // Toggle between manual and odometry mode for turret
        button(() -> gamepad1.left_bumper)
                .toggleOnBecomesFalse()
                .whenBecomesTrue(() -> Turret.INSTANCE.changeToManual().schedule())
                .whenBecomesFalse(() -> Turret.INSTANCE.changeToAuto().schedule());

        // Turret angle control - D-Pad Right/Left
        button(() -> gamepad1.dpad_right)
                .whenBecomesTrue(() -> Turret.INSTANCE.increaseAngle().schedule());

        button(() -> gamepad1.dpad_left)
                .whenBecomesTrue(() -> Turret.INSTANCE.decreaseAngle().schedule());

        // Hood position control - D-Pad Up/Down
        button(() -> gamepad1.dpad_up)
                .whenBecomesTrue(() -> {
                    hoodPosition = Math.min(0.750, hoodPosition + HOOD_INCREMENT);
                    Shooter.INSTANCE.hoodServo.setPosition(hoodPosition);
                    Shooter.INSTANCE.HOOD_POSITION = hoodPosition;
                });

        button(() -> gamepad1.dpad_down)
                .whenBecomesTrue(() -> {
                    hoodPosition = Math.max(0.246, hoodPosition - HOOD_INCREMENT);
                    Shooter.INSTANCE.hoodServo.setPosition(hoodPosition);
                    Shooter.INSTANCE.HOOD_POSITION = hoodPosition;
                });

        // Emergency stop
        button(() -> gamepad1.a)
                .whenBecomesTrue(() -> Turret.INSTANCE.emergencyStop().schedule());
    }

    @Override
    public void onUpdate() {
        LOOP_TIMER.reset();

        // Update current pose from follower for odometry tracking

        // Loop timing
        telemetry.addData("Loop Time (ms)", LOOP_TIME);
        telemetry.addData("Loop Time (hz)", (1000/LOOP_TIME));

        Configuration.CURRENT_POSE = PedroComponent.follower().getPose();

        // Turret telemetry
        telemetry.addLine();
        telemetry.addData("Turret Mode", Turret.INSTANCE.mode);
        telemetry.addData("Turret Angle", Turret.INSTANCE.TURRET_ANGLE);
        telemetry.addData("Turret Position", Turret.INSTANCE.TURRET_POSITION);
        telemetry.addData("Manual Angle", Turret.INSTANCE.TURRET_ANGLE);
        telemetry.addData("Target Degree", Turret.INSTANCE.TARGET_DEGREE);
        telemetry.addData("Heading Degree", Turret.INSTANCE.HEADING_DEGREE);
        telemetry.addData("ODO Target", Turret.INSTANCE.ODO_TARGET);

        // Position and distance
        telemetry.addLine();
        telemetry.addData("Position", Configuration.CURRENT_POSE);
        telemetry.addData("Distance to Goal (in)", Shooter.INSTANCE.GOAL_DISTANCE);

        // Hood telemetry
        telemetry.addLine();
        telemetry.addData("=== Hood ===", "");
        telemetry.addData("Hood Position", hoodPosition);
        telemetry.addData("Hood Angle (deg)", Shooter.INSTANCE.HOOD_ANGLE);

        // Shooter RPM telemetry
        telemetry.addLine();
        telemetry.addData("=== Shooter (Auto RPM) ===", "");
        telemetry.addData("Calculated Target RPM", Shooter.INSTANCE.FLYWHEEL_RPM_GOAL);
        double m1VelocityTicks = Math.abs(Shooter.INSTANCE.flywheelMotor1.getMotor().getVelocity());
        double m2VelocityTicks = Math.abs(Shooter.INSTANCE.flywheelMotor2.getMotor().getVelocity());
//        telemetry.addData("M1 Actual RPM", Shooter.velocityToRPM(m1VelocityTicks));
//        telemetry.addData("M2 Actual RPM", Shooter.velocityToRPM(m2VelocityTicks));

        // Servo telemetry
        telemetry.addLine();
        telemetry.addData("Turret Servo 1 Position", Turret.INSTANCE.turretServo1.getPosition());
        telemetry.addData("Turret Servo 2 Position", Turret.INSTANCE.turretServo2.getPosition());

        // Controls help
        telemetry.addLine();
        telemetry.addLine("--- Controls ---");
        telemetry.addData("Left Bumper", "Toggle Manual/Auto Turret Mode");
        telemetry.addData("D-Pad Right/Left", "Turret Angle +/-");
        telemetry.addData("D-Pad Up/Down", "Hood Position +/-");
        telemetry.addData("A Button", "Emergency Stop");

        telemetry.update();

        LOOP_TIME = LOOP_TIMER.milliseconds();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
    }
}
