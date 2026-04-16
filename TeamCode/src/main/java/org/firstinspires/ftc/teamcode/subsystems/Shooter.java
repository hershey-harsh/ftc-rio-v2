package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Configuration;

import java.util.Locale;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.ServoGroup;

public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();

    public static final double TICKS_PER_REV = 28.0;

    public static PIDCoefficients coefficients = new PIDCoefficients(0.0125, 0.0, 0.0001);
    public static BasicFeedforwardParameters ffcoefficients = new BasicFeedforwardParameters(0.00015, 0.00005, 0.0);

    public static double HOOD_INCREMENT = 0.05;
    public static double RPM_INCREMENT = 100.0;

    public MotorEx flywheelMotor1;
    public MotorEx flywheelMotor2;
    public MotorGroup flywheelMotor;

    private ControlSystem controlSystem;

    public ServoEx hoodServo1;
    public ServoEx hoodServo2;
    public ServoGroup hoodServo;

    public double FLYWHEEL_RPM_GOAL = 0;
    public double TARGET_RPM = 0;
    public double CURRENT_RPM = 0;

    public double HOOD_ANGLE = 70;
    public double HOOD_POSITION = 0.1;
    public double MIN_HOOD_ANGLE = 43.0;
    public double MAX_HOOD_ANGLE = 70.0;

    public Mode MODE = Mode.ODOMETRY, LAST_MODE = null;
    public boolean DEBUG_TELEMETRY = false;

    public double GOAL_DISTANCE = 0;
    private boolean started = false;

    public enum Mode {
        MANUAL,
        ODOMETRY,
    }

    @Override
    public void initialize() {
        flywheelMotor1 = new MotorEx(ActiveOpMode.hardwareMap().get(DcMotorEx.class, Configuration.RIGHT_TURRET_MOTOR));
        flywheelMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor1.getMotor().setDirection(DcMotorEx.Direction.REVERSE);
        flywheelMotor2 = new MotorEx(ActiveOpMode.hardwareMap().get(DcMotorEx.class, Configuration.LEFT_TURRET_MOTOR));
        flywheelMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor2.getMotor().setDirection(DcMotorEx.Direction.FORWARD);

        flywheelMotor = new MotorGroup(flywheelMotor1, flywheelMotor2);

        hoodServo1 = new ServoEx(ActiveOpMode.hardwareMap().get(Servo.class, Configuration.HOOD_SERVO_LEFT));
        hoodServo2 = new ServoEx(ActiveOpMode.hardwareMap().get(Servo.class, Configuration.HOOD_SERVO_RIGHT));
        hoodServo1.getServo().setDirection(Servo.Direction.REVERSE);
        hoodServo = new ServoGroup(hoodServo1, hoodServo2);

        controlSystem = ControlSystem.builder()
                .velPid(coefficients)
                .basicFF(ffcoefficients)
                .build();

        started = false;
        MODE = Mode.MANUAL;
        LAST_MODE = null;
        TARGET_RPM = 0;
    }

    @Override
    public void periodic() {
        if (!started) return;

        Telemetry t = ActiveOpMode.telemetry();
        Pose pose = Configuration.CURRENT_POSE;

        if (Configuration.ALLIANCE == Configuration.Alliance.RED) {
            GOAL_DISTANCE = (Math.hypot(
                    (Configuration.RED_GOAL_POSE.getX() + Configuration.X_GOAL_OFFSET) - pose.getX(),
                    (Configuration.RED_GOAL_POSE.getY() + Configuration.Y_GOAL_OFFSET) - pose.getY())) * 0.0254;
        } else {
            GOAL_DISTANCE = (Math.hypot(
                    (Configuration.BLUE_GOAL_POSE.getX() + Configuration.X_GOAL_OFFSET) - pose.getX(),
                    (Configuration.BLUE_GOAL_POSE.getY() + Configuration.Y_GOAL_OFFSET) - pose.getY())) * 0.0254;
        }

        if (MODE != LAST_MODE) {
            switch (MODE) {
                case MANUAL:
                    break;
                case ODOMETRY:
                    break;
            }
            LAST_MODE = MODE;
        }

        if (MODE == Mode.ODOMETRY) {
            HOOD_ANGLE = getHoodAngle(GOAL_DISTANCE);
            if (Configuration.CURRENT_POSE.getY() < 36) {
                Shooter.INSTANCE.setHoodAngle(43);
            } else {
                Shooter.INSTANCE.setHoodAngle(HOOD_ANGLE);
            }
        }

        double targetVelocity = rpmToVelocity(TARGET_RPM);
        controlSystem.setGoal(new KineticState(0.0, targetVelocity));
        flywheelMotor.setPower(controlSystem.calculate(flywheelMotor.getState()));

        double currentRPM = velocityToRPM(flywheelMotor.getState().getVelocity());
        CURRENT_RPM = currentRPM;

        t.addLine();
        t.addData("----- Shooter Status -----", "");
        t.addData("Shooter Mode", MODE.name());
        t.addData("Target RPM", String.format(Locale.US, "%.0f", TARGET_RPM));
        t.addData("Current RPM", String.format(Locale.US, "%.0f", currentRPM));
        t.addData("Up To Speed", isUpToSpeed());

        if (DEBUG_TELEMETRY) {
            t.addLine();
            t.addData("----- Shooter Debug -----", "");
            t.addData("Goal Distance (m)", String.format(Locale.US, "%.3f", GOAL_DISTANCE));
            t.addData("Hood Angle (deg)", String.format(Locale.US, "%.1f", HOOD_ANGLE));
            t.addData("Hood Position", String.format(Locale.US, "%.3f", HOOD_POSITION));
            t.addData("RPM Error", String.format(Locale.US, "%.0f", TARGET_RPM - currentRPM));
            t.addData("RPM Tolerance", String.format(Locale.US, "%.0f", RPM_TOLERANCE));
            t.addData("Kinematic RPM Goal", String.format(Locale.US, "%.0f", KINEMATIC_RPM_GOAL));
            t.addData("Kinematic Velocity (m/s)", String.format(Locale.US, "%.3f", v));
            t.addData("Time of Flight (s)", String.format(Locale.US, "%.3f", tof));
            t.addData("Flywheel Started", started);
        }
    }

    private double a, b, n, t_u, t_g, tof, vX, vY, v, m;
    public double KINEMATIC_RPM_GOAL = 0;

    public double getHoodAngle(double meters) {
//        return Math.max((-4.8701 * meters) + 59.754, 43);
//        return Math.max((-9.1295 * meters) + 77.73622, 43);
        return Math.max((-5.8282 * meters) + 64.82552, 43);
    }

    public void updateKinematics(double distMeters, double hoodAngleRad) {
        a = (-distMeters * Math.tan(hoodAngleRad) + Configuration.SHOOTER_HEIGHT_TO_GOAL) / (distMeters * distMeters);
        b = -Math.tan(hoodAngleRad) - (2 * a * distMeters);
        n = -b / (2 * a);
        m = (a * (n * n)) + (b * n) + Configuration.SHOOTER_HEIGHT_TO_GOAL;

        t_u = Math.sqrt((2 * m) / 9.8);
        t_g = Math.sqrt((2 * (m - Configuration.SHOOTER_HEIGHT_TO_GOAL)) / 9.8);
        tof = t_u + t_g;

        vX = distMeters / tof;
        vY = (m - 0.5 * (-9.8) * (t_u * t_u)) / t_u;

        v = Math.sqrt((vX * vX) + (vY * vY));

        KINEMATIC_RPM_GOAL = (v / (2 * Math.PI * 0.036)) * 60;
    }

    public double shooterVKinematic() { return v; }
    public double getTof() { return tof; }
    public double getKinematicRPMGoal() { return KINEMATIC_RPM_GOAL; }
    public static double rpmToArtifactMSVelocity(double rpm) { return (rpm / 60.0) * (2 * Math.PI * 0.036); }
    public static double artifactVelocityMStoRPM(double velocity) { return (velocity / (2 * Math.PI * 0.036)) * 60; }
    public static double rpmToVelocity(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }
    public static double velocityToRPM(double velocity) { return (velocity / TICKS_PER_REV) * 60.0; }

    public static double RPM_TOLERANCE = 400;

    public boolean isUpToSpeed() {
        return true; // Override to always return true for testing without actual hardware
//        if (TARGET_RPM <= 0) return false;
//        double currentRPM = velocityToRPM(flywheelMotor.getState().getVelocity());
//        return Math.abs(TARGET_RPM - currentRPM) <= RPM_TOLERANCE;
    }

    public void stopShooter() {
        flywheelMotor1.getMotor().setPower(0);
        flywheelMotor2.getMotor().setPower(0);
    }

    public void setHoodAngle(double degrees) {
        if (degrees >= 70) {
            degrees = 70;
        } else if (degrees <= 43) {
            degrees = 43;
        }
        HOOD_ANGLE = degrees;
        HOOD_POSITION = 0.1 + (70 - degrees) * (0.9 / 27.0);
        hoodServo.setPosition(HOOD_POSITION);
    }

    public Command on() {
        return new InstantCommand(() -> {
            started = true;
            flywheelMotor1.getMotor().setMotorEnable();
            flywheelMotor2.getMotor().setMotorEnable();
            MODE = Mode.ODOMETRY;
        });
    }

    public Command off() {
        return new InstantCommand(() -> {
            MODE = Mode.MANUAL;
            stopShooter();
        });
    }

    public Command start() {
        return new InstantCommand(() -> {
            started = true;
            MODE = Mode.MANUAL;
            hoodServo1.getServo().getController().pwmEnable();
            hoodServo2.getServo().getController().pwmEnable();
            flywheelMotor1.getMotor().setMotorEnable();
            flywheelMotor2.getMotor().setMotorEnable();
        });
    }

    public Command emergencyStop() {
        return new InstantCommand(() -> {
            MODE = Mode.MANUAL;
            hoodServo1.getServo().getController().pwmDisable();
            hoodServo2.getServo().getController().pwmDisable();
            flywheelMotor1.getMotor().setMotorDisable();
            flywheelMotor2.getMotor().setMotorDisable();
        });
    }

    public Command changeToManual() {
        return new InstantCommand(() -> MODE = Mode.MANUAL);
    }

    public Command changeToAuto() {
        return new InstantCommand(() -> MODE = Mode.ODOMETRY);
    }

    public Command increaseHood() {
        return new InstantCommand(() -> {
            if (MODE != Mode.MANUAL) return;
            double newAngle = Math.min(HOOD_ANGLE + HOOD_INCREMENT, MAX_HOOD_ANGLE);
            setHoodAngle(newAngle);
        });
    }

    public Command decreaseHood() {
        return new InstantCommand(() -> {
            if (MODE != Mode.MANUAL) return;
            double newAngle = Math.max(HOOD_ANGLE - HOOD_INCREMENT, MIN_HOOD_ANGLE);
            setHoodAngle(newAngle);
        });
    }

    public Command adjustShooterRPM(double stickValue) {
        return new InstantCommand(() -> {
            if (MODE != Mode.MANUAL) return;
            TARGET_RPM = Math.max(0, TARGET_RPM + (stickValue * RPM_INCREMENT));
        });
    }
}

