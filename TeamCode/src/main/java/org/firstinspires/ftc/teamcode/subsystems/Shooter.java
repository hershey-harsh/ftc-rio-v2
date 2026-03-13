package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Map;
import java.util.TreeMap;

import org.firstinspires.ftc.teamcode.Configuration;

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
    public static BasicFeedforwardParameters ffcoefficients = new BasicFeedforwardParameters(0.00015, 0.0, 0.0);

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
    public double READ_RPM = 0;

    public double HOOD_ANGLE = 70;
    public double HOOD_POSITION = 0.1;
    public double MIN_HOOD_ANGLE = 43.0;
    public double MAX_HOOD_ANGLE = 70.0;

    public Mode mode = Mode.odometry;

    public double GOAL_DISTANCE = 0;
    private boolean started = false;

    public enum Mode {
        manual,
        odometry,
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
        mode = Mode.manual;
        TARGET_RPM = 0;

//        velController = new PIDController(kP, kI, kD);
//        voltageSensor = ActiveOpMode.hardwareMap().voltageSensor.iterator().next();
    }

    @Override
    public void periodic() {
        if (!started) return;

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

        if (mode == Mode.odometry) {
            HOOD_ANGLE = getHoodAngle(GOAL_DISTANCE);
            setHoodAngle(HOOD_ANGLE);
        }

        double targetVelocity = rpmToVelocity(TARGET_RPM);
        controlSystem.setGoal(new KineticState(0.0, targetVelocity));
        flywheelMotor.setPower(controlSystem.calculate(flywheelMotor.getState()));

        double m1Ticks = Math.abs(flywheelMotor1.getMotor().getVelocity());
        double m2Ticks = Math.abs(flywheelMotor2.getMotor().getVelocity());
        READ_RPM = velocityToRPM((m1Ticks + m2Ticks) / 2.0);
    }

    public static class ShotParameters {
        public final double DISTANCE;
        public final double ARTIFACT_VELOCITY;
        public final double HOOD_ANGLE;
        public final double TIME_OF_FLIGHT;

        public ShotParameters(double DISTANCE, double ARTIFACT_VELOCITY, double HOOD_ANGLE, double TIME_OF_FLIGHT) {
            this.DISTANCE = DISTANCE;
            this.ARTIFACT_VELOCITY = ARTIFACT_VELOCITY;
            this.HOOD_ANGLE = HOOD_ANGLE;
            this.TIME_OF_FLIGHT = TIME_OF_FLIGHT;
        }
    }

    private static final TreeMap<Double, ShotParameters> SHOOTER_LOOKUP = new TreeMap<>();
    static {
        //                                               Distance (in -> m)                 Velocity M/S               HoodAngle             ToF
        SHOOTER_LOOKUP.put(42.12 * 0.0254,  new ShotParameters(42.12 * 0.0254,  rpmToArtifactMSVelocity(2600), 70, 0.6));
        SHOOTER_LOOKUP.put(68.63 * 0.0254,  new ShotParameters(68.63 * 0.0254,  rpmToArtifactMSVelocity(3000), 50, 0.55));
        SHOOTER_LOOKUP.put(75.54 * 0.0254,  new ShotParameters(75.54 * 0.0254,  rpmToArtifactMSVelocity(3200), 44, 0.48));
    }

    public static ShotParameters getShooterLUT(double distInches) {
        if (SHOOTER_LOOKUP.isEmpty()) return new ShotParameters(distInches, 0, 0.5, 0.4);

        Map.Entry<Double, ShotParameters> lo = SHOOTER_LOOKUP.floorEntry(distInches);
        Map.Entry<Double, ShotParameters> hi = SHOOTER_LOOKUP.ceilingEntry(distInches);

        if (lo == null) return hi != null ? hi.getValue() : new ShotParameters(distInches, 0, 0.5, 0.4);
        if (hi == null) return lo.getValue();
        if (lo.getKey().equals(hi.getKey())) return lo.getValue();

        double t = (distInches - lo.getKey()) / (hi.getKey() - lo.getKey());
        ShotParameters a = lo.getValue(), b = hi.getValue();
        return new ShotParameters(
                distInches,
                a.ARTIFACT_VELOCITY  + t * (b.ARTIFACT_VELOCITY  - a.ARTIFACT_VELOCITY),
                a.HOOD_ANGLE + t * (b.HOOD_ANGLE - a.HOOD_ANGLE),
                a.TIME_OF_FLIGHT       + t * (b.TIME_OF_FLIGHT       - a.TIME_OF_FLIGHT)
        );
    }

    private double a, b, n, t_u, t_g, tof, vX, vY, v, m;
    public double kinematicRPMGoal = 0;

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

        kinematicRPMGoal = (v / (2 * Math.PI * 0.036)) * 60;
    }

    public double shooterVKinematic() { return v; }
    public double getTof() { return tof; }
    public double getKinematicRPMGoal() { return kinematicRPMGoal; }

    public static ShotParameters getShooterValues(double GOAL_DISTANCE) {
        return getShooterLUT(GOAL_DISTANCE);
    }

    public static double rpmToArtifactMSVelocity(double rpm) {
        return (rpm / 60.0) * (2 * Math.PI * 0.036);
    }

    public static double artifactVelocityMStoRPM(double velocity) {
        return (velocity / (2 * Math.PI * 0.036)) * 60;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }

    public static double velocityToRPM(double velocity) {
        return (velocity / TICKS_PER_REV) * 60.0;
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
            mode = Mode.odometry;
        });
    }

    public Command off() {
        return new InstantCommand(() -> {
            mode = Mode.manual;
            stopShooter();
        });
    }

    public Command start() {
        return new InstantCommand(() -> {
            started = true;
            mode = Mode.manual;
            hoodServo1.getServo().getController().pwmEnable();
            hoodServo2.getServo().getController().pwmEnable();
            flywheelMotor1.getMotor().setMotorEnable();
            flywheelMotor2.getMotor().setMotorEnable();
        });
    }

    public Command emergencyStop() {
        return new InstantCommand(() -> {
            mode = Mode.manual;
            hoodServo1.getServo().getController().pwmDisable();
            hoodServo2.getServo().getController().pwmDisable();
            flywheelMotor1.getMotor().setMotorDisable();
            flywheelMotor2.getMotor().setMotorDisable();
        });
    }

    public Command changeToManual() {
        return new InstantCommand(() -> mode = Mode.manual);
    }

    public Command changeToAuto() {
        return new InstantCommand(() -> mode = Mode.odometry);
    }

    public Command increaseHood() {
        return new InstantCommand(() -> {
            if (mode != Mode.manual) return;
            double newAngle = Math.min(HOOD_ANGLE + HOOD_INCREMENT, MAX_HOOD_ANGLE);
            setHoodAngle(newAngle);
        });
    }

    public Command decreaseHood() {
        return new InstantCommand(() -> {
            if (mode != Mode.manual) return;
            double newAngle = Math.max(HOOD_ANGLE - HOOD_INCREMENT, MIN_HOOD_ANGLE);
            setHoodAngle(newAngle);
        });
    }

    public Command adjustShooterRPM(double stickValue) {
        return new InstantCommand(() -> {
            if (mode != Mode.manual) return;
            TARGET_RPM = Math.max(0, TARGET_RPM + (stickValue * RPM_INCREMENT));
        });
    }
}

