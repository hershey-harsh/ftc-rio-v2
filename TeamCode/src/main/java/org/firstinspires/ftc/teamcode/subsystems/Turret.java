package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.ServoGroup;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Configuration;


public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();

    public static double ANGLE_INCREMENT = 1.0;
    public static double BACKLASH_COMP_DEGREES = 2; // pre-load gears to take up backlash slack

    public ServoEx turretServo1;
    public ServoEx turretServo2;
    public ServoGroup turretServo;

    public double HEADING_DEGREE = 0, TARGET_DEGREE = 0, ODO_TARGET = 0, TURRET_POSITION = 0, TURRET_ANGLE = 0, TRUE_TARGET_DEGREE, ERROR = 0;
    public boolean TURRET_IN_RANGE = false;


    public Mode mode = Mode.odometry;

    public enum Mode {
        manual,
        odometry,
    }

    @Override
    public void initialize() {
        turretServo1 = new ServoEx(ActiveOpMode.hardwareMap().get(Servo.class, Configuration.TURRET_SERVO_LEFT));
        turretServo2 = new ServoEx(ActiveOpMode.hardwareMap().get(Servo.class, Configuration.TURRET_SERVO_RIGHT));
        turretServo = new ServoGroup(turretServo1, turretServo2);

        turretServo1.getServo().getController().pwmEnable();
        turretServo2.getServo().getController().pwmEnable();

        mode = Mode.odometry;
        setAngle(0);
    }

    @Override
    public void periodic() {
        Pose pose = Configuration.CURRENT_POSE;
        HEADING_DEGREE = Math.toDegrees(pose.getHeading());

        if (Configuration.ALLIANCE == Configuration.Alliance.RED) {
            ODO_TARGET = Math.atan2(
                    ((Configuration.RED_GOAL_POSE.getY() + Configuration.Y_GOAL_OFFSET) - pose.getY()),
                    ((Configuration.RED_GOAL_POSE.getX() + Configuration.X_GOAL_OFFSET) - pose.getX())
            );
        } else {
            ODO_TARGET = Math.atan2(
                    ((Configuration.BLUE_GOAL_POSE.getY() + Configuration.Y_GOAL_OFFSET) - pose.getY()),
                    ((Configuration.BLUE_GOAL_POSE.getX() + Configuration.X_GOAL_OFFSET) - pose.getX())
            );
        }

        if (mode == Mode.odometry) {
            double newTarget = Math.toDegrees(ODO_TARGET) - HEADING_DEGREE;
            TARGET_DEGREE = newTarget;
            TRUE_TARGET_DEGREE = newTarget;
            setAngle(TARGET_DEGREE + Configuration.TURRET_OFFSET);
        } else if (mode == Mode.manual) {
            setAngle(TARGET_DEGREE + Configuration.TURRET_OFFSET);
        }
    }

    private void setAngle(double angle) {
        TURRET_ANGLE = angle;
        TURRET_POSITION = interpolateAngle(angle);
        turretServo.setPosition(TURRET_POSITION);
    }

    private double interpolateAngle(double angle) {
        angle = AngleUnit.normalizeDegrees(angle);
        if (angle > 165) {
            angle = 165;
        } else if (angle < -155) {
            angle = -155;
        }

        double result = 0.5 - (angle * 0.25) / 90;

        if (result > 0.8) {
            result = 0.8;
            TURRET_IN_RANGE = false;
        } else if (result < 0.2) {
            TURRET_IN_RANGE = false;
            result = 0.2;
        } else {
            TURRET_IN_RANGE = true;
        }

        return result;
    }

    public Command start() {
        return new InstantCommand(() -> {
            turretServo1.getServo().getController().pwmEnable();
            turretServo2.getServo().getController().pwmEnable();
            setAngle(0);
        });
    }

    public Command emergencyStop() {
        return new InstantCommand(() -> {
            turretServo1.getServo().getController().pwmDisable();
            turretServo2.getServo().getController().pwmDisable();
        });
    }

    public Command changeToManual() {
        return new InstantCommand(() -> {
            mode = Mode.manual;
            TARGET_DEGREE = 0;
        });
    }

    public Command changeToAuto() {
        return new InstantCommand(() -> {
            mode = Mode.odometry;
        });
    }

    public Command increaseAngle() {
        return new InstantCommand(() -> {
            TARGET_DEGREE = TARGET_DEGREE + ANGLE_INCREMENT;
            if (TARGET_DEGREE > 165) {
                TARGET_DEGREE = 165;
            } else if (TARGET_DEGREE < -155) {
                TARGET_DEGREE = -155;
            }
        });
    }

    public Command decreaseAngle() {
        return new InstantCommand(() -> {
            TARGET_DEGREE = TARGET_DEGREE - ANGLE_INCREMENT;
            if (TARGET_DEGREE > 165) {
                TARGET_DEGREE = 165;
            } else if (TARGET_DEGREE < -155) {
                TARGET_DEGREE = -155;
            }
        });
    }

    public Command resetAngle() {
        return new InstantCommand(() -> {
            TARGET_DEGREE = 0;
        });
    }

}
