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

    // Turret servo travel limits — THESE set the usable aim range. With the 0.25/90 deg→servo
    // scale: [0.2, 0.8] = ±108°, [0.1, 0.9] = ±144°, [0.04, 0.93] ≈ the ±155-165° hard limits
    // declared by the angle clamp in interpolateAngle(). The turret "clamps to one side / stops
    // following" when the goal swings past this range (e.g. goal behind the robot). Default is
    // widened to ±144° (well inside the declared mechanical limit) so it tracks across far more
    // of the field. If the servo buzzes/strains at the ends, narrow these (live via SOTM Tuner);
    // if your turret can safely go further, widen toward [0.04, 0.93]. Tune ON the robot.
    public static double MIN_SERVO_POS = 0.1;
    public static double MAX_SERVO_POS = 0.9;

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
        Configuration.TURRET_PREDICTIVE_OFFSET = 0;
        Configuration.TURRET_ZONE_OFFSET = 0;
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
            // Mechanical offset + predictive rotational lead + zone offset (set by AimController).
            setAngle(TARGET_DEGREE + Configuration.TURRET_OFFSET
                    + Configuration.TURRET_PREDICTIVE_OFFSET + Configuration.TURRET_ZONE_OFFSET);
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

        if (result > MAX_SERVO_POS) {
            result = MAX_SERVO_POS;
            TURRET_IN_RANGE = false;
        } else if (result < MIN_SERVO_POS) {
            TURRET_IN_RANGE = false;
            result = MIN_SERVO_POS;
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