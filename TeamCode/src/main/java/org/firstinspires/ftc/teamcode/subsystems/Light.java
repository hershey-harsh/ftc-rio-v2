package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.ServoEx;

import org.firstinspires.ftc.teamcode.Configuration;

public class Light implements Subsystem {
    public static final Light INSTANCE = new Light();

    private ServoEx robotLight;
    private ServoEx turretLight;
    private ElapsedTime robotTimer = new ElapsedTime();
    private ElapsedTime turretTimer = new ElapsedTime();

    private double robotColor = WHITE, robotInterval = 0;
    private boolean robotBlinkOn = true;
    private int robotBlinkCount = 0, robotBlinkAmount = -1;

    private double turretColor = WHITE, turretInterval = 0;
    private boolean turretBlinkOn = true;
    private int turretBlinkCount = 0, turretBlinkAmount = -1;

    public static final double OFF = 0.0, RED = 0.277, ORANGE = 0.333, YELLOW = 0.388;
    public static final double SAGE = 0.444, GREEN = 0.5, AZURE = 0.555, BLUE = 0.611;
    public static final double INDIGO = 0.666, VIOLET = 0.722, WHITE = 1.0;

    public enum Target {
        ROBOT,
        TURRET,
        BOTH
    }

    private Light() {}

    @Override
    public void initialize() {
        robotLight = new ServoEx(ActiveOpMode.hardwareMap().get(Servo.class, Configuration.ROBOT_LIGHT));
        turretLight = new ServoEx(ActiveOpMode.hardwareMap().get(Servo.class, Configuration.TURRET_LIGHT));

        robotColor = WHITE;
        robotInterval = 0;
        robotBlinkOn = true;
        robotBlinkCount = 0;
        robotBlinkAmount = -1;

        turretColor = WHITE;
        turretInterval = 0;
        turretBlinkOn = true;
        turretBlinkCount = 0;
        turretBlinkAmount = -1;

        robotLight.getServo().setPosition(GREEN);
        turretLight.getServo().setPosition(GREEN);
    }

    @Override
    public void periodic() {
        if (robotInterval > 0 && robotTimer.milliseconds() >= robotInterval) {
            robotBlinkOn = !robotBlinkOn;
            if (!robotBlinkOn) robotBlinkCount++;
            robotTimer.reset();

            if (robotBlinkAmount > 0 && robotBlinkCount >= robotBlinkAmount) {
                robotInterval = 0;
                robotBlinkOn = false;
            }
        }
        robotLight.setPosition(robotBlinkOn ? robotColor : OFF);

        if (turretInterval > 0 && turretTimer.milliseconds() >= turretInterval) {
            turretBlinkOn = !turretBlinkOn;
            if (!turretBlinkOn) turretBlinkCount++;
            turretTimer.reset();

            if (turretBlinkAmount > 0 && turretBlinkCount >= turretBlinkAmount) {
                turretInterval = 0;
                turretBlinkOn = false;
            }
        }
        turretLight.setPosition(turretBlinkOn ? turretColor : OFF);
    }

    public Command setColor(double color, Target target) {
        return new InstantCommand(() -> {
            if (target == Target.ROBOT || target == Target.BOTH) {
                this.robotColor = color;
                this.robotInterval = 0;
                this.robotBlinkOn = true;
                this.robotBlinkCount = 0;
                this.robotBlinkAmount = -1;
            }
            if (target == Target.TURRET || target == Target.BOTH) {
                this.turretColor = color;
                this.turretInterval = 0;
                this.turretBlinkOn = true;
                this.turretBlinkCount = 0;
                this.turretBlinkAmount = -1;
            }
        });
    }

    public Command setColor(double color) {
        return setColor(color, Target.BOTH);
    }

    public Command setBlinkingColor(double color, double intervalMs, int amount, Target target) {
        return new InstantCommand(() -> {
            if (target == Target.ROBOT || target == Target.BOTH) {
                this.robotColor = color;
                this.robotInterval = intervalMs;
                this.robotBlinkOn = true;
                this.robotBlinkCount = 0;
                this.robotBlinkAmount = amount;
                robotTimer.reset();
            }
            if (target == Target.TURRET || target == Target.BOTH) {
                this.turretColor = color;
                this.turretInterval = intervalMs;
                this.turretBlinkOn = true;
                this.turretBlinkCount = 0;
                this.turretBlinkAmount = amount;
                turretTimer.reset();
            }
        });
    }

    public Command setBlinkingColor(double color, double intervalMs, int amount) {
        return setBlinkingColor(color, intervalMs, amount, Target.BOTH);
    }

    public Command setBlinkingColor(double color, double intervalMs, Target target) {
        return setBlinkingColor(color, intervalMs, -1, target);
    }

    public Command setBlinkingColor(double color, double intervalMs) {
        return setBlinkingColor(color, intervalMs, -1, Target.BOTH);
    }

    public Command setBlinkingColor(double color, Target target) {
        return setBlinkingColor(color, 500, -1, target);
    }

    public Command setBlinkingColor(double color) {
        return setBlinkingColor(color, 500, -1, Target.BOTH);
    }

    public Command off(Target target) {
        return setColor(OFF, target);
    }

    public Command off() {
        return setColor(OFF, Target.BOTH);
    }
}
