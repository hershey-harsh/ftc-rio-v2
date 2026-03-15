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

    private ElapsedTime ROBOT_TIMER = new ElapsedTime();
    private ElapsedTime TURRET_TIMER = new ElapsedTime();

    private double ROBOT_COLOR = WHITE, ROBOT_INTERVAL = 0;
    private boolean ROBOT_BLINK_ON = true;
    private int ROBOT_BLINK_COUNT = 0, ROBOT_BLINK_AMOUNT = -1;

    private double TURRET_COLOR = WHITE, TURRET_INTERVAL = 0;
    private boolean TURRET_BLINK_ON = true;
    private int TURRET_BLINK_COUNT = 0, TURRET_BLINK_AMOUNT = -1;

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

        ROBOT_COLOR = GREEN;
        ROBOT_INTERVAL = 0;
        ROBOT_BLINK_ON = true;
        ROBOT_BLINK_COUNT = 0;
        ROBOT_BLINK_AMOUNT = -1;

        TURRET_COLOR = GREEN;
        TURRET_INTERVAL = 0;
        TURRET_BLINK_ON = true;
        TURRET_BLINK_COUNT = 0;
        TURRET_BLINK_AMOUNT = -1;

        robotLight.getServo().setPosition(GREEN);
        turretLight.getServo().setPosition(GREEN);
    }

    @Override
    public void periodic() {
        if (ROBOT_INTERVAL > 0 && ROBOT_TIMER.milliseconds() >= ROBOT_INTERVAL) {
            ROBOT_BLINK_ON = !ROBOT_BLINK_ON;
            if (!ROBOT_BLINK_ON) ROBOT_BLINK_COUNT++;
            ROBOT_TIMER.reset();

            if (ROBOT_BLINK_AMOUNT > 0 && ROBOT_BLINK_COUNT >= ROBOT_BLINK_AMOUNT) {
                ROBOT_INTERVAL = 0;
                ROBOT_BLINK_ON = false;
            }
        }
        robotLight.setPosition(ROBOT_BLINK_ON ? ROBOT_COLOR : OFF);

        if (TURRET_INTERVAL > 0 && TURRET_TIMER.milliseconds() >= TURRET_INTERVAL) {
            TURRET_BLINK_ON = !TURRET_BLINK_ON;
            if (!TURRET_BLINK_ON) TURRET_BLINK_COUNT++;
            TURRET_TIMER.reset();

            if (TURRET_BLINK_AMOUNT > 0 && TURRET_BLINK_COUNT >= TURRET_BLINK_AMOUNT) {
                TURRET_INTERVAL = 0;
                TURRET_BLINK_ON = false;
            }
        }
        turretLight.setPosition(TURRET_BLINK_ON ? TURRET_COLOR : OFF);
    }

    public Command setColor(double color, Target target) {
        return new InstantCommand(() -> {
            if (target == Target.ROBOT || target == Target.BOTH) {
                this.ROBOT_COLOR = color;
                this.ROBOT_INTERVAL = 0;
                this.ROBOT_BLINK_ON = true;
                this.ROBOT_BLINK_COUNT = 0;
                this.ROBOT_BLINK_AMOUNT = -1;
            }
            if (target == Target.TURRET || target == Target.BOTH) {
                this.TURRET_COLOR = color;
                this.TURRET_INTERVAL = 0;
                this.TURRET_BLINK_ON = true;
                this.TURRET_BLINK_COUNT = 0;
                this.TURRET_BLINK_AMOUNT = -1;
            }
        });
    }

    public Command setColor(double color) {
        return setColor(color, Target.BOTH);
    }

    public Command setBlinkingColor(double color, double intervalMs, int amount, Target target) {
        return new InstantCommand(() -> {
            if (target == Target.ROBOT || target == Target.BOTH) {
                this.ROBOT_COLOR = color;
                this.ROBOT_INTERVAL = intervalMs;
                this.ROBOT_BLINK_ON = true;
                this.ROBOT_BLINK_COUNT = 0;
                this.ROBOT_BLINK_AMOUNT = amount;
                ROBOT_TIMER.reset();
            }
            if (target == Target.TURRET || target == Target.BOTH) {
                this.TURRET_COLOR = color;
                this.TURRET_INTERVAL = intervalMs;
                this.TURRET_BLINK_ON = true;
                this.TURRET_BLINK_COUNT = 0;
                this.TURRET_BLINK_AMOUNT = amount;
                TURRET_TIMER.reset();
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
