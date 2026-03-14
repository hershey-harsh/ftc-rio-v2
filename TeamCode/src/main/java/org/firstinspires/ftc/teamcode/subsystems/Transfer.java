package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Configuration;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.units.Distance;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.ServoGroup;


public class Transfer implements Subsystem {
    public static final Transfer INSTANCE = new Transfer();

    public static double INTAKE_POWER = 1.0;
    public static double THIRD_GATE_POWER;
    public static double POWER_INCREMENT = 0.1;
    private double CURRENT_POWER = 0;
    public static double GATE_ONE_OPEN = 0.661, GATE_ONE_CLOSED = 0.738;
    public static double GATE_TWO_OPEN = 0.388, GATE_TWO_CLOSED = 0.315;

    public MotorEx transferMotor1;
    public MotorEx transferMotor2;

    public ServoEx servoGate1;
    public ServoEx servoGate2;

    public DigitalChannel gate1And2Sensor;
    public RevColorSensorV3 gate3Sensor;
    public static double gate3Distance;

    public NormalizedRGBA colors;
    public static boolean override = false;

    // Gate 3 auto-stop
    private boolean GATE_OVERRIDE = false;       // true when openGate() is called — ignores distance
    private boolean GATE3_STOPPED = false;        // true after ball detected and motor stopped
    private boolean STARTED = false;             // true after intake() is called — prevents running during init
    private static final double GATE3_THRESHOLD = 3.0;  // cm

    // All-full auto-stop: both gate3 AND gate1And2 triggered for 3+ seconds
    private final ElapsedTime BOTH_FULL_TIMER = new ElapsedTime();
    private boolean BOTH_FULL_TIMER_RUNNING = false;
    private boolean ALL_STOPPED = false;          // true after both sensors full for 3s — all motors stopped
    private static final double ALL_FULL_TIMEOUT = 3.0; // seconds

    @Override
    public void initialize() {
        transferMotor1 = new MotorEx(ActiveOpMode.hardwareMap().get(DcMotorEx.class, Configuration.TRANSFER_MOTOR_ONE)).reversed();
        transferMotor2 = new MotorEx(ActiveOpMode.hardwareMap().get(DcMotorEx.class, Configuration.TRANSFER_MOTOR_TWO));

        servoGate1 = new ServoEx(ActiveOpMode.hardwareMap().get(Servo.class, Configuration.SERVO_GATE_LEFT));
        servoGate2 = new ServoEx(ActiveOpMode.hardwareMap().get(Servo.class, Configuration.SERVER_GATE_RIGHT));

        gate1And2Sensor = ActiveOpMode.hardwareMap().get(DigitalChannel.class, Configuration.GATE_1_AND_2_SENSOR);
        gate1And2Sensor.setMode(DigitalChannel.Mode.INPUT);
        gate3Sensor = ActiveOpMode.hardwareMap().get(RevColorSensorV3.class, Configuration.GATE_3_SENSOR);

        STARTED = false;
        GATE_OVERRIDE = false;
        GATE3_STOPPED = false;
        ALL_STOPPED = false;
        BOTH_FULL_TIMER_RUNNING = false;
        servoGate1.setPosition(GATE_ONE_CLOSED);
        servoGate2.setPosition(GATE_TWO_CLOSED);
    }

    @Override
    public void periodic() {
        gate3Distance = gate3Sensor.getDistance(DistanceUnit.CM);
        boolean GATE3_FULL = gate3Distance < GATE3_THRESHOLD;
        boolean GATE12_FULL = false; // true = ball detected

        if (!STARTED) return;
        if (GATE_OVERRIDE) return;

        // --- All-full auto-stop: both sensors full for 3+ seconds, stop motor1 (gate 1&2) ---
        // motor2 (gate 3) is already handled by gate3 auto-stop below
        if (GATE3_FULL && GATE12_FULL) {
            if (!BOTH_FULL_TIMER_RUNNING) {
                BOTH_FULL_TIMER.reset();
                BOTH_FULL_TIMER_RUNNING = true;
            }
            if (!ALL_STOPPED && BOTH_FULL_TIMER.seconds() >= ALL_FULL_TIMEOUT) {
                ALL_STOPPED = true;
                transferMotor1.setPower(0);
            }
        } else {
            BOTH_FULL_TIMER_RUNNING = false;
            if (ALL_STOPPED) {
                ALL_STOPPED = false;
                transferMotor1.setPower(-INTAKE_POWER);
            }
        }

        if (ALL_STOPPED) return;

        if (GATE3_FULL) {
            if (!GATE3_STOPPED) {
                GATE3_STOPPED = true;
                new SequentialGroup(
                        new Delay(0.5),
                        new InstantCommand(() -> {
                            if (!GATE_OVERRIDE && !ALL_STOPPED) transferMotor2.setPower(0);
                        })
                ).schedule();
            }
        } else {
            if (GATE3_STOPPED) {
                GATE3_STOPPED = false;
                transferMotor2.setPower(-INTAKE_POWER);
            }
        }
    }

    public Command intake() {
        return new InstantCommand(() -> {
            STARTED = true;
            CURRENT_POWER = -INTAKE_POWER;
            THIRD_GATE_POWER = INTAKE_POWER;
            transferMotor1.setPower(CURRENT_POWER);
            transferMotor2.setPower(-THIRD_GATE_POWER);
        });
    }

    public Command outtake() {
        return new InstantCommand(() -> {
            transferMotor1.setPower(1);
        });
    }

    public Command stop() {
        return new InstantCommand(() -> {
            CURRENT_POWER = 0;
            THIRD_GATE_POWER = 0;
            transferMotor1.setPower(CURRENT_POWER);
            transferMotor2.setPower(-THIRD_GATE_POWER);
        });
    }

    public Command intakeAll() {
        return new InstantCommand(() -> {
            CURRENT_POWER = -INTAKE_POWER;
            THIRD_GATE_POWER = -INTAKE_POWER;
            transferMotor1.setPower(CURRENT_POWER);
            transferMotor2.setPower(-THIRD_GATE_POWER);
        });
    }

    public Command outtakeAll() {
        return new InstantCommand(() -> {
            transferMotor1.setPower(1);
            transferMotor2.setPower(1);
        });
    }

    public Command stopAll() {
        return new InstantCommand(() -> {
            CURRENT_POWER = 0;
            transferMotor1.setPower(CURRENT_POWER);
            transferMotor2.setPower(CURRENT_POWER);
        });
    }

    public Command start() {
        return new InstantCommand(() -> {
            transferMotor1.getMotor().setMotorEnable();
            transferMotor2.getMotor().setMotorEnable();
            servoGate1.getServo().getController().pwmEnable();
            servoGate2.getServo().getController().pwmEnable();
        });
    }

    public Command emergencyStopAll() {
        return new InstantCommand(() -> {
            CURRENT_POWER = 0;
            transferMotor1.getMotor().setMotorDisable();
            transferMotor2.getMotor().setMotorDisable();
            servoGate1.getServo().getController().pwmDisable();
            servoGate2.getServo().getController().pwmDisable();
        });
    }

    public  Command openGate() {
        return new InstantCommand(() -> {
            GATE_OVERRIDE = true;
            GATE3_STOPPED = false;
            ALL_STOPPED = false;
            BOTH_FULL_TIMER_RUNNING = false;
            THIRD_GATE_POWER = INTAKE_POWER;

            // Run motors first, then open gate
            transferMotor1.setPower(-INTAKE_POWER);
            transferMotor2.setPower(-THIRD_GATE_POWER);
            servoGate1.setPosition(GATE_ONE_OPEN);
            servoGate2.setPosition(GATE_TWO_OPEN);
        });
    }

    public Command closeGate() {
        return new InstantCommand(() -> {
            GATE_OVERRIDE = false;
            GATE3_STOPPED = false;
            ALL_STOPPED = false;
            BOTH_FULL_TIMER_RUNNING = false;
            THIRD_GATE_POWER = INTAKE_POWER;
            transferMotor2.setPower(-THIRD_GATE_POWER);
            servoGate1.setPosition(GATE_ONE_CLOSED);
            servoGate2.setPosition(GATE_TWO_CLOSED);
        });
    }
}