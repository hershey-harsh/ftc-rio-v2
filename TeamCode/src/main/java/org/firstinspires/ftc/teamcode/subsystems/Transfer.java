package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Configuration;

import java.util.Locale;

import dev.nextftc.core.commands.Command;
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
    public boolean GATE3_STOPPED = false;        // true after ball detected and motor stopped
    public boolean GATE3_BALL_PRESENT = false;   // latched: true = ball confirmed at gate 3
    private boolean STARTED = false;             // true after intake() is called — prevents running during init
    private static final double GATE3_THRESHOLD = 8;  // cm

    // Gate 3 motor stall protection — backstop against burnout when a ball blocks the wheel
    // and the distance sensor misses it. Runs in ALL modes, including auto/override where the
    // functional auto-stop below is bypassed (that's where it stalled/burned before). Sustained
    // current above GATE3_STALL_CURRENT for GATE3_STALL_TIME latches the motor off until a gate
    // command (open/close) clears it. Tune the threshold to the motor with the "Gate3 Motor (A)"
    // telemetry: set it a bit above the normal free-running draw, below the stall draw.
    public static double GATE3_STALL_CURRENT = 5.0;   // amps
    public static double GATE3_STALL_TIME = 0.4;      // seconds of sustained overcurrent
    public static double gate3MotorCurrent = 0;       // last reading (telemetry)
    public boolean GATE3_MOTOR_STALLED = false;       // latched: motor cut due to stall
    private final ElapsedTime gate3StallTimer = new ElapsedTime();

    // Gate 1&2 break beam debounce — sensor is noisy when ball is present (toggles 3-4x in ~2s)
    // count rising edges (transitions from false→true) in a rolling window.
    // If see enough edges within the window latch GATE12_BALL_PRESENT = true.
    // The latch is only cleared when the gate is opened (ball leaves).
    private boolean gate12LastRawState = false;    // previous raw sensor reading
    public int gate12TransitionCount = 0;          // number of falling edges in current window
    private final ElapsedTime gate12WindowTimer = new ElapsedTime(); // rolling window timer
    public boolean GATE12_BALL_PRESENT = false;   // latched: true = ball confirmed at gate 1&2
    private static final double GATE12_DEBOUNCE_WINDOW = 4.5;  // seconds window to count transitions
    public int GATE12_DEBOUNCE_THRESHOLD = 4;    // transitions needed to confirm ball

    // All full auto-stop: both gate3 AND gate1And2 latched full (method cooked i think)
    public boolean ALL_STOPPED = false;          // true after both sensors full — all motors stopped

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
        GATE3_BALL_PRESENT = false;
        ALL_STOPPED = false;
        GATE3_MOTOR_STALLED = false;
        gate3StallTimer.reset();
        GATE12_BALL_PRESENT = false;
        gate12LastRawState = false;
        gate12TransitionCount = 0;
        gate12WindowTimer.reset();
        servoGate1.setPosition(GATE_ONE_CLOSED);
        servoGate2.setPosition(GATE_TWO_CLOSED);
    }

    @Override
    public void periodic() {
        // --- Gate 3 motor stall protection (runs in ALL modes — burnout backstop) ---
        // Must come before the override/STARTED early-returns below, because the worst stalls
        // happened in auto (override == true), where the functional auto-stop never runs.
        gate3MotorCurrent = transferMotor2.getMotor().getCurrent(CurrentUnit.AMPS);
        if (GATE3_MOTOR_STALLED) {
            transferMotor2.setPower(0);                       // hold off until a gate command clears it
        } else if (gate3MotorCurrent >= GATE3_STALL_CURRENT) {
            if (gate3StallTimer.seconds() >= GATE3_STALL_TIME) {
                transferMotor2.setPower(0);
                GATE3_MOTOR_STALLED = true;
                GATE3_BALL_PRESENT = true;                    // a jam IS a ball the sensor missed
                GATE3_STOPPED = true;
            }
        } else {
            gate3StallTimer.reset();                          // current normal → restart the stall clock
        }

        gate3Distance = gate3Sensor.getDistance(DistanceUnit.CM);

        if (!GATE3_BALL_PRESENT && gate3Distance < GATE3_THRESHOLD) {
            GATE3_BALL_PRESENT = true;
        }

        boolean gate12Raw = gate1And2Sensor.getState();

        if (!GATE12_BALL_PRESENT) {
            if (!gate12Raw && gate12LastRawState) {
                if (gate12TransitionCount == 0) {
                    gate12WindowTimer.reset();
                }
                gate12TransitionCount++;
            }

            if (gate12TransitionCount >= GATE12_DEBOUNCE_THRESHOLD
                    && gate12WindowTimer.seconds() <= GATE12_DEBOUNCE_WINDOW) {
                GATE12_BALL_PRESENT = true;
            }

            if (gate12WindowTimer.seconds() > GATE12_DEBOUNCE_WINDOW) {
                gate12TransitionCount = 0;
            }
        }
        gate12LastRawState = gate12Raw;

        Telemetry t = ActiveOpMode.telemetry();
        t.addLine();
        t.addData("----- Transfer Status -----", "");
        t.addData("Gate3 Motor (A)", String.format(Locale.US, "%.2f", gate3MotorCurrent));
        t.addData("Gate3 Stalled", GATE3_MOTOR_STALLED);
        t.addData("Gate3 Ball Present", GATE3_BALL_PRESENT);
        t.addData("Gate1&2 Ball Present", GATE12_BALL_PRESENT);

        if (!STARTED) return;
        if (GATE_OVERRIDE) return;
        if (override) return;

        if (GATE3_BALL_PRESENT) {
            GATE3_STOPPED = true;
            transferMotor2.setPower(0);
        } else if (GATE3_STOPPED) {
            GATE3_STOPPED = false;
            transferMotor2.setPower(-INTAKE_POWER);
        }

        if (GATE3_BALL_PRESENT && GATE12_BALL_PRESENT) {
            ALL_STOPPED = true;
            transferMotor1.setPower(0);
        } else if (ALL_STOPPED) {
            ALL_STOPPED = false;
            transferMotor1.setPower(-INTAKE_POWER);
        }
    }

    public Command intake() {
        return new InstantCommand(() -> {
            STARTED = true;
            CURRENT_POWER = -INTAKE_POWER;
            THIRD_GATE_POWER = INTAKE_POWER;

            if (!ALL_STOPPED) {
                transferMotor1.setPower(CURRENT_POWER);
            }
            if (!GATE3_STOPPED && !ALL_STOPPED) {
                transferMotor2.setPower(-THIRD_GATE_POWER);
            }
        });
    }

    public Command outtake() {
        return new InstantCommand(() -> {
            transferMotor1.setPower(1);
        });
    }

    public Command stop() {
        return new InstantCommand(() -> {
            STARTED = false;
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
        return openGate(INTAKE_POWER);
    }

    public Command openGate(double power) {
        return new InstantCommand(() -> {
            if (!Shooter.INSTANCE.isUpToSpeed()) return;

            GATE_OVERRIDE = true;
            GATE3_MOTOR_STALLED = false;
            gate3StallTimer.reset();
            GATE3_STOPPED = false;
            GATE3_BALL_PRESENT = false;
            ALL_STOPPED = false;
            THIRD_GATE_POWER = power;
            GATE12_BALL_PRESENT = false;
            gate12TransitionCount = 0;
            gate12WindowTimer.reset();

            transferMotor1.setPower(-power);
            transferMotor2.setPower(-THIRD_GATE_POWER);
            servoGate1.setPosition(GATE_ONE_OPEN);
            servoGate2.setPosition(GATE_TWO_OPEN);
        });
    }

    public Command closeGate() {
        return new InstantCommand(() -> {
            GATE_OVERRIDE = false;
            GATE3_MOTOR_STALLED = false;
            gate3StallTimer.reset();
            GATE3_STOPPED = false;
            GATE3_BALL_PRESENT = false;
            ALL_STOPPED = false;
            THIRD_GATE_POWER = INTAKE_POWER;

            transferMotor2.setPower(-THIRD_GATE_POWER);
            servoGate1.setPosition(GATE_ONE_CLOSED);
            servoGate2.setPosition(GATE_TWO_CLOSED);
        });
    }
}
