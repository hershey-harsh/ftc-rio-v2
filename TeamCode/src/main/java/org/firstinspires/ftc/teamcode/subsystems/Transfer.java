package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
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

    public RevColorSensorV3 gate3Sensor;
    public static double gate3Distance;

    public NormalizedRGBA colors;
    public static boolean override = false;

    // Gate 3 auto-stop
    private boolean gateOverride = false;       // true when openGate() is called — ignores distance
    private boolean gate3Stopped = false;        // true after ball detected and motor stopped
    private boolean started = false;             // true after intake() is called — prevents running during init
    private static final double GATE3_THRESHOLD = 3.0;  // cm

    @Override
    public void initialize() {
        transferMotor1 = new MotorEx(ActiveOpMode.hardwareMap().get(DcMotorEx.class, Configuration.TRANSFER_MOTOR_ONE)).reversed();
        transferMotor2 = new MotorEx(ActiveOpMode.hardwareMap().get(DcMotorEx.class, Configuration.TRANSFER_MOTOR_TWO));

        servoGate1 = new ServoEx(ActiveOpMode.hardwareMap().get(Servo.class, Configuration.SERVO_GATE_LEFT));
        servoGate2 = new ServoEx(ActiveOpMode.hardwareMap().get(Servo.class, Configuration.SERVER_GATE_RIGHT));

        gate3Sensor = ActiveOpMode.hardwareMap().get(RevColorSensorV3.class, Configuration.GATE_3_SENSOR);

        started = false;
        gateOverride = false;
        gate3Stopped = false;
        servoGate1.setPosition(GATE_ONE_CLOSED);
        servoGate2.setPosition(GATE_TWO_CLOSED);
    }

    @Override
    public void periodic() {
        gate3Distance = gate3Sensor.getDistance(DistanceUnit.CM);

        if (!started) return;
        if (gateOverride) return;

        if (gate3Distance < GATE3_THRESHOLD) {
            // Ball detected — run for 0.5s then stop
            if (!gate3Stopped) {
                gate3Stopped = true;
                new SequentialGroup(
                        new Delay(0.5),
                        new InstantCommand(() -> {
                            if (!gateOverride) transferMotor2.setPower(0);
                        })
                ).schedule();
            }
        } else {
            // No ball — if motor was stopped, restart it
            if (gate3Stopped) {
                gate3Stopped = false;
                transferMotor2.setPower(-INTAKE_POWER);
            }
        }
    }

    public Command intake() {
        return new InstantCommand(() -> {
            started = true;
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

    public Command startAll() {
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
            gateOverride = true;
            gate3Stopped = false;
            THIRD_GATE_POWER = INTAKE_POWER;
            servoGate1.setPosition(GATE_ONE_OPEN);
            servoGate2.setPosition(GATE_TWO_OPEN);
            transferMotor2.setPower(-THIRD_GATE_POWER);
        });
    }

    public Command closeGate() {
        return new InstantCommand(() -> {
            gateOverride = false;
            gate3Stopped = false;
            THIRD_GATE_POWER = INTAKE_POWER;
            transferMotor2.setPower(-THIRD_GATE_POWER);
            servoGate1.setPosition(GATE_ONE_CLOSED);
            servoGate2.setPosition(GATE_TWO_CLOSED);
        });
    }
}