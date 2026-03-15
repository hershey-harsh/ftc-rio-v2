package org.firstinspires.ftc.teamcode.opmodes;
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
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Deprecated
@TeleOp(name = "Rushed")
public class Rushed extends NextFTCOpMode {
    private double X_VELOCITY = 0;
    private double Y_VELOCITY = 0;

    double LOOP_TIME = 0;
    ElapsedTime LOOP_TIMER = new ElapsedTime();

    private DriverControlledCommand driverControlled;

    // Track toggle states for gate motors
    private boolean gate3Running = false;
    private boolean gate1And2Running = false;

    public Rushed() {
        addComponents(
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Shooter.INSTANCE),
                new SubsystemComponent(Transfer.INSTANCE),
                new SubsystemComponent(Turret.INSTANCE)
        );
    }

    @Override
    public void onInit() {
        Gamepads.gamepad1().leftStickY();
        Gamepads.gamepad1().leftStickX();
        Gamepads.gamepad1().rightStickX();

        PedroComponent.follower().setStartingPose(Configuration.CURRENT_POSE);

        // Close gate on init
        Shooter.INSTANCE.off().schedule();
        Transfer.INSTANCE.closeGate().schedule();
    }

    @Override
    public void onStartButtonPressed() {
        driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX(),
                !Configuration.FIELD_CENTRIC
        );

        // Invert the controls by using negative scalar
        driverControlled.setScalar(-Configuration.CONTROL_SCALE);

        driverControlled.schedule();

        // Start intake on play
        Transfer.INSTANCE.intake().schedule();
        Shooter.INSTANCE.on().schedule();

        // X button - localize to 0, 0, 90 degrees
        button(() -> gamepad1.x)
                .whenTrue(() -> PedroComponent.follower().setPose(Configuration.MANUAL_LOCALIZATION_POSE));




        // Right bumper - open gate while held, close when released
        button(() -> gamepad2.right_bumper)
                .whenTrue(() -> Transfer.INSTANCE.openGate().schedule())
                .whenBecomesFalse(() -> Transfer.INSTANCE.closeGate().schedule());

        button(() -> gamepad2.b)
                .toggleOnBecomesFalse()
                .whenBecomesTrue(() -> Transfer.INSTANCE.intake().schedule())
                .whenBecomesFalse(() -> Transfer.INSTANCE.stop().schedule());

        button(() -> gamepad2.a)
                .toggleOnBecomesFalse()
                .whenBecomesTrue(() -> Transfer.INSTANCE.outtake().schedule())
                .whenBecomesFalse(() -> Transfer.INSTANCE.stop().schedule());

//        // B button - toggle gate 3 (turn off if running, turn on if stopped)
//        button(() -> gamepad2.b)
//                .toggleOnBecomesFalse()
//                .whenBecomesTrue(() -> {
//                    if (gate3Running) {
//                        Transfer.INSTANCE.gate3Stop().schedule();
//                    } else {
//                        Transfer.INSTANCE.gate3Start().schedule();
//                    }
//                    gate3Running = !gate3Running;
//                });
//
//        // A button - toggle gate 1 and 2 (turn off if running, turn on if stopped)
//        button(() -> gamepad2.a)
//                .toggleOnBecomesFalse()
//                .whenBecomesTrue(() -> {
//                    if (gate1And2Running) {
//                        Transfer.INSTANCE.gate1And2Stop().schedule();
//                    } else {
//                        Transfer.INSTANCE.gate1And2Start().schedule();
//                    }
//                    gate1And2Running = !gate1And2Running;
//                });
    }

    @Override
    public void onUpdate() {
        LOOP_TIMER.reset();

        telemetry.addData("Loop Time (ms)", LOOP_TIME);
        telemetry.addData("Loop Time (hz)", (1000/LOOP_TIME));

        driverControlled.setScalar(-Configuration.CONTROL_SCALE);

        X_VELOCITY = PedroComponent.follower().getVelocity().getXComponent();
        Y_VELOCITY = PedroComponent.follower().getVelocity().getYComponent();

        double goalDistance = Shooter.INSTANCE.GOAL_DISTANCE;
        double hoodAngle = Shooter.INSTANCE.HOOD_ANGLE;
//        double TOF = Shooter.INSTANCE.getTOF(goalDistance, Math.toRadians(hoodAngle));

//        Configuration.setAimPointOffset(-X_VELOCITY * Configuration.ARTIFACT_TRANSFER_TIME + TOF,
//                -Y_VELOCITY * Configuration.ARTIFACT_TRANSFER_TIME + TOF);

//        telemetry.addData("COLOR R:", "%.3f", Transfer.redValues);
//        telemetry.addData("COLOR G:", "%.3f", Transfer.greenValues);
//        telemetry.addData("COLOR B:", "%.3f", Transfer.blueValues);
//        telemetry.addData("Is Purple:", Transfer.INSTANCE.isPurple());
//        telemetry.addData("Is Green:", Transfer.INSTANCE.isGreen());
        telemetry.addData("=== Position ===", "");
        telemetry.addData("Position X:", PedroComponent.follower().getPose().getX());
        telemetry.addData("Position Y:", PedroComponent.follower().getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));

//        telemetry.addData("=== Limelight ===", "");
//        telemetry.addData("LL Auto Update", Limelight.INSTANCE.autoUpdateEnabled);
//        telemetry.addData("LL Localizing", Limelight.INSTANCE.limelightResult != null && Limelight.INSTANCE.limelightResult.isValid());

        telemetry.addData("=== Turret ===", "");
        telemetry.addData("Turret Mode", Turret.INSTANCE.mode);
//        telemetry.addData("Turret Angle", Turret.INSTANCE.TURRET_ANGLE);
//        telemetry.addData("Turret Target", Turret.INSTANCE.TARGET_DEGREE);
//        telemetry.addData("Turret Servo Pos", "%.3f", Turret.INSTANCE.TURRET_POSITION);

        telemetry.addData("=== Shooter ===", "");
        telemetry.addData("Shooter Mode", Shooter.INSTANCE.mode);
        telemetry.addData("Goal Distance", Shooter.INSTANCE.GOAL_DISTANCE);
        //telemetry.addData("Flywheel RPM Goal", Shooter.INSTANCE.FLYWHEEL_RPM_GOAL);
        //telemetry.addData("Hood Angle", Shooter.INSTANCE.HOOD_ANGLE);
        //telemetry.addData("Time of Flight", TOF);

        BindingManager.update();
        telemetry.update();

        LOOP_TIME = LOOP_TIMER.milliseconds();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
    }
}


//        if (Shooter.INSTANCE.mode == Shooter.Mode.odometry) {
//X_VELOCITY = PedroComponent.follower().getVelocity().getXComponent();
//Y_VELOCITY = PedroComponent.follower().getVelocity().getYComponent();
//
//TRUE_TARGET_DEGREE = Turret.INSTANCE.TRUE_TARGET_DEGREE;
//FLYWHEEL_VALUES = Shooter.getShooterValues(Shooter.INSTANCE.GOAL_DISTANCE);
//
//Shooter.INSTANCE.TARGET_RPM = Shooter.INSTANCE.artifactVelocityMStoRPM(FLYWHEEL_VALUES.ARTIFACT_VELOCITY);
//Shooter.INSTANCE.HOOD_ANGLE = FLYWHEEL_VALUES.HOOD_ANGLE;
//
////            double weight = Shooter.INSTANCE.getWeight();
////            Configuration.setAimPointOffset(-X_VELOCITY * weight, -Y_VELOCITY * weight);
//
//double vyr = ((Y_VELOCITY * 0.0254) * Math.sin(Math.PI / 2 - TRUE_TARGET_DEGREE))
//        + ((X_VELOCITY * 0.0254) * Math.sin(TRUE_TARGET_DEGREE));
//double vxr = -((Y_VELOCITY * 0.0254) * Math.cos(Math.PI / 2 - TRUE_TARGET_DEGREE))
//        + ((X_VELOCITY * 0.0254) * Math.cos(TRUE_TARGET_DEGREE));
//
//double vn = FLYWHEEL_VALUES.ARTIFACT_VELOCITY - (vyr * Shooter.vcWeight);
//double vt = Math.sqrt((vn * vn) + (vxr * vxr));
//
//double oo = Math.atan((-vxr) / vn);
//
//Configuration.TURRET_OFFSET = oo;
//Shooter.INSTANCE.HOOD_ANGLE = FLYWHEEL_VALUES.HOOD_ANGLE;
//Shooter.INSTANCE.TARGET_RPM = Shooter.INSTANCE.artifactVelocityMStoRPM(vt) * 2.75;