package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.driving.DriverControlledCommand;

import org.firstinspires.ftc.teamcode.Configuration;

/**
 * BLUE competition teleop with NORMAL stick driving (left stick = forward/strafe, right stick X
 * = rotate) instead of the Forza trigger drive. Everything else — SOTM, operator turret trim,
 * the gate run, and the gamepad-1 Y / X / right-bumper bindings — is inherited unchanged from
 * {@link CompetitionBlue}.
 */
@TeleOp(name = "Competition Blue (Normal)", group = "Competition")
public class CompetitionBlueNormal extends CompetitionBlue {
    @Override
    protected DriverControlledCommand createDriverControlled() {
        return new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().negate(),
                !Configuration.FIELD_CENTRIC
        );
    }
}
