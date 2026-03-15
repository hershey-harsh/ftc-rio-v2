package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;

import static dev.nextftc.bindings.Bindings.*;
import org.firstinspires.ftc.teamcode.subsystems.Light;

@Deprecated
@TeleOp(name = "Light", group = "Debug")
public class LightDebug extends NextFTCOpMode {
    double LOOP_TIME = 0;
    ElapsedTime LOOP_TIMER = new ElapsedTime();

    // Available colors to cycle through
    private static final double[] COLORS = {
            Light.OFF, Light.RED, Light.ORANGE, Light.YELLOW,
            Light.SAGE, Light.GREEN, Light.AZURE, Light.BLUE,
            Light.INDIGO, Light.VIOLET, Light.WHITE
    };
    private static final String[] COLOR_NAMES = {
            "OFF", "RED", "ORANGE", "YELLOW",
            "SAGE", "GREEN", "AZURE", "BLUE",
            "INDIGO", "VIOLET", "WHITE"
    };

    private int colorIndex = 5; // start on GREEN
    private Light.Target currentTarget = Light.Target.BOTH;
    private boolean blinking = false;
    private double blinkInterval = 500; // ms
    private static final double BLINK_STEP = 50; // ms

    public LightDebug() {
        addComponents(
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Light.INSTANCE)
        );
    }

    @Override
    public void onInit() {
        // Lights initialize to GREEN in Light.initialize()
    }

    @Override
    public void onStartButtonPressed() {
        // D-Pad Right — next color
        button(() -> gamepad1.dpad_right)
                .whenBecomesTrue(() -> {
                    colorIndex = (colorIndex + 1) % COLORS.length;
                    applyColor();
                });

        // D-Pad Left — previous color
        button(() -> gamepad1.dpad_left)
                .whenBecomesTrue(() -> {
                    colorIndex = (colorIndex - 1 + COLORS.length) % COLORS.length;
                    applyColor();
                });

        // A — cycle target: BOTH -> ROBOT -> TURRET -> BOTH
        button(() -> gamepad1.a)
                .whenBecomesTrue(() -> {
                    switch (currentTarget) {
                        case BOTH:   currentTarget = Light.Target.ROBOT;  break;
                        case ROBOT:  currentTarget = Light.Target.TURRET; break;
                        case TURRET: currentTarget = Light.Target.BOTH;   break;
                    }
                    applyColor();
                });

        // B — toggle blinking
        button(() -> gamepad1.b)
                .whenBecomesTrue(() -> {
                    blinking = !blinking;
                    applyColor();
                });

        // D-Pad Up — increase blink speed (decrease interval)
        button(() -> gamepad1.dpad_up)
                .whenBecomesTrue(() -> {
                    blinkInterval = Math.max(100, blinkInterval - BLINK_STEP);
                    if (blinking) applyColor();
                });

        // D-Pad Down — decrease blink speed (increase interval)
        button(() -> gamepad1.dpad_down)
                .whenBecomesTrue(() -> {
                    blinkInterval = Math.min(2000, blinkInterval + BLINK_STEP);
                    if (blinking) applyColor();
                });

        // X — turn lights off
        button(() -> gamepad1.x)
                .whenBecomesTrue(() -> {
                    blinking = false;
                    Light.INSTANCE.off().schedule();
                });

        // Y — turn lights on with current color
        button(() -> gamepad1.y)
                .whenBecomesTrue(this::applyColor);

        // Apply initial color
        applyColor();
    }

    private void applyColor() {
        double color = COLORS[colorIndex];
        if (blinking) {
            Light.INSTANCE.setBlinkingColor(color, blinkInterval, currentTarget).schedule();
        } else {
            Light.INSTANCE.setColor(color, currentTarget).schedule();
        }
    }

    @Override
    public void onUpdate() {
        LOOP_TIMER.reset();

        // Telemetry
        telemetry.addData("Loop Time (ms)", LOOP_TIME);
        telemetry.addData("Loop Time (hz)", LOOP_TIME > 0 ? (1000 / LOOP_TIME) : 0);

        telemetry.addLine();
        telemetry.addData("=== Light ===", "");
        telemetry.addData("Color", COLOR_NAMES[colorIndex]);
        telemetry.addData("Color Value", COLORS[colorIndex]);
        telemetry.addData("Target", currentTarget);
        telemetry.addData("Blinking", blinking);
        telemetry.addData("Blink Interval (ms)", blinkInterval);

        telemetry.addLine();
        telemetry.addLine("--- Controls ---");
        telemetry.addData("D-Pad Right/Left", "Cycle Color");
        telemetry.addData("D-Pad Up/Down", "Blink Speed +/-");
        telemetry.addData("A", "Cycle Target (Both/Robot/Turret)");
        telemetry.addData("B", "Toggle Blinking");
        telemetry.addData("X", "All Off");
        telemetry.addData("Y", "Apply Current Color");

        telemetry.update();

        LOOP_TIME = LOOP_TIMER.milliseconds();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
    }
}
