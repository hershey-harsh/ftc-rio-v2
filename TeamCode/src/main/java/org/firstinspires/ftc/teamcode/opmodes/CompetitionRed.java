package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Configuration;

/**
 * RED competition teleop. Identical to {@link CompetitionBlue} except for the alliance — all
 * aiming, offsets, gate-run poses, and the close-shoot zone re-mirror automatically.
 */
@TeleOp(name = "Competition Red", group = "Competition")
public class CompetitionRed extends CompetitionBlue {
    @Override
    public void setupOpMode() {
        Configuration.ALLIANCE = Configuration.Alliance.RED;
    }
}
