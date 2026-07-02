package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Configuration;

/**
 * RED competition teleop with NORMAL stick driving. Identical to {@link CompetitionBlueNormal}
 * except for the alliance.
 */
@TeleOp(name = "Competition Red (Normal)", group = "Competition")
public class CompetitionRedNormal extends CompetitionBlueNormal {
    @Override
    public void setupOpMode() {
        Configuration.ALLIANCE = Configuration.Alliance.RED;
    }
}
