package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Configuration;

@Disabled
@TeleOp(name = "Red FLL Demo", group = "AA FLL Demo")
public class RedFLLDemoSTOM extends org.firstinspires.ftc.teamcode.opmodes.BlueFLLDemoSTOMBase {
    @Override
    public void setupOpMode() {
        Configuration.ALLIANCE = Configuration.Alliance.RED;
    }
}
