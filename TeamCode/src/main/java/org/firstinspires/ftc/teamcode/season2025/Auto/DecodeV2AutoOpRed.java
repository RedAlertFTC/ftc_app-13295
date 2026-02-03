package org.firstinspires.ftc.teamcode.season2025.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name = "DecodeV2 AutoOp Red")
@Disabled
public class DecodeV2AutoOpRed extends  DecodeV2AutoOp
{
    @Override
    public void runOpMode(){
        setAprilTagID(24);
        _autoAlliance = autoAlliance.RED;
        super.runOpMode();
    }
}
