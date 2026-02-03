package org.firstinspires.ftc.teamcode.season2025.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name = "DecodeV2 AutoOp Blue")
@Disabled
public class DecodeV2AutoOpBlue extends DecodeV2AutoOp
{


    @Override
    public void runOpMode(){
        setAprilTagID(20);
        _autoAlliance = autoAlliance.BLUE;
        super.runOpMode();
    }
}
