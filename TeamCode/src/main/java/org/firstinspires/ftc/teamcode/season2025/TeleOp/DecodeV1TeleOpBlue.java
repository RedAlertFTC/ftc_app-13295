package org.firstinspires.ftc.teamcode.season2025.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DecodeV1 TeleOp Blue")
public class DecodeV1TeleOpBlue extends DecodeV1Teleop
{

    @Override
    public void runOpMode() throws InterruptedException {
        setGoalAprilTag(20);
        _teleOpAlliance = teleOpAlliance.BLUE;

        super.runOpMode();
    }

}
