package org.firstinspires.ftc.teamcode.season2025.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DecodeV1 TeleOp Red")
@Disabled
public class DecodeV1TeleOpRed extends DecodeV1Teleop
{
    @Override
    public void runOpMode() throws InterruptedException {
        setGoalAprilTag(24);
        _teleOpAlliance = teleOpAlliance.RED;

        super.runOpMode();
    }
}
