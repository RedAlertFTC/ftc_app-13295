package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "DMRoverAutonomousV1")
public class DMRoverAutonomousV1 extends DMRoverAbstract {

    @Override
    public void runOpMode() {

        liftArm.setPower(1);
        sleep(1000);
        liftArm.setPower(0);

        leftDrive.setPower(1);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setPower(1);
    }
}
