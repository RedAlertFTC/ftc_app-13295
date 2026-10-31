package org.firstinspires.ftc.teamcode.testing2025;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="MotorTesting", group="TeleOp")
@Disabled
public class MotorTesting extends LinearOpMode {

    private DcMotor TestMotor = null;


    @Override
    public void runOpMode() {

        TestMotor = hardwareMap.get(DcMotor.class, "test_motor");
        TestMotor.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.a){
                TestMotor.setPower(500);
            }

            if(gamepad1.b){
                TestMotor.setPower(0);
            }
        }
    }

}
