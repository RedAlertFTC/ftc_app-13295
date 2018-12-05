package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "DriveTrainTest_OneStick")
class DriveTrainTest2 extends LinearOpMode {

    @Override
    public void runOpMode() {
        double left;
        double right;

        DcMotor leftDrive = hardwareMap.dcMotor.get("mLeft");
        DcMotor rightDrive = hardwareMap.dcMotor.get("mRight");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.left_stick_y>0){
                left = gamepad1.left_stick_y;
                right = gamepad1.left_stick_x;
                leftDrive.setPower(left);
                rightDrive.setPower(right);
            }
            /*else{
                left = gamepad1.left_stick_y;
                right = gamepad1.left_stick_x;
                leftDrive.setPower(left);
                rightDrive.setPower(right);
            }*/

            if (gamepad1.left_stick_x>0) {
                left = gamepad1.left_stick_y;
                right = -gamepad1.left_stick_x;
                leftDrive.setPower(left);
                rightDrive.setPower(right);
            }
            /*else{
                left = -gamepad1.left_stick_y;
                right = gamepad1.left_stick_x;
                leftDrive.setPower(left);
                rightDrive.setPower(right);
            }

            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);*/
        }
    }
}