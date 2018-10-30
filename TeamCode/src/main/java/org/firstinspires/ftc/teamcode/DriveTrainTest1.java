package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "DriveTrainTest_TwoStick")
class DriveTrainTest1 extends LinearOpMode {

    @Override
    public void runOpMode() {
        double left;
        double right;

        DcMotor leftDrive = hardwareMap.dcMotor.get("mLeft");
        DcMotor rightDrive = hardwareMap.dcMotor.get("mRight");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            left = gamepad1.left_stick_y;
            right = gamepad1.right_stick_x;
            leftDrive.setPower(left);
            rightDrive.setPower(right);

            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);
        }
    }
}