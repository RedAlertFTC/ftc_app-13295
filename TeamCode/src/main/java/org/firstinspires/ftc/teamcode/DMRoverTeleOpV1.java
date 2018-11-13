package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DMRoverTeleOpV1 extends DMRoverAbstract {


    public void runOpMode() {

        loop();


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


