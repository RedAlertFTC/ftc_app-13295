package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DMRoverTeleOpV1")
public class DMRoverTeleOpV1 extends DMRoverAbstract {

    double left;
    double right;
    double extend;
    double MCSarm;


    public void init(){

        super.init();

    }

        public void runOpMode() {

            // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            left = gamepad1.left_stick_y;
            right = gamepad1.left_stick_x;
            leftDrive.setPower(left);
            rightDrive.setPower(right);

/*
            //activate extension and MCS arm in unison
            if (gamepad2.right_bumper = true) {
                extensionMotor.setDirection(DcMotor.Direction.FORWARD);
                mineralArm.setDirection(DcMotor.Direction.FORWARD);
                extensionMotor.setPower(.25);
                mineralArm.setPower(.5);
            } else {
                extensionMotor.setPower(0);
                mineralArm.setPower(0);
            }

            if (gamepad2.left_bumper = true) {
                extensionMotor.setDirection(DcMotor.Direction.REVERSE);
                mineralArm.setDirection(DcMotor.Direction.REVERSE);
                extensionMotor.setPower(.25);
                mineralArm.setPower(.5);
            } else {
                extensionMotor.setPower(0);
                mineralArm.setPower(0);
            }
*/


            // activate extension and MCS arm
            extend = gamepad2.right_stick_x;
            extensionMotor.setPower(extend);

            MCSarm = gamepad2.left_stick_y;
            mineralArm.setPower(MCSarm);



            //rotate the motor on the MCS box
            if (gamepad2.left_trigger > 0) {
                mineralBox.setDirection(DcMotor.Direction.FORWARD);
                mineralBox.setPower(1);
            }

            if (gamepad2.right_trigger > 0) {
                mineralBox.setDirection(DcMotor.Direction.REVERSE);
                mineralBox.setPower(1);
            }



            // activate lifting arm
            if (gamepad1.dpad_down = true) {
                liftArm.setDirection(DcMotor.Direction.FORWARD);
                liftArm.setPower(1);
            }

            if (gamepad1.dpad_up = true) {
                liftArm.setDirection(DcMotor.Direction.REVERSE);
                liftArm.setPower(1);
            }
        }
    }
}