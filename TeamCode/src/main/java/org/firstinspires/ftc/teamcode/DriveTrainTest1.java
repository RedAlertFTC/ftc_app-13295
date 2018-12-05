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
        double extend;
        double MCSarm;

        DcMotor leftDrive = hardwareMap.dcMotor.get("mLeft");
        DcMotor rightDrive = hardwareMap.dcMotor.get("mRight");
        DcMotor mineralArm = hardwareMap.dcMotor.get("mArm");
        DcMotor mineralBox = hardwareMap.dcMotor.get("mBox");
        DcMotor liftArm = hardwareMap.dcMotor.get("mLift");
        DcMotor extensionMotor = hardwareMap.dcMotor.get("mExtend");

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        liftArm.setPower(0);
        mineralArm.setPower(0);
        mineralBox.setPower(0);
        extensionMotor.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setDirection(DcMotor.Direction.FORWARD);

        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        mineralArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mineralArm.setDirection(DcMotor.Direction.FORWARD);

        mineralBox.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mineralBox.setDirection(DcMotor.Direction.FORWARD);

        liftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftArm.setDirection(DcMotor.Direction.FORWARD);

        extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extensionMotor.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()){

            // Run wheels in tank drive (note: The joystick goes negative when pushed forwards, so negate it)
            left = gamepad1.left_stick_y;
            right = gamepad1.right_stick_y;
            leftDrive.setPower(left);
            rightDrive.setPower(right);


                // run until the end of the match (driver presses STOP)
                while (opModeIsActive()) {

                    // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
                    left = gamepad1.left_stick_y;
                    right = gamepad1.left_stick_x;
                    leftDrive.setPower(left);
                    rightDrive.setPower(right);

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


                    while (gamepad2.left_trigger > 0) {
                        mineralBox.setDirection(DcMotor.Direction.FORWARD);
                        mineralBox.setPower(1);
                    }

                    while (gamepad2.right_trigger > 0) {
                        mineralBox.setDirection(DcMotor.Direction.REVERSE);
                        mineralBox.setPower(1);
                    }

                    extend = gamepad2.right_stick_x;
                    extensionMotor.setPower(extend);

                    MCSarm = gamepad2.left_stick_y;
                    mineralArm.setPower(MCSarm);

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
    }