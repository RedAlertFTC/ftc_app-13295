package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class DMRoverAbstract extends OpMode {

    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor mineralArm;
    DcMotor mineralBox;
    DcMotor liftArm;
    DcMotor extensionMotor;

    public static int seqRobot = 1;


    public void init() {

        leftDrive = hardwareMap.dcMotor.get("mLeft");
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setDirection(DcMotor.Direction.FORWARD);

        rightDrive = hardwareMap.dcMotor.get("mRight");
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        mineralArm = hardwareMap.dcMotor.get("mArm");
        mineralArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mineralArm.setDirection(DcMotor.Direction.FORWARD);

        mineralBox = hardwareMap.dcMotor.get("mBox");
        mineralBox.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mineralBox.setDirection(DcMotor.Direction.REVERSE);

        liftArm = hardwareMap.dcMotor.get("mLift");
        liftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftArm.setDirection(DcMotor.Direction.REVERSE);

        extensionMotor = hardwareMap.dcMotor.get("mExtend");
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionMotor.setDirection(DcMotor.Direction.REVERSE);
    }


    public void loop() {

    }


    public void stop() {

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        mineralArm.setPower(0);
        mineralBox.setPower(0);
        liftArm.setPower(0);
        extensionMotor.setPower(0);
    }


    int cmdMoveR(float distIn, float encoderCntPerIn, double power, DcMotor motor) {

        // Solve for encoder count target. (int) needed to cast result as integer
        int target = ((int) (distIn * encoderCntPerIn));// + motor.getCurrentPosition();

        // Set motor target and power
        motor.setTargetPosition(target);
        motor.setPower(power);

        return target;
    }


    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException var4) {
            Thread.currentThread().interrupt();
        }

    }
}