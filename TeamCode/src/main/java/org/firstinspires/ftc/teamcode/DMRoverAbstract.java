package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class DMRoverAbstract extends OpMode {

    DcMotor leftDrive = hardwareMap.dcMotor.get("mLeft");
    DcMotor rightDrive = hardwareMap.dcMotor.get("mRight");
    DcMotor mineralArm = hardwareMap.dcMotor.get("mArm");
    DcMotor mineralBox = hardwareMap.dcMotor.get("mBox");
    DcMotor liftArm = hardwareMap.dcMotor.get("mLift");
    DcMotor extensionMotor = hardwareMap.dcMotor.get("mExtend");


    public void init() {

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
    }

    public void stop() {

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        mineralArm.setPower(0);
        mineralBox.setPower(0);
        liftArm.setPower(0);
        extensionMotor.setPower(0);
    }


    public void loop() {

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

    private volatile boolean   isStarted       = false;
    private volatile boolean   stopRequested   = false;

    public final void idle() {
        // Otherwise, yield back our thread scheduling quantum and give other threads at
        // our priority level a chance to run
        Thread.yield();
    }

    public final boolean isStopRequested() {
        return this.stopRequested || Thread.currentThread().isInterrupted();
    }

    public final boolean isStarted() {
        return this.isStarted || Thread.currentThread().isInterrupted();
    }

    public final boolean opModeIsActive() {
        boolean isActive = !this.isStopRequested() && this.isStarted();
        if (isActive) {
            idle();
        }
        return isActive;
    }
}
