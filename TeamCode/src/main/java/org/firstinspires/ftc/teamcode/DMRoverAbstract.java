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


    // Initiates and names motors
    public void init()
    {
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


    public void loop()
    {

    }


    // Sets all motors to 0 power
    public void stop()
    {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        mineralArm.setPower(0);
        mineralBox.setPower(0);
        liftArm.setPower(0);
        extensionMotor.setPower(0);
    }


    // Initiates moves with encoders
    int cmdMoveR(float distIn, float encoderCntPerIn, double power, DcMotor motor)
    {
        // Solve for encoder count target. (int) needed to cast result as integer
        int target = ((int) (distIn * encoderCntPerIn));

        // Set motor target and power
        motor.setTargetPosition(target);
        motor.setPower(power);

        return target;
    }


    public final void sleep(long milliseconds)
    {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException var4) {
            Thread.currentThread().interrupt();
        }

    }

    // Shows values for motor power and position on driver's station for Autonomous
    public void telemetry()
    {
        telemetry.addData("left motor power",leftDrive.getPower());
        telemetry.addData("left motor encoder",leftDrive.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("right motor power",rightDrive.getPower());
        telemetry.addData("right motor encoder",rightDrive.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("extend motor power",extensionMotor.getPower());
        telemetry.addLine();
        telemetry.addData("main arm motor power",mineralArm.getPower());
        telemetry.addLine();
        telemetry.addData("box motor power",mineralBox.getPower());
        telemetry.addLine();
        telemetry.addData("lift arm motor power",liftArm.getPower());
        telemetry.update();

    }

    // Shows values for motor power and position and controls on driver's station for TeleOp
    public void teleOpTelemetry()
    {
        telemetry.addData("left motor power",leftDrive.getPower());
        telemetry.addData("left motor encoder",leftDrive.getCurrentPosition());
        telemetry.addData("left motor stick",gamepad1.left_stick_y);
        telemetry.addLine();
        telemetry.addData("right motor power",rightDrive.getPower());
        telemetry.addData("right motor stick",gamepad1.right_stick_y);
        telemetry.addData("right motor encoder",rightDrive.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("extend motor power",extensionMotor.getPower());
        telemetry.addData("extend motor control",gamepad2.right_stick_x);
        telemetry.addLine();
        telemetry.addData("main arm motor power",mineralArm.getPower());
        telemetry.addData("main arm stick",gamepad2.left_stick_y);
        telemetry.addLine();
        telemetry.addData("box motor power",mineralBox.getPower());
        telemetry.addData("box motor left bumper",gamepad2.left_bumper);
        telemetry.addData("box motor right bumper",gamepad2.right_bumper);
        telemetry.addLine();
        telemetry.addData("lift arm motor power",liftArm.getPower());
        telemetry.addData("lift arm up",gamepad2.dpad_up);
        telemetry.addData("lift arm down",gamepad2.dpad_down);
        telemetry.update();
    }
}