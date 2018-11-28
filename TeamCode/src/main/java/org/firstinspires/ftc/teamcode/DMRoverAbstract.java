package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public abstract class DMRoverAbstract extends LinearOpMode {

    double left;
    double right;

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
}