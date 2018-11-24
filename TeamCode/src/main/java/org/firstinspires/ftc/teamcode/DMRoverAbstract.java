package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.concurrent.TimeUnit;

public abstract class DMRoverAbstract extends LinearOpMode {

    double left;
    double right;


    DcMotor leftDrive = hardwareMap.dcMotor.get("mLeft");
    DcMotor rightDrive = hardwareMap.dcMotor.get("mRight");
    DcMotor mineralArm = hardwareMap.dcMotor.get("mArm");
    //DcMotor mineralBox = hardwareMap.dcMotor.get("mBox");
    DcMotor liftArm = hardwareMap.dcMotor.get("mLift");
    //DcMotor extensionMotor = hardwareMap.dcMotor.get("mExtend");
    /* @Override
    public void init() {
         leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
         rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
         mineralArm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
         mineralBox.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
         liftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
         extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
     }*/
}


