package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.concurrent.TimeUnit;

public abstract class DMRoverAbstract extends LinearOpMode {

    double left;
    double right;

    volatile boolean   isStarted       = false;
    volatile boolean   stopRequested   = false;


    DcMotor leftDrive = hardwareMap.dcMotor.get("mLeft");
    DcMotor rightDrive = hardwareMap.dcMotor.get("mRight");
    DcMotor mineralArm = hardwareMap.dcMotor.get("mineralArm");
    DcMotor mineralCollector = hardwareMap.dcMotor.get("boxMotor");
    DcMotor liftArm = hardwareMap.dcMotor.get("liftArm");



    public void runOpMode() {

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }





}


