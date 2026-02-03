package org.firstinspires.ftc.teamcode.season2025.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.season2025.Components.BallLauncher;
import org.firstinspires.ftc.teamcode.season2025.Components.BallSpooner;
import org.firstinspires.ftc.teamcode.season2025.Components.Turntable;

@Autonomous(name="DecodeV3AutoOpWallBlue", group = "Robot")
@Disabled
public class DecodeV3AutoOpWallBlue extends LinearOpMode{

    private enum RobotStatus
    {
        ESCAPE,
        ESCAPING,
       // SPINNING,
        SHOOT,
        SHOOTING,
        DONE_SHOOTING,
        LEAVE_ZONE
    }

    private ElapsedTime runtime = new ElapsedTime();

    private RobotStatus robotStatus = RobotStatus.ESCAPE;

    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private static final double ESCAPE_TIME = 0.5;
    private static final double SPIN_TIME = 2;
    private static final double SHOOTING_TIME = 6;
    private static final double LEAVE_WHITE_TIME = 2;

    private BallSpooner _ballSpooner;
    private Turntable _turntable;
    private BallLauncher _ballLauncher;

    @Override
    public void runOpMode() {

        frontLeftDrive = hardwareMap.get(DcMotor.class, "fl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bl");
        backRightDrive = hardwareMap.get(DcMotor.class, "br");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        initialize();

        runtime.reset();
        waitForStart();


        while(opModeIsActive()){

            //EscapeWall
            //SpinToGoal
            //PowerUp
            //Spooner
            //Turntable(slot)
            //Spooner
            //Tutntable(slot)
            //Spooner

            telemetry.addData("RobotState", robotStatus);

            switch (robotStatus){
                case ESCAPE:
                    EscapeWall();
                    break;
               // case SPINNING:
                //    SpinningState();
                case SHOOT:
                    ShootingState();
                    break;
                case LEAVE_ZONE:
                    LeaveZone();
                    break;

            }

            telemetry.update();
        }
    }


    private void EscapeWall(){
        if (runtime.seconds() <= ESCAPE_TIME){
            robotStatus = RobotStatus.ESCAPING;
            Backward();
        }
        else{
            runtime.reset();
            StopRobot();
            robotStatus = RobotStatus.SHOOTING;
        }
    }

    private void ShootAllBalls(){

        _ballLauncher.setLaunchPresetTwo();
        sleep(2000);
        _turntable.moveToIndex(1);
        sleep(1000);
        _ballSpooner.fire();
        sleep(1000);
        _turntable.moveToIndex(2);
        sleep(1000);
        _ballSpooner.fire();
        sleep(1000);
        _turntable.moveToIndex(3);
        sleep(1000);
        _ballSpooner.fire();

    }

    private void initialize(){
        _ballSpooner = new BallSpooner(hardwareMap, telemetry);
        _turntable = new Turntable(hardwareMap, telemetry);
        _ballLauncher = new BallLauncher(hardwareMap, telemetry);
    }

    private void Backward(){
        frontLeftDrive.setPower(-1);
        frontRightDrive.setPower(-1);
        backLeftDrive.setPower(-1);
        backRightDrive.setPower(-1);
    }

    private void SpinLeft(){
        frontLeftDrive.setPower(1);
        backLeftDrive.setPower(1);
        frontRightDrive.setPower(-1);
        backRightDrive.setPower(-1);
    }

    private void Left(){
        frontLeftDrive.setPower(-1);
        backLeftDrive.setPower(1);
        frontRightDrive.setPower(1);
        backRightDrive.setPower(-1);
    }
    private void StopRobot(){
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    private void SpinningState(){
        if (runtime.seconds() <= SPIN_TIME){
            SpinLeft();
        }
        else{
            StopRobot();
            runtime.reset();
            robotStatus = RobotStatus.SHOOT;
        }
    }

    private void ShootingState(){
        if (runtime.seconds() <= SHOOTING_TIME){
            robotStatus = RobotStatus.SHOOTING;
            ShootAllBalls();
        }
        else {
            StopRobot();
            runtime.reset();
            robotStatus = RobotStatus.LEAVE_ZONE;
        }
    }

    private void LeaveZone(){
        if (runtime.seconds() <= LEAVE_WHITE_TIME){
            Left();
        }
        else {
            StopRobot();
            runtime.reset();

        }
    }
}
