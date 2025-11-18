package org.firstinspires.ftc.teamcode.season2025.Components;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BallLauncher {

    private HardwareMap _hardwareMap;
    private Telemetry _telemetry;
    private DcMotorEx leftLaunchMotor;
    private DcMotorEx rightLaunchMotor;
    private double INCREMENT = 0.01;

    private Servo linearServo;
    private double _maxPower = 1.0;
    private double _currentPower = 0.0;
    private double currentRPM = 0.0;
    private double _increment = 0.05;
    private double currentPos;
    private double newPos;
    double maxRPM = 312; //Example desired RPM
    double currentTPS = 0;
    double leftTPR= 537.7; //Example for a REV HD HEX 40:1 spur motor
    double rightTPR = 384.5;
    //double rightTargetTPS = (desiredRPM / 60) * rPPR;
    //double leftTargetTPS =  (desiredRPM / 60) * lPPR;



    public BallLauncher(HardwareMap hardwareMap, Telemetry telemetry) {

        _hardwareMap = hardwareMap;
        _telemetry = telemetry;
        init();

    }






    private void init() {
        leftLaunchMotor = _hardwareMap.get(DcMotorEx.class, "launchLeft");
        rightLaunchMotor = _hardwareMap.get(DcMotorEx.class, "launchRight");
        rightLaunchMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftLaunchMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftLaunchMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightLaunchMotor.setDirection(DcMotorEx.Direction.FORWARD);

        linearServo = _hardwareMap.get(Servo.class, "linearServo");

//        rightLaunchMotor.setPower(_currentPower);
//        leftLaunchMotor.setPower(_currentPower);

    }

    public double currentSpeed() { return _currentPower; }
    public double currentRPM(){ return currentRPM; }
    public double currentTPS(){ return currentTPS; }
    public double getLeftLaunchPower(){ return leftLaunchMotor.getPower(); }
    public double getRightLaunchPower(){ return rightLaunchMotor.getPower(); }


    public void increaseLauncherSpeed(){

        double _newPower = _currentPower + _increment;
        if (_newPower >=1) _newPower = 1;
        _currentPower = _newPower;
        rightLaunchMotor.setPower(_currentPower);
        leftLaunchMotor.setPower(_currentPower);

        _telemetry.addData("increased launch speed", "");
        _telemetry.update();
    }

    public void decreaseLauncherSpeed(){
        double _newPower = _currentPower - _increment;
        if (_newPower <= 0) _newPower = 0;
        _currentPower = _newPower;
        rightLaunchMotor.setPower(_currentPower);
        leftLaunchMotor.setPower(_currentPower);
    }


    public double getCurrentPower()
    {
        return _currentPower;
    }

    public void aimLauncherUp()
    {
        currentPos = linearServo.getPosition();
        newPos = currentPos + INCREMENT;
        linearServo.setPosition(newPos);
        currentPos = newPos;

        if (currentPos >= 1){
            currentPos = 1;
            linearServo.setPosition(currentPos);
        }
    }

    public void aimLauncherDown()
    {
        currentPos = linearServo.getPosition();
        newPos = currentPos - INCREMENT;
        linearServo.setPosition(newPos);
        currentPos = newPos;

        if (currentPos <= 0){
            currentPos = 0;
            linearServo.setPosition(currentPos);
        }
    }



}
