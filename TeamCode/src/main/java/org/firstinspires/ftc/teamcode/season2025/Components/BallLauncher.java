package org.firstinspires.ftc.teamcode.season2025.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
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
    private double INCREMENT = 0.05;

    private Servo linearServo;
    private double _maxPower = 1.0;
    private double _currentPower = 0.0;
    private double currentRPM = 0.0;
    private double MAX_RPM = 100;
    private double MIN_RPM = 5;
    private double RPM_INCREMENT = 5;
    private double _increment = 0.05;
    private double currentPos = 0.5;
    private double newPos;
    double currentTPS = 0;
    private double firingTPS = 500.0;


    double MAX_TPS = 2800;
    double MIN_TPS = 700;
    //double TPS_INCREMENT = 100;
    double TPS_INCREMENT = 50;


    double TPR = 28.0;

    //double leftTPR= 28.0; //Example for a REV HD HEX 40:1 spur motor
    //double rightTPR = 28.0;


    // Define your motor's TICKS_PER_REV
    // Replace with the actual value for your motor (e.g., from goBILDA specs)
    public static final double TICKS_PER_REV = 28; // Example for a goBILDA 5202 motor


    public BallLauncher(HardwareMap hardwareMap, Telemetry telemetry) {

        _hardwareMap = hardwareMap;
        _telemetry = telemetry;
        init();
    }

    private void init() {
        leftLaunchMotor = _hardwareMap.get(DcMotorEx.class, "launchLeft");
        rightLaunchMotor = _hardwareMap.get(DcMotorEx.class, "launchRight");

        leftLaunchMotor.setDirection(DcMotorEx.Direction.FORWARD);
        leftLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLaunchMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        rightLaunchMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLaunchMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        linearServo = _hardwareMap.get(Servo.class, "linearServo");
        linearServo.setPosition(currentPos);

//        rightLaunchMotor.setPower(_currentPower);
//        leftLaunchMotor.setPower(_currentPower);

    }

    public double currentSpeed() { return _currentPower; }
    public double getCurrentRPM() { return currentRPM; }
    public double getCurrentTPS() { return currentTPS; }
    public double getLeftLaunchPower(){ return leftLaunchMotor.getPower(); }
    public double getRightLaunchPower(){ return rightLaunchMotor.getPower(); }
    public double getCurrentPos() { return currentPos; }


    public void increaseLauncherSpeed(){

        double _newPower = _currentPower + _increment;
        if (_newPower >=1) _newPower = 1;
        _currentPower = _newPower;
        rightLaunchMotor.setPower(_currentPower);
        leftLaunchMotor.setPower(_currentPower);

       // _telemetry.addData("increased launch speed", "");
        _telemetry.update();
    }


    public double getLeftVelocity() {
        return leftLaunchMotor.getVelocity();
    }
    public double getRightVelocity() {
        return rightLaunchMotor.getVelocity();
    }

    public void setLauncherVelocity(double inputTPS){
        currentTPS = inputTPS;
        leftLaunchMotor.setVelocity(currentTPS);
        rightLaunchMotor.setVelocity(currentTPS);

    }
    public void increaseLauncherSpeedByRPM()
    {
        currentRPM += RPM_INCREMENT;
        if(currentRPM < MIN_RPM) currentRPM = MIN_RPM;
        if(currentRPM > MAX_RPM) currentRPM = MAX_RPM;
        currentTPS = ((currentRPM / 2.14)/ 60) * TICKS_PER_REV;


        leftLaunchMotor.setVelocity(currentTPS);
        rightLaunchMotor.setVelocity(currentTPS);
    }

    public void increaseLauncherSpeedByTPS()
    {
        currentTPS += TPS_INCREMENT;
        if(currentTPS < MIN_TPS) currentTPS = MIN_TPS;
        if(currentTPS > MAX_TPS) currentTPS = MAX_TPS;

        leftLaunchMotor.setVelocity(currentTPS);
        rightLaunchMotor.setVelocity(currentTPS);
    }

    public void decreaseLauncherSpeedByPower(){
        double _newPower = _currentPower - _increment;
        if (_newPower <= 0) _newPower = 0;
        _currentPower = _newPower;
        rightLaunchMotor.setPower(_currentPower);
        leftLaunchMotor.setPower(_currentPower);
    }

    public void decreaseLauncherSpeedByRPM()
    {
        currentRPM -= RPM_INCREMENT;
        if(currentRPM < MIN_RPM) currentRPM = 0;

        if(currentRPM > 0) {
            currentTPS = ((currentRPM/ 2.14)/ 60) * TICKS_PER_REV;

            leftLaunchMotor.setVelocity(currentTPS);
            rightLaunchMotor.setVelocity(currentTPS);
        } else {
            leftLaunchMotor.setVelocity(0);
            rightLaunchMotor.setVelocity(0);
        }
    }

    public void decreaseLauncherSpeedByTPS()
    {
        currentTPS -= TPS_INCREMENT;
        if(currentTPS < MIN_TPS) currentTPS = 0;

        leftLaunchMotor.setVelocity(currentTPS);
        rightLaunchMotor.setVelocity(currentTPS);
    }

    public double getCurrentPower()
    {
        return _currentPower;
    }

    public void aimLauncherUp()
    {
        currentPos += INCREMENT;
        if(currentPos > 0.75) currentPos = 0.75;
        linearServo.setPosition(currentPos);

    }
    public void aimLauncherDown()
    {
        currentPos -= INCREMENT;
        if(currentPos < 0.25) currentPos = 0.25;
        linearServo.setPosition(currentPos);

    }

    public void setLauncherAngle(double pos){

        linearServo.setPosition(pos);
    }

    public void setLaunchPresetOne(){
        currentTPS = 1150;
        currentPos = 0.25;
        linearServo.setPosition(currentPos);
        leftLaunchMotor.setVelocity(currentTPS);
        rightLaunchMotor.setVelocity(currentTPS);
    }

    public void setLaunchPresetTwo(){
        currentTPS = 1300;
        currentPos = 0.45;
        linearServo.setPosition(currentPos);
        leftLaunchMotor.setVelocity(currentTPS);
        rightLaunchMotor.setVelocity(currentTPS);
    }

    public void setLaunchPresetThree(){
        currentTPS = 1730;
        currentPos = 0.55;
        linearServo.setPosition(currentPos);
        leftLaunchMotor.setVelocity(currentTPS);
        rightLaunchMotor.setVelocity(currentTPS);
    }

    public void turnOffLauncher(){
        currentTPS = 0;
        currentPos = 0.4;
        linearServo.setPosition(currentPos);
        leftLaunchMotor.setVelocity(currentTPS);
        rightLaunchMotor.setVelocity(currentTPS);
    }

    public boolean isReadyToFire(double distance) {
        //TODO: determine when it is ready

        return true;
    }
}
