package org.firstinspires.ftc.teamcode.season2025;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
        leftLaunchMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftLaunchMotor.setDirection(DcMotorEx.Direction.FORWARD);

        rightLaunchMotor = _hardwareMap.get(DcMotorEx.class, "launchRight");
        rightLaunchMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightLaunchMotor.setDirection(DcMotorEx.Direction.REVERSE);

        linearServo = _hardwareMap.get(Servo.class, "linearServo");

    }

    public double currentSpeed() { return _currentPower; }
    public double currentRPM(){ return currentRPM; }
    public double currentTPS(){ return currentTPS; }
    public void start(double speed)
    {
        if(_currentPower < 1.0 && speed < 1.0) {
            setMotorSpeed(speed);
            _currentPower = speed;
        }
    }
    public void speedUpLauncher()
    {
        double newPower = _currentPower + _increment;
        if(newPower > 1.0) newPower = 1.0;
        setMotorSpeed(newPower);
    }
    public void speedUp(double increment)
    {
        double newPower = _currentPower + increment;
        if(newPower > 1.0) newPower = 1.0;
        setMotorSpeed((newPower));
    }

    public void stop()
    {
       setMotorSpeed(0);
    }

    public void slowDownLauncher()
    {
        double newPower = _currentPower - _increment;
        if(newPower < 0.0 ) newPower = 0.0;
        setMotorSpeed(newPower);
    }
    public void slowDown(double increment)
    {
        double newPower = _currentPower - increment;
        if(newPower < 0.0) newPower = 0.0;
        setMotorSpeed(newPower);
    }


    public double getCurrentPower()
    {
        return _currentPower;
    }

    private void setMotorSpeed(double power)
    {;
        double RPM = convertToRPM(power);
        currentTPS = convertToTPS(RPM);
       // leftLaunchMotor.setVelocity(currentTPS);
        //rightLaunchMotor.setVelocity(currentTPS);
        _currentPower = power;
        currentRPM = RPM;
        _telemetry.addData("LaunchPower", _currentPower);

    }

    public void aimLauncherUp()
    {
        currentPos = linearServo.getPosition();
        newPos = currentPos + INCREMENT;
        linearServo.setPosition(newPos);
        currentPos = newPos;

        if (currentPos >= 1){
            linearServo.setPosition(1);
        }
    }

    public void aimLauncherDown()
    {
        currentPos = linearServo.getPosition();
        newPos = currentPos - INCREMENT;
        linearServo.setPosition(newPos);
        currentPos = newPos;

        if (currentPos <= 0){
            linearServo.setPosition(0);
        }
    }

    private double convertToTPS(double RPM)
    {
         return (RPM / 60) * 500;
    }

    private double convertToRPM(double power)
    {
        return maxRPM * power;
    }

}
