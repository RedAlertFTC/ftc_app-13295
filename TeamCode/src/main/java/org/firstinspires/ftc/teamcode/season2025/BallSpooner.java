package org.firstinspires.ftc.teamcode.season2025;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BallSpooner
{
    private double spoonerPeak = 1.0;
    private Servo _spooningServo;
    private Telemetry _telemetry;
    private HardwareMap _hardwareMap;
    private boolean isPopping;
    private boolean isReturning;
    private  boolean isResting;
    private SpoonerState spoonerStatus;

    public double getSpoonerPosition(){ return _spooningServo.getPosition(); }
    public SpoonerState getSpoonerStatus() { return spoonerStatus; }


    public enum SpoonerState{
        Firing,
        Resting,
        Fired;
    }

    public void popBall() {
        spoonerStatus = SpoonerState.Firing;
        _spooningServo.setPosition(1);
    }

    public void returnBallSpooner() {

        spoonerStatus = SpoonerState.Fired;
        _spooningServo.setPosition(0);
    }

    public void restBallSpooner(){
        spoonerStatus = SpoonerState.Resting;
    }

    public void checkBallSpooner(){
        if (_spooningServo.getPosition() >= spoonerPeak){
            _spooningServo.setPosition(spoonerPeak);
        }
       // _spooningServo.setPosition(0);
        _spooningServo.setPosition(0);
    }

    public BallSpooner(HardwareMap hardwareMap, Telemetry telemetry)
    {
        _telemetry = telemetry;
        _hardwareMap = hardwareMap;
        init();
    }

    private void init(){
        _spooningServo = _hardwareMap.get(Servo.class, "spooningServo");
        spoonerStatus = SpoonerState.Resting;
        _spooningServo.setPosition(0);
    }

}