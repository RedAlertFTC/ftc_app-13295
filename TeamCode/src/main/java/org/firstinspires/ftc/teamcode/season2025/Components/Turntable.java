package org.firstinspires.ftc.teamcode.season2025.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turntable
{

    private enum TurntableState
    {
        RESTING,
        MOVING
    }

    private Servo _turntable;
    private Telemetry _telemetry;
    private HardwareMap _hardwareMap;

    private BallSpooner _ballSpooner;

    private int _currentSlot = 2;
    public double _currentPosition = 0.55;
    private int MAX_SLOT = 3;
    private int MIN_SLOT = 1;

    private double MIN_POSITION = 0;
    private double MIDDLE_POSITION = 0.475;
    private double MAX_POSITION = 0.92;

    double turntableIncrement;

    public int currentSlot(){ return _currentSlot; }

    public Turntable(HardwareMap hardwareMap, Telemetry telemetry)
    {
        _telemetry = telemetry;
        _hardwareMap = hardwareMap;
        init();
    }
    public  Turntable(HardwareMap hardwareMap, Telemetry telemetry, BallSpooner spooner) {
        _telemetry = telemetry;
        _hardwareMap = hardwareMap;
        _ballSpooner = spooner;
        init();
    }

    private void init()
    {
        _turntable = _hardwareMap.get(Servo.class, "turntable");
        _turntable.setPosition(MIDDLE_POSITION);
    }


//    public void rotateTurntable(int increment)
//    {
//        int newSlot = _currentSlot - increment;
//        _currentSlot = newSlot;
//
//    }



    public void increaseIndex() {


        if (_ballSpooner != null && !_ballSpooner.isREST()){
            return;
        }
        if (_currentSlot < MAX_SLOT){
            _currentSlot++;
        }

    }

    public void decreaseIndex() {

        if (_ballSpooner != null && !_ballSpooner.isREST()){
            return;
        }
        if(_currentSlot > MIN_SLOT){
            _currentSlot--;
        }

    }

    public void moveToIndex(int index){

    }

    public void updateCurrentSlot()
    {
        if(_currentSlot == 1){
            _turntable.setPosition(MIN_POSITION);
            _currentPosition = MIN_POSITION;
        }

        if(_currentSlot == 2){
            _turntable.setPosition(MIDDLE_POSITION);
            _currentPosition = MIDDLE_POSITION;
        }

        if(_currentSlot == 3){
            _turntable.setPosition(MAX_POSITION);
            _currentPosition = MAX_POSITION;
        }
    }

}
