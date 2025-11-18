package org.firstinspires.ftc.teamcode.season2025.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turntable
{
    private Servo _turntable;
    private Telemetry _telemetry;
    private HardwareMap _hardwareMap;
    private int _currentSlot = 1;
    public double _currentPosition = 0;

    double turntableIncrement;

    public int currentSlot(){ return _currentSlot; }

    public Turntable(HardwareMap hardwareMap, Telemetry telemetry)
    {
        _telemetry = telemetry;
        _hardwareMap = hardwareMap;
        init();
    }

    private void init()
    {
        _turntable = _hardwareMap.get(Servo.class, "turntable");
    }


    public void rotateTurntable(int increment)
    {
        int newSlot = _currentSlot - increment;
        _currentSlot = newSlot;

    }

    public void updateCurrentSlot()
    {
       _currentPosition =  _turntable.getPosition();


        if(_currentSlot == 1){
            _turntable.setPosition(0);
        }

        if(_currentSlot == 2){
            _turntable.setPosition(0.5);
        }

        if(_currentSlot == 3){
            _turntable.setPosition(1);
        }

        if(_currentPosition >= 1){
            _currentSlot = 3;
        }

        if (_currentSlot <= 0){
            _currentSlot = 1;
        }
    }

}
