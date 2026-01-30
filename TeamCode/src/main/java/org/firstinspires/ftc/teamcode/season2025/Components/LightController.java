package org.firstinspires.ftc.teamcode.season2025.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LightController
{
    public Servo _lightOne;
    public Servo _lightTwo;
    private LightColor lightColor;
    private HardwareMap _hardwareMap;
    private Telemetry _telemetry;

    public LightController(HardwareMap hardwareMap, Telemetry telemetry) {

        _hardwareMap = hardwareMap;
        _telemetry = telemetry;
       initialize();
    }

    public void SetLightOne(LightColor color){
        _lightOne.setPosition(color.getLightColor());
    }

    public void SetLightTwo(LightColor color){
        _lightTwo.setPosition(color.getLightColor());
    }



    private void initialize()
    {

        _lightOne = _hardwareMap.get(Servo.class, "lightOne");
        _lightTwo = _hardwareMap.get(Servo.class, "lightTwo");
    }



}
