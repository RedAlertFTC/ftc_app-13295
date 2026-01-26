package org.firstinspires.ftc.teamcode.season2025.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LightController
{
    public Servo _lightOne;
    private LightColor lightColor;
    private HardwareMap _hardwareMap;
    private Telemetry _telemetry;

    public LightController(HardwareMap hardwareMap, Telemetry telemetry) {

        _hardwareMap = hardwareMap;
        _telemetry = telemetry;
       initialize();
    }

    public void SetColor(LightColor color){
        _lightOne.setPosition(color.getLightColor());
    }



    private void initialize()
    {
        _lightOne = _hardwareMap.get(Servo.class, "lightOne");
    }



}
