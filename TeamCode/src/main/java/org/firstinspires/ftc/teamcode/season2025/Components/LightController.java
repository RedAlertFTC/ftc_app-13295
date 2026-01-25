package org.firstinspires.ftc.teamcode.season2025.Components;

import com.qualcomm.robotcore.hardware.Servo;

public class LightController
{
    public Servo _lightOne;

    private void setLightToRed(){
    // 0.277

        _lightOne.setPosition(0.277);
    }

    private void setLightToGreen(){
        // 0.5
        _lightOne.setPosition(0.5);
    }
}
