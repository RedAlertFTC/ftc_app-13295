package org.firstinspires.ftc.teamcode.season2025;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BallSpooner
{
    private double spoonerPeak = 0.6;
    private Servo _spooningServo;
    private Telemetry _telemetry;
    private HardwareMap _hardwareMap;


    public void popBall() throws InterruptedException {
        // 1.Arm rises
        _spooningServo.setPosition(spoonerPeak);

        // 2.Wait period
        _spooningServo.wait(100);

        // 3.Arm falls
        _spooningServo.setPosition(0);
    }



    public BallSpooner(HardwareMap hardwareMap, Telemetry telemetry)
    {
        _telemetry = telemetry;
        _hardwareMap = hardwareMap;
        init();
    }

    private void init(){ _spooningServo = _hardwareMap.get(Servo.class, "spooningServo"); }

}
