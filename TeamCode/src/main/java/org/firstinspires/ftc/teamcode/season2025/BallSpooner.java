package org.firstinspires.ftc.teamcode.season2025;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BallSpooner
{
    private Servo _spooningServo;
    private Telemetry _telemetry;
    private HardwareMap _hardwareMap;

    public BallSpooner(HardwareMap hardwareMap, Telemetry telemetry)
    {
        _telemetry = telemetry;
        _hardwareMap = hardwareMap;
        init();
    }

    private void init(){_spooningServo = _hardwareMap.get(Servo.class, "spooningServo"); }

}
