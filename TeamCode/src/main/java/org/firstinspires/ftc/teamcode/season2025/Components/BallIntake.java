package org.firstinspires.ftc.teamcode.season2025.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BallIntake
{
    private DcMotor intakeMotor = null;
    private double intakeIncrement = 0.05;
    private Telemetry _telemetry;
    private HardwareMap _hardwareMap;
    private double _currentPower = 0.0;

    public BallIntake(HardwareMap hardwareMap, Telemetry telemetry)
    {
        _telemetry = telemetry;
        _hardwareMap = hardwareMap;
        init();
    }


    private void init()
    {
        intakeMotor = _hardwareMap.get(DcMotor.class, "intakeMotor");
    }
    public void increaseIntakeSpeed()
    {
        double newPower = _currentPower + intakeIncrement;
        if(newPower > 1.0) newPower = 1.0;
       intakeMotor.setPower(newPower);
    }
    public void decreaseIntakeSpeed()
    {
        double newPower = _currentPower - intakeIncrement;
        if(newPower < 0.0) newPower = 0.0;
        intakeMotor.setPower(newPower);
    }

    public void enableIntake(){
        intakeMotor.setPower(1);
    }

    public void disableIntake(){
        intakeMotor.setPower(0);
    }
}
