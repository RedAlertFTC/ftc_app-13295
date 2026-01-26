package org.firstinspires.ftc.teamcode.season2025.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class BallSpooner
{
    private Servo servo;

    private final HardwareMap _hardwareMap;
    private final Telemetry _telemetry;
    long elapTrigger = 400;
    long startMs = 0;
    FiringEnum currentState = FiringEnum.REST;

    double serverStart = 1.0;
    double serverFired = .5;


    public BallSpooner(HardwareMap hardwareMap, Telemetry telemetry)
    {
        _telemetry = telemetry;
        _hardwareMap = hardwareMap;
        init();
    }

    public boolean isREST(){
        if (currentState == FiringEnum.REST){
            return true;
        }
        else{
            return false;
        }
    }



    // Enum for tracking target
    private enum FiringEnum {
        REST,
        FIRE,
        FIRING,
        FIRED,
        RESETTING
    }
    public void fire(){

        if (currentState == FiringEnum.REST){
            currentState = FiringEnum.FIRE;
        }

    }

    public void init() {

        servo = _hardwareMap.get(Servo.class, "spooningServo");
        double currentPos = servo.getPosition();

        long startMs = 0;

        //Reset the server to the start position
        servo.setPosition(serverStart);
    }

    public void updateSpoonerState() {

        _telemetry.addData("Servo:State", currentState);

        switch (currentState)
        {
            case REST:
                break;
            case FIRE:
                currentState = FiringEnum.FIRING;
                startMs = System.currentTimeMillis();
                servo.setPosition(serverFired);
                break;
            case FIRING:
                if(System.currentTimeMillis() > (startMs + elapTrigger)){
                    currentState = FiringEnum.FIRED;
                }
                break;
            case FIRED:
                currentState = FiringEnum.RESETTING;
                startMs = System.currentTimeMillis();
                servo.setPosition(serverStart);
                break;
            case RESETTING:
                if(System.currentTimeMillis() > (startMs + elapTrigger)){
                    currentState = FiringEnum.REST;
                }
                break;
            default:
                break;
        }
    }
}
