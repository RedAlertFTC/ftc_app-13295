package org.firstinspires.ftc.teamcode.season2025.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class BallSpooner
{
    private Servo servo;

    private final HardwareMap _hardwareMap;
    private final Telemetry _telemetry;
    long elapTrigger = 500;
    long startMs = 0;

    double stateTime = 0;

    FiringEnum currentState = FiringEnum.REST;

    double serverStart = 1.0;
    double serverFired = .75;

    public FiringEnum SpoonerState(){ return currentState; }


    public BallSpooner(HardwareMap hardwareMap, Telemetry telemetry)
    {
        _telemetry = telemetry;
        _hardwareMap = hardwareMap;
        init();
    }

    public boolean isBusy(){
        return currentState != FiringEnum.REST;
    }



    // Enum for tracking target
    private enum FiringEnum {
        REST,
        FIRE,
        FIRING,
        FIRED,
        RESETTING
    }

    long fireAttempt = 0;
    public long getFireAttempt()
    {
        return fireAttempt;
    }
    long fireCounter = 0;

    public long getFireCounter() {
        return fireCounter;
    }

    public void fire() {
        fireAttempt++;
        if (currentState == FiringEnum.REST){
            fireCounter++;
            currentState = FiringEnum.FIRE;
        }
    }

    public void init()
    {
        servo = _hardwareMap.get(Servo.class, "spooningServo");
        double currentPos = servo.getPosition();

        long startMs = 0;

        //Reset the server to the start position
        servo.setPosition(serverStart);
    }

    public void updateSpoonerState() {

        //_telemetry.addData("Servo:State", currentState);

        switch (currentState)
        {
            case REST:
                stateTime = System.currentTimeMillis();
                break;
            case FIRE:
                stateTime = System.currentTimeMillis();
                currentState = FiringEnum.FIRING;
                startMs = System.currentTimeMillis();
                servo.setPosition(serverFired);
                break;
            case FIRING:
                stateTime = System.currentTimeMillis();
                if(System.currentTimeMillis() > (startMs + elapTrigger)){
                    currentState = FiringEnum.FIRED;
                }
                break;
            case FIRED:
                stateTime = System.currentTimeMillis();
                startMs = System.currentTimeMillis();
                currentState = FiringEnum.RESETTING;
                servo.setPosition(serverStart);
                break;
            case RESETTING:
                stateTime = System.currentTimeMillis();
                if(System.currentTimeMillis() > (startMs + elapTrigger)){
                    currentState = FiringEnum.REST;
                }
                break;
            default:
                stateTime = System.currentTimeMillis();
                break;
        }
    }

    public void resetCounters() {
        fireAttempt = 0;
        fireCounter = 0;
    }

    public double getStateDurationSec() {
        return (System.currentTimeMillis() - stateTime) / 1000;
    }
}
