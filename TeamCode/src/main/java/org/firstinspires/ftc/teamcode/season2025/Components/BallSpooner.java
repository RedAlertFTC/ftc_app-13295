package org.firstinspires.ftc.teamcode.season2025.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BallSpooner {

    private Servo servo;

    private final HardwareMap _hardwareMap;
    private final Telemetry _telemetry;

    private final long elapTrigger = 300; // ms

    private long startMs = 0;        // timing transitions
    private long stateStartMs = 0;   // state duration tracking

    private double serverStart = 1.0;
    private double serverFired = 0.75;

    private FiringEnum currentState = FiringEnum.REST;

    // Enum for tracking state
    private enum FiringEnum {
        REST,
        FIRE,
        FIRING,
        FIRED,
        RESETTING
    }

    public FiringEnum SpoonerState() {
        return currentState;
    }

    public BallSpooner(HardwareMap hardwareMap, Telemetry telemetry) {
        _telemetry = telemetry;
        _hardwareMap = hardwareMap;
        init();
    }

    public void init() {
        servo = _hardwareMap.get(Servo.class, "spooningServo");
        servo.setPosition(serverStart);
        currentState = FiringEnum.REST;
        stateStartMs = System.currentTimeMillis();
    }

    public boolean isBusy() {
        return currentState != FiringEnum.REST;
    }

    // Counters
    private long fireAttempt = 0;
    private long fireCounter = 0;

    public long getFireAttempt() {
        return fireAttempt;
    }

    public long getFireCounter() {
        return fireCounter;
    }

    public void fire() {
        fireAttempt++;
        if (currentState == FiringEnum.REST) {
            fireCounter++;
            transitionTo(FiringEnum.FIRE);
        }
    }

    /** MUST be called every loop */
    public void updateSpoonerState() {

        switch (currentState) {

            case REST:
                // idle
                break;

            case FIRE:
                servo.setPosition(serverFired);
                startMs = System.currentTimeMillis();
                transitionTo(FiringEnum.FIRING);
                break;

            case FIRING:
                if (System.currentTimeMillis() - startMs >= elapTrigger) {
                    transitionTo(FiringEnum.FIRED);
                }
                break;

            case FIRED:
                servo.setPosition(serverStart);
                startMs = System.currentTimeMillis();
                transitionTo(FiringEnum.RESETTING);
                break;

            case RESETTING:
                if (System.currentTimeMillis() - startMs >= elapTrigger) {
                    transitionTo(FiringEnum.REST);
                }
                break;
        }
    }

    private void transitionTo(FiringEnum newState) {
        currentState = newState;
        stateStartMs = System.currentTimeMillis();
    }

    public void resetCounters() {
        fireAttempt = 0;
        fireCounter = 0;
    }

    public double getStateDurationSec() {
        return (System.currentTimeMillis() - stateStartMs) / 1000.0;
    }
}
