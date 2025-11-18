package org.firstinspires.ftc.teamcode.season2025.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.season2025.AllianceColor;

@Autonomous
public class DecodeV1AutoOp extends OpMode {

    enum State {
        WAIT_FOR_A,
        WAIT_FOR_B,
        WAIT_FOR_X,
        FINiSHED,

    }



    private AllianceColor _allianceColor;

    public AllianceColor getAllianceColor(){ return _allianceColor; }
    public void setAlliance(AllianceColor allianceColor){
        _allianceColor = allianceColor;
    }
    State state = State.WAIT_FOR_A;

    @Override
    public void init(){
        state = State.WAIT_FOR_A;
    }

    @Override
    public void loop(){
        telemetry.addData("Current State", state);

        switch (state){

            case WAIT_FOR_A:
                telemetry.addLine("To exit state, press A");
                if (gamepad1.a){
                    state = State.WAIT_FOR_B;
                }
                break;

            case WAIT_FOR_B:
                telemetry.addLine("To exit state, press B");
                if (gamepad1.b){
                    state = State.WAIT_FOR_X;
                }
                break;

            case WAIT_FOR_X:
                telemetry.addLine("To exit state, press X");
                if (gamepad1.x){
                    state = State.FINiSHED;
                }
                break;

            default:
                telemetry.addLine("Auto State Machine finished");
        }
    }


    private void hardwareInit(HardwareMap hardwareMap){

    }

}
