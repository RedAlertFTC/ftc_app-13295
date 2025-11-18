package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.corelib.control.DebouncedButton;
import org.firstinspires.ftc.teamcode.corelib.control.DisasterGamePad;

@TeleOp(name = "GamepadTester", group = "Linear OpMode")
public class GamepadTester extends LinearOpMode {


    private DebouncedButton testButton;
    private DisasterGamePad _disasterGamePad1;
    private DisasterGamePad _disasterGamePad2;

    @Override
    public void runOpMode() {

        Gamepad1Setup();
        Gamepad2Setup();

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {

//            if (testButton.getRise()){
//                telemetry.addData("Gamepad1:A:Pressed", "true");
//            } else {
//                telemetry.addData("Gamepad1:A:Pressed", "false");
//            }

            if(gamepad1.a) {
                telemetry.addData("Gamepad1:A:Pressed", "true");
            } else {
                telemetry.addData("Gamepad1:A:Pressed", "false");
            }

            if(gamepad1.b) {
                telemetry.addData("Gamepad1:B:Pressed", "true");
            } else {
                telemetry.addData("Gamepad1:B:Pressed", "false");
            }

            if (gamepad2.a){
                telemetry.addData("Gamepad2:A:Pressed", "true");
            }
            else{
                telemetry.addData("Gamepad2:A:Pressed", "false");
            }

            if (gamepad2.b){
                telemetry.addData("Gamepad2:B:Pressed", "true");
            }
            else{
                telemetry.addData("Gamepad2:B:Pressed", "false");
            }

//            if(gp2BButton.getRise()) {
//                telemetry.addData("Gamepad2:B", "pressed");
//            } else  {
//                telemetry.addData("Gamepad2:B", "not pressed");
//            }
//            if(gp2AButton.getRise()) {
//                telemetry.addData("Gamepad2:A", "pressed");
//            } else  {
//                telemetry.addData("Gamepad2:A", "not pressed");
//            }
//            if(gp2YButton.getRise()) {
//                telemetry.addData("Gamepad2:Y", "pressed");
//            } else  {
//                telemetry.addData("Gamepad2:Y", "not pressed");
//            }
//            if(gp2XButton.getRise()) {
//                telemetry.addData("Gamepad2:X", "pressed");
//            } else  {
//                telemetry.addData("Gamepad2:X", "not pressed");
//            }

            telemetry.update();

        }
    }

    void Gamepad1Setup() {
        _disasterGamePad1 = new DisasterGamePad(gamepad1);
        testButton = new DebouncedButton(_disasterGamePad1.getAButton());
    }

    DebouncedButton gp2YButton;
    DebouncedButton gp2XButton;
    DebouncedButton gp2AButton;
    DebouncedButton gp2BButton;
    void Gamepad2Setup() {
        _disasterGamePad2 = new DisasterGamePad(gamepad2);
        gp2YButton = new DebouncedButton(_disasterGamePad2.getYButton());
        gp2XButton = new DebouncedButton(_disasterGamePad2.getXButton());
        gp2AButton = new DebouncedButton(_disasterGamePad2.getAButton());
        gp2BButton = new DebouncedButton(_disasterGamePad2.getBButton());
    }
}
