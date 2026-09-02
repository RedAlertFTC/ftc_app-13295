package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.corelib.control.ButtonToggle;
import org.firstinspires.ftc.teamcode.corelib.control.DebouncedButton;
import org.firstinspires.ftc.teamcode.corelib.control.DebouncedGamepadButtons;
import org.firstinspires.ftc.teamcode.corelib.control.DisasterGamePad;

@TeleOp(name = "ServoTester", group = "Linear OpMode")
@Disabled
public class ServoTester extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    static final double INCREMENT   = 0.001;
    private Servo servo;

    private DebouncedButton testButton;
    private DisasterGamePad disasterGamepad1;
    private DebouncedGamepadButtons disasterGamepad1Buttons;

    // Button toggles for tracking control
    private ButtonToggle aButtonToggle;

    boolean firing = false;
    boolean testButtonPressed = false;

    // Enum for tracking target
    private enum FiringEnum {
        REST,
        FIRING,
        FIRED,
        RESETTING
    }

    @Override
    public void runOpMode() throws InterruptedException {


        FiringEnum currentState = FiringEnum.REST;

        disasterGamepad1 = new DisasterGamePad(gamepad1);
        disasterGamepad1Buttons = new DebouncedGamepadButtons(disasterGamepad1);

        //testButton = new DebouncedButton(disasterGamepad1.getYButton());

        // Initialize button toggles
        aButtonToggle = new ButtonToggle();

        servo = hardwareMap.get(Servo.class, "spooningServo");
        double currentPos = servo.getPosition();



        long elapTrigger = 300;
        long startMs = 0;

        waitForStart();
        runtime.reset();

        servo.setPosition(0);

        // Ramp motor speeds till stop pressed.
        while(opModeIsActive()) {

            telemetry.addData("Servo:State", currentState);
            telemetry.addData("Servo.Position", servo.getPosition());

            switch (currentState)
            {
                case REST:
                    if(disasterGamepad1Buttons.getaButton().getRise()) {
                        currentState = FiringEnum.FIRING;
                        startMs = System.currentTimeMillis();
                        servo.setPosition(1);
                    }
                    break;
                case FIRING:
                    if(System.currentTimeMillis() > (startMs + elapTrigger)){
                        currentState = FiringEnum.FIRED;
                    }
                    break;
                case RESETTING:
                    if(System.currentTimeMillis() > (startMs + elapTrigger)){
                        currentState = FiringEnum.REST;
                    }
                    break;
                case FIRED:
                    currentState = FiringEnum.RESETTING;
                    startMs = System.currentTimeMillis();
                    servo.setPosition(0);
                    break;
                default:
                    break;
            }

//            if (testButton.getRise() && !firing){
//                testButtonPressed = true;
//                firing = true;
//               //currentPos = currentPos + INCREMENT;
//                servo.setPosition(1);
//                //servo.setPosition(0);
//            }



            if (gamepad1.a){
                //currentPos = currentPos + INCREMENT;
                //servo.setPosition(1);
               // servo.setPosition(0);

            }
            if (gamepad1.a){
                //currentPos = currentPos - INCREMENT;
            }

            telemetry.update();

        }
    }
}
