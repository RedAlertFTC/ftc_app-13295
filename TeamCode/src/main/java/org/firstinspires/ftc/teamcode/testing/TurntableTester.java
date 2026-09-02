package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.corelib.control.DebouncedButton;
import org.firstinspires.ftc.teamcode.corelib.control.DisasterGamePad;

@TeleOp(name = "Turntable Tester", group = "Testing")
@Disabled
public class TurntableTester extends LinearOpMode {

    static final double INCREMENT   = 0.5;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    Servo servo;
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;
    private DisasterGamePad _driverOneGamepad;
    private DebouncedButton _increaseIndex;
    private DebouncedButton _decreaseIndex;

    @Override
    public void runOpMode() {

        // Connect to servo (Assume Robot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        servo = hardwareMap.get(Servo.class, "turntable");

        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        servo.setPosition(0.5);
        waitForStart();

        _driverOneGamepad = new DisasterGamePad(gamepad1);

        _increaseIndex = new DebouncedButton(_driverOneGamepad.getRightBumper());
        _decreaseIndex = new DebouncedButton(_driverOneGamepad.getLeftBumper());

        // Scan servo till stop pressed.
        while(opModeIsActive()){

            // slew the servo, according to the rampUp (direction) variable.
            if (_increaseIndex.getRise()) {
                // Keep stepping up until we hit the max value.
                position += INCREMENT ;
                if (position >= MAX_POS ) {
                    position = MAX_POS;
                    rampUp = !rampUp;   // Switch ramp direction
                }
            }
            else if (_decreaseIndex.getRise())
            {
                // Keep stepping down until we hit the min value.
                position -= INCREMENT ;
                if (position <= MIN_POS ) {
                    position = MIN_POS;
                    rampUp = !rampUp;  // Switch ramp direction
                }
            }

            // Display the current value
            telemetry.addData("Servo Position", "%5.2f", position);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the servo to the new position and pause;
            servo.setPosition(position);
            sleep(CYCLE_MS);
            //idle();

        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
