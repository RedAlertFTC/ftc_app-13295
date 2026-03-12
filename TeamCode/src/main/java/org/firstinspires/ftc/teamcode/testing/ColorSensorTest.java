package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@TeleOp(name="ColorSensorTest", group = "Robot")
public class ColorSensorTest extends LinearOpMode {
    private NormalizedColorSensor colorSensor;

    float gain = 2;

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Light Detected", ((OpticalDistanceSensor) colorSensor).getLightDetected());
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            if (gamepad1.a) {
                // Only increase the gain by a small amount, since this loop will occur multiple times per second.
                gain += 0.005;
            } else if (gamepad1.b && gain > 1) { // A gain of less than 1 will make the values smaller, which is not helpful.
                gain -= 0.005;
            }

            //Determining the amount of red, green, and blue
            telemetry.addData("Red", "%.3f", colors.red);
            telemetry.addData("Green", "%.3f", colors.green);
            telemetry.addData("Blue", "%.3f", colors.blue);
            telemetry.addData("Gain", gain);

            // Tell the sensor our desired gain value (normally you would do this during initialization,
            // not during the loop)
            colorSensor.setGain(gain);
            telemetry.update();
        }
    }
}