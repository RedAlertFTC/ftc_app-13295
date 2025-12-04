package org.firstinspires.ftc.teamcode.testing;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "DecodeTestHardware", group = "Robot")
public class DecodeHardwareTester extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();

        int selector = 0;

        // Initialize motors
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, "fl"); // motor 2
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "fr"); // motor 3
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, "bl"); // motor 0
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "br"); // motor 1


        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor"); // motor 0
        DcMotorEx leftLaunchMotor = hardwareMap.get(DcMotorEx.class, "launchLeft"); // motor 1
        DcMotorEx rightLaunchMotor = hardwareMap.get(DcMotorEx.class, "launchRight"); // motor 2

        //CRServo collectServo = hardwareMap.get(CRServo.class, "collect"); // servo 0
//        Servo deliveryServo = hardwareMap.get(Servo.class, "Delivery");

        Servo spooningServo = hardwareMap.get(Servo.class, "spooningServo");
        Servo turntableServo = hardwareMap.get(Servo.class, "turntable");
        Servo linearServo = hardwareMap.get(Servo.class, "linearServo");

//
//        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
//        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");
//
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLaunchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean aButtonDown = false;
        boolean bButtonDown = false;
        String selectedHardware = "";
        DcMotorEx selectedMotor = null;

        while (opModeIsActive()) {

            double leftStickY = -gamepad1.left_stick_y;
            boolean aButton = gamepad1.a;
            boolean bButton = gamepad1.b;

            if (aButton) {
                aButtonDown = true;
            }
            if (!aButton && aButtonDown) {
                aButtonDown = false;
                selector++;
            }
            if (bButton) {
                bButtonDown = true;
            }
            if (!bButton && bButtonDown) {
                bButtonDown = false;
                selector--;
                if (selector == -1) selector = 8;
            }

            telemetry.addData("Selection", selector);
            telemetry.addData("leftStickY", leftStickY);

            switch (selector) {
                case 0:
                    selectedHardware = "Front Left Motor";
                    selectedMotor = frontLeftMotor;
                    break;
                case 1:
                    selectedHardware = "Back Right Motor";
                    selectedMotor = backRightMotor;
                    break;
                case 2:
                    selectedHardware = "Front Right Motor";
                    selectedMotor = frontRightMotor;
                    break;
                case 3:
                    selectedHardware = "Back Left Motor";
                    selectedMotor = backLeftMotor;
                    break;
                case 4:
                    selectedHardware = "Intake Motor";
                    selectedMotor = intakeMotor;
                    intakeMotor.setPower(leftStickY);
                    break;
                case 5:
                    selectedHardware = "Left Launch Motor";
                    selectedMotor = leftLaunchMotor;
                    leftLaunchMotor.setPower(leftStickY);
                    break;
                case 6:
                    selectedHardware = "Right Launch Motor";
                    selectedMotor = rightLaunchMotor;
                    rightLaunchMotor.setPower(leftStickY);
                    break;
                case 7:
                    selectedHardware = "Spoon Servo";
                    spooningServo.setPosition(leftStickY);
                    telemetry.addData("Spoon servo: ", "%.2f", leftStickY);
                    break;
                case 8:
                    selectedHardware = "Spindexer Servo";
                    turntableServo.setPosition(leftStickY);
                    telemetry.addData("Turntable servo: ", "%.2f", leftStickY);
                    break;
                case 9:
                    selectedHardware = "Linear Server";
                    linearServo.setPosition(leftStickY);
                    telemetry.addData("Linear servo: ", "%.2f", leftStickY);
                    break;
//                case 7:
//                    selectedHardware = "Winch Motor";
//                    selectedMotor = rightLaunchMotor;
//                    break;
//                case 8:
//                    selectedHardware = "Hang Motor";
//                    selectedMotor = hangMotor;
//                    break;
//                case 7:
//                    deliveryServo.setPosition(leftStickY);
//                    telemetry.addData("Delivery servo: ", "%.2f", leftStickY);
//                    break;
//                case 8:
//                    NormalizedRGBA colors = colorSensor.getNormalizedColors();
//                    telemetry.addLine()
//                            .addData("Red", "%.3f", colors.red)
//                            .addData("Green", "%.3f", colors.green)
//                            .addData("Blue", "%.3f", colors.blue);
//                    break;
                default:
                    selector = 0;
                    break;
            }
            if (selectedHardware.toLowerCase().contains("motor")) {
                selectedMotor.setPower(leftStickY);
                telemetry.addData(selectedHardware, "");
                telemetry.addData("Power: ", "%.2f", selectedMotor.getPower());
                telemetry.addData("Encoder: ", "%d", selectedMotor.getCurrentPosition());
                telemetry.addData("Velocity: ", "%.2f", selectedMotor.getVelocity());
                telemetry.addData("Current: ", "%.2f", selectedMotor.getCurrent(CurrentUnit.MILLIAMPS));
            }

            telemetry.update();
        }
    }
}