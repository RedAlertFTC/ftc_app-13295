package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Coop", group="TeleOp")
public class Coop extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;



    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "fl");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        backRightDrive = hardwareMap.get(DcMotor.class, "br");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //gamepad1, right wheel
            if (gamepad1.right_stick_y > 0){
                backRightDrive.setPower(500);
            }
            else if(gamepad1.right_stick_y < 0) {
                backRightDrive.setPower(-500);
            }
            else{
                backRightDrive.setPower(0);
            }

            //gamepad1, left wheel
            if (gamepad1.left_stick_y > 0){
                backLeftDrive.setPower(500);
            }
            else if(gamepad1.right_stick_y < 0) {
                backLeftDrive.setPower(-500);
            }
            else{
                backLeftDrive.setPower(0);
            }

            //gamepad2, right wheel
            if (gamepad2.right_stick_y > 0){
                frontRightDrive.setPower(500);
            }
            else if(gamepad2.right_stick_y < 0) {
                frontRightDrive.setPower(-500);
            }
            else{
                frontRightDrive.setPower(0);
            }

            //gamepad2, left wheel
            if (gamepad2.left_stick_y > 0){
                frontLeftDrive.setPower(500);
            }
            else if(gamepad2.right_stick_y < 0) {
                frontLeftDrive.setPower(-500);
            }
            else{
                frontLeftDrive.setPower(0);
            }



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
         //   telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
          //  telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.update();
        }
    }
}



