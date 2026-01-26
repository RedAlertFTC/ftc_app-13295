package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "LimelightGoalPositioningTest_PID", group = "Linear OpMode")
public class LimelightGoalPositioningTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;

    private Limelight3A limelight;

    // ---------------- PID constants ----------------
    private final double kP_turn = 0.01;
    private final double kI_turn = 0.0;
    private final double kD_turn = 0.0;
    private final double deadband = 1.0;   // degrees

    private double integral = 0;
    private double lastError = 0;

    @Override
    public void runOpMode() {

        // -------------------- Hardware Init --------------------
        frontLeftDrive = hardwareMap.get(DcMotor.class, "fl");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "bl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        backRightDrive  = hardwareMap.get(DcMotor.class, "br");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // AprilTag pipeline
        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();
        runtime.reset();

        // -------------------- Main Loop --------------------
        while (opModeIsActive()) {

            double turnPower = 0.0;

            if (gamepad1.left_bumper) {

                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()) {
                    telemetry.addData("tv", 1);
                    telemetry.addData("tx (deg)", "%.2f", result.getTx());
                    telemetry.addData("ty (deg)", "%.2f", result.getTy());

                    Pose3D botpose = result.getBotpose();
                    if (botpose != null) {
                        double x = botpose.getPosition().x;
                        double y = botpose.getPosition().y;
                        double z = botpose.getPosition().z;

                        double distanceCm = Math.sqrt(x * x + y * y + z * z) * 100.0;
                        double distanceIn = distanceCm * 0.393701;
                        telemetry.addData("AprilTag Distance (in)", "%.1f", distanceIn);
                    }

                    // -------- PID Turn Control --------
                    double error = result.getTx();
                    integral += error;
                    double derivative = error - lastError;

                    turnPower = (kP_turn * error)
                            + (kI_turn * integral)
                            + (kD_turn * derivative);

                    lastError = error;

                    if (Math.abs(error) < deadband) {
                        turnPower = 0;
                    }

                } else {
                    telemetry.addData("tv", 0);
                    // Search spin ONLY while bumper is held
                    turnPower = 0.2;
                }

            } else {
                // Left bumper NOT held → stop everything
                turnPower = 0.0;
                integral = 0;
                lastError = 0;
            }

            // ---------------- Drive Motors ----------------
            frontLeftDrive.setPower(turnPower);
            backLeftDrive.setPower(turnPower);
            frontRightDrive.setPower(-turnPower);
            backRightDrive.setPower(-turnPower);

            telemetry.addData("Left Bumper", gamepad1.left_bumper);
            telemetry.addData("Turn Power", "%.2f", turnPower);
            telemetry.addData("Run Time", runtime.toString());
            telemetry.update();
        }
    }
}
