package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


@TeleOp(name="SnapToGoal", group="LinearOpMode" )
public class SnapToGoal extends LinearOpMode {
    private Limelight3A limelight;

    private DcMotorEx frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;

    private enum RobotStatus {
        ESCAPING,
        SPINNING,
        SNAPPING,
        POSITIONING
    }

    private RobotStatus robotStatus = RobotStatus.SPINNING;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();


        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "fl");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "bl");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "fr");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "br");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);



        while (opModeIsActive()) {

            // LIMELIGHT TELEMETRY AND DISTANCE CALCULATIONS
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();

                    // Position is in meters
                    double x = botpose.getPosition().x;
                    double y = botpose.getPosition().y;
                    double z = botpose.getPosition().z;

                    // Distances
                    double forwardDistanceMeters = z;
                    double totalDistanceMeters = Math.sqrt(x*x + y*y + z*z);

                    // Convert to inches
                    double forwardDistanceInches = forwardDistanceMeters * 39.37;
                    double totalDistanceInches = totalDistanceMeters * 39.37;

                    double distance = result.getBotpose().getPosition().z;


                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("Botpose", botpose.toString());
                    telemetry.addData("Forward Distance (in)", "%.1f", forwardDistanceInches);
                    telemetry.addData("Total Distance (in)", "%.1f", totalDistanceInches);

                    robotStatus = RobotStatus.SNAPPING;
                }
            }

            // STATE MACHINE
            switch (robotStatus) {
                case ESCAPING:
                    break;

                case SPINNING:
                    Spin();
                    break;

                case SNAPPING:
                    SnapAprilTag();
                    break;

                case POSITIONING:
                    PositionToAprilTag();
                    break;

            }
        }
    }


    private void Spin() {
        frontLeftDrive.setPower(0.5);
        backLeftDrive.setPower(0.5);
        frontRightDrive.setPower(-0.5);
        backRightDrive.setPower(-0.5);
    }

    private void TweakLeft(){
        frontLeftDrive.setPower(-0.2);
        backLeftDrive.setPower(-0.2);
        frontRightDrive.setPower(0.2);
        backRightDrive.setPower(0.2);
    }

    private void TweakRight(){
        frontLeftDrive.setPower(0.2);
        backLeftDrive.setPower(0.2);
        frontRightDrive.setPower(-0.2);
        backRightDrive.setPower(-0.2);
    }

    private void StopBot(){
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    private void Forward(){
        frontLeftDrive.setPower(0.5);
        backLeftDrive.setPower(0.5);
        frontRightDrive.setPower(0.5);
        backRightDrive.setPower(0.5);
    }
    private void Backward(){
        frontLeftDrive.setPower(-0.5);
        backLeftDrive.setPower(-0.5);
        frontRightDrive.setPower(-0.5);
        backRightDrive.setPower(-0.5);
    }
    private void SnapAprilTag() {

        StopBot();

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            if (result.getTx() > 0.05)
            {
                TweakLeft();
            }
            else if (result.getTx() < -0.05)
            {
                TweakRight();
            }

            if (result.getTx() < 0.05 && result.getTx() > -0.05){
                StopBot();
                robotStatus = RobotStatus.POSITIONING;
            }

        }
    }

    private void PositionToAprilTag(){
        StopBot();
        LLResult result = limelight.getLatestResult();
        double distance = result.getBotpose().getPosition().z;

        if (result != null && result.isValid()){
            if (distance > 60.05){
                Forward();
            }
            else if(distance < 59.95){
                Backward();
            }

            if (distance > 59.95 && distance < 60.05){
                StopBot();
            }
        }
    }
}






