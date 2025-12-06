package org.firstinspires.ftc.teamcode.season2025.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.season2025.Components.BallAimer;
import org.firstinspires.ftc.teamcode.season2025.Components.BallLauncher;
import org.firstinspires.ftc.teamcode.season2025.Components.BallSpooner;
import org.firstinspires.ftc.teamcode.season2025.Components.Turntable;
import org.firstinspires.ftc.teamcode.season2025.FeatureFlags;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name="DecodeV2AutoOp", group = "Robot")
public class DecodeV2AutoOp extends LinearOpMode {

    private AutoStates autoStates;
    public autoAlliance _autoAlliance;

    public enum autoAlliance {
        RED,
        BLUE
    }

    private enum AutoStates{
        ESCAPING,
        FINDINGTAG,
        LAUNCHING,
        COLLECTING,
        FINAL,
    }

    final double DESIRED_DISTANCE = 50.0;  // Target distance to tag (inches)
    final double DESIRED_YAW = -10;

    final double SPEED_GAIN  = 0.02;
    final double STRAFE_GAIN = 0.015;
    final double TURN_GAIN   = 0.01;

    final double MAX_AUTO_SPEED = 0.5;
    final double MAX_AUTO_STRAFE= 0.5;
    final double MAX_AUTO_TURN  = 0.3;

    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;

    private static final boolean USE_WEBCAM = true;
    private static boolean FINDING_TARGET = true;
    private int goalAprilTagId = -1;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    private BallLauncher _ballLauncher;
    private Turntable _turntable;
    private BallSpooner _ballSpooner;
    private BallAimer _ballAimer;
    private ElapsedTime runtime = new ElapsedTime();
    private static final double RUN_TIME = 2;

    @Override
    public void runOpMode() {

        initAprilTag();

        frontLeftDrive = hardwareMap.get(DcMotor.class, "fl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bl");
        backRightDrive = hardwareMap.get(DcMotor.class, "br");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        _ballAimer = new BallAimer();

        if (FeatureFlags.launchEnabled()){
            _ballLauncher = new BallLauncher(hardwareMap, telemetry);
        }

        if (FeatureFlags.turnTableEnabled()){
            _turntable = new Turntable(hardwareMap, telemetry);
        }

        if (FeatureFlags.ballSpoonerEnabled()){
            _ballSpooner = new BallSpooner(hardwareMap, telemetry);
        }

        if (USE_WEBCAM)
            setManualExposure(6, 250);

        autoStates = AutoStates.ESCAPING;

        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()){
            switch (autoStates){
                case ESCAPING:
                    escapeWall();
                    break;
                case FINDINGTAG:
                    shootingPos();
                    break;
                case LAUNCHING:
                    if (FeatureFlags.launchEnabled() && FeatureFlags.ballSpoonerEnabled() && FeatureFlags.turnTableEnabled()){
                        launchAllBalls();
                    }
                    break;
                case COLLECTING:
                    break;
                case FINAL:
                    break;
            }

            telemetry.update();
        }
    }

    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) return;

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    public void moveRobot(double x, double y, double yaw) {
        double frontLeftPower    =  x - y - yaw;
        double frontRightPower   =  x + y + yaw;
        double backLeftPower     =  x + y - yaw;
        double backRightPower    =  x - y + yaw;

        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    private boolean shootingPos() {

        boolean targetFound = false;
        double drive = 0;
        double strafe = 0;
        double turn = 0;

        desiredTag = null;  // reset at start

        // Look for a valid tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && (goalAprilTagId < 0 || detection.id == goalAprilTagId)) {
                targetFound = true;
                desiredTag = detection;
                FINDING_TARGET = false;
                break;
            }
        }

        if (targetFound && desiredTag != null) {
            // Display telemetry for found tag
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);

            // Compute errors
            double rangeError = desiredTag.ftcPose.range - DESIRED_DISTANCE;
            double lateralError = desiredTag.ftcPose.range * Math.sin(Math.toRadians(desiredTag.ftcPose.bearing));
            double yawError = desiredTag.ftcPose.yaw;

            // Deadband check: stop if very close to desired distance and roughly aligned
            if (Math.abs(rangeError) < 1.0 && Math.abs(lateralError) < 1.0 && Math.abs(yawError) < 2.0) {
                stopMoving();
                autoStates = AutoStates.LAUNCHING;
                telemetry.addLine("Robot at desired position — stopping");
            } else {
                // Calculate movements
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                strafe = Range.clip(lateralError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                turn = Range.clip(yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                moveRobot(drive, strafe, turn);
            }

        } else {
            // No tag found — spin in place to search
            telemetry.addLine("No valid AprilTag detected — spinning to search");
            spin();
        }

        //telemetry.update();
        return targetFound;
    }




    public void stopMoving(){
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
        backLeftDrive.setPower(0);
    }

    private void spin(){
        frontLeftDrive.setPower(-0.5);
        backLeftDrive.setPower(-0.5);
        frontRightDrive.setPower(0.5);
        backRightDrive.setPower(0.5);
    }

    private void launchAllBalls(){
        _ballLauncher.setLauncherVelocity(500);
        _turntable.moveToIndex(1);
        _ballSpooner.fire();
        _turntable.moveToIndex(2);
        _ballSpooner.fire();
        _turntable.moveToIndex(3);
        _ballSpooner.fire();
        sleep(20);
        autoStates = AutoStates.COLLECTING;
    }

    private void escapeWall(){
        if (runtime.seconds() < RUN_TIME){
            frontLeftDrive.setPower(1);
            frontRightDrive.setPower(1);
            backLeftDrive.setPower(1);
            backRightDrive.setPower(1);
        } else {
            autoStates = AutoStates.FINDINGTAG;
        }
    }

    public void setAprilTagID(int aprilTagID){
        goalAprilTagId = aprilTagID;
    }
}
