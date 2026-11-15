package org.firstinspires.ftc.teamcode.season2025.Auto;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.season2025.Components.BallAimer;
import org.firstinspires.ftc.teamcode.season2025.Components.BallLauncher;
import org.firstinspires.ftc.teamcode.season2025.Components.BallSpooner;
import org.firstinspires.ftc.teamcode.season2025.Components.Turntable;
import org.firstinspires.ftc.teamcode.season2025.FeatureFlags;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name="DecodeV2AutoOp", group = "Robot")
@Disabled
public class DecodeV2AutoOp extends LinearOpMode {

    private AutoStates autoStates;
    public autoAlliance _autoAlliance;
    private SparkFunOTOS myOtos;

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
    private AprilTagSingleDetection desiredTag = null;

    private BallLauncher _ballLauncher;
    private Turntable _turntable;
    private BallSpooner _ballSpooner;
    private BallAimer _ballAimer;
    private ElapsedTime runtime = new ElapsedTime();
    private static final double RUN_TIME = 3;

    @Override
    public void runOpMode() {

        initAprilTag();

        frontLeftDrive = hardwareMap.get(DcMotor.class, "fl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bl");
        backRightDrive = hardwareMap.get(DcMotor.class, "br");
        myOtos = hardwareMap.get(SparkFunOTOS.class, "otos");

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

        configureOtos();

        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()){

            SparkFunOTOS.Pose2D pos = myOtos.getPosition();

            switch (autoStates){

                case ESCAPING:
                    escapeWall();
                    break;
                case FINDINGTAG:
                    //shootingPos();
                    break;
                case LAUNCHING:
                    if (FeatureFlags.launchEnabled() && FeatureFlags.ballSpoonerEnabled() && FeatureFlags.turnTableEnabled()){
                       // launchAllBalls();
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
        for (AprilTagDetection rawDetection : currentDetections) {
            // SDK 12: id/metadata/center only exist on single-tag detections (clusters are skipped)
            if (!(rawDetection instanceof AprilTagSingleDetection)) continue;
            AprilTagSingleDetection detection = (AprilTagSingleDetection) rawDetection;
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

                moveRobot(-drive, -strafe, turn);
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
        _ballLauncher.setLaunchPresetTwo();
        sleep(2000);
        _turntable.moveToIndex(1);
        sleep(1000);
        _ballSpooner.fire();
        sleep(1000);
        _turntable.moveToIndex(2);
        sleep(1000);
        _ballSpooner.fire();
        sleep(1000);
        _turntable.moveToIndex(3);
        sleep(1000);
        _ballSpooner.fire();
    }

    private void escapeWall(){
        if (myOtos.getPosition().x <= 35.5 ){

            frontLeftDrive.setPower(-1);
            frontRightDrive.setPower(-1);
            backLeftDrive.setPower(-1);
            backRightDrive.setPower(-1);

        }
        else {
            stopMoving();
        }

        if (myOtos.getPosition().y <= 9.5) {

            frontLeftDrive.setPower(1);
            frontRightDrive.setPower(-1);
            backLeftDrive.setPower(-1);
            backRightDrive.setPower(1);

        }
        else {
            stopMoving();
        }
    }

    public void setAprilTagID(int aprilTagID){
        goalAprilTagId = aprilTagID;
    }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0.5, -1.25, 90); // -90 h for "correct" values
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        myOtos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        myOtos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();







    }
}
