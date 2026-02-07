package org.firstinspires.ftc.teamcode.season2025.Auto;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.season2025.Components.BallLauncher;
import org.firstinspires.ftc.teamcode.season2025.Components.BallSpooner;
import org.firstinspires.ftc.teamcode.season2025.Components.Turntable;
import org.firstinspires.ftc.teamcode.season2025.FeatureFlags;

@Autonomous(name="ShootingAuto", group = "Robot")
public class ShootingAuto extends LinearOpMode {

    private SparkFunOTOS myOtos;

    private enum FireState {
        IDLE,
        START_LAUNCHER,
        WAIT_FOR_LAUNCHER,
        FIRE,
        WAIT_FOR_FIRE,
        ADVANCE_TURNTABLE,
        WAIT_FOR_TURNTABLE,
        DONE
    }



    private FireState firingState = FireState.IDLE;

    private final ElapsedTime stateTimer = new ElapsedTime();

    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private boolean launchReady = false;
    private boolean strafeReady = false;

    private BallLauncher _ballLauncher;
    private Turntable _turntable;
    private BallSpooner _ballSpooner;

    long elapTrigger = 5000;
    long startMs = 0;

    private ElapsedTime runtime = new ElapsedTime();
    private static final double RUN_TIME = 3;

    @Override
    public void runOpMode() {


        frontLeftDrive = hardwareMap.get(DcMotor.class, "fl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bl");
        backRightDrive = hardwareMap.get(DcMotor.class, "br");
        myOtos = hardwareMap.get(SparkFunOTOS.class, "otos");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);



        if (FeatureFlags.launchEnabled()){
            _ballLauncher = new BallLauncher(hardwareMap, telemetry);
        }

        if (FeatureFlags.turnTableEnabled()){
            _turntable = new Turntable(hardwareMap, telemetry);
        }

        if (FeatureFlags.ballSpoonerEnabled()){
            _ballSpooner = new BallSpooner(hardwareMap, telemetry);
        }

        configureOtos();

        telemetry.update();
        waitForStart();

        startMs = System.currentTimeMillis();

        setExpiration(FireState.IDLE, 2000);

        while (opModeIsActive()){

            //_ballSpooner.updateSpoonerState();
            //ShootAllBalls();


            update();

        }
    }


    boolean isBusy = false;
    public boolean isBusy() {
        return isBusy;
    }



    public void update() {

        switch (firingState) {

            case START_LAUNCHER:
                //_ballLauncher.setLaunchPresetThree();
                stateTimer.reset();
                firingState = FireState.WAIT_FOR_LAUNCHER;
                break;

            case WAIT_FOR_LAUNCHER:
                //if (_ballLauncher.isReadyToFire(36.0)) {
                //    firingState = FireState.FIRE;
                //}
                firingState = FireState.FIRE;
                break;

            case IDLE:
                if(isExpired(FireState.IDLE)) {
                    firingState = FireState.FIRE;
                    setExpiration(FireState.FIRE, 2000);
                }
                break;
            case FIRE:

                _ballSpooner.fire();
                if(isExpired(FireState.FIRE)) {
                    firingState = FireState.IDLE;
                    setExpiration(FireState.IDLE, 500);
                }


                //stateTimer.reset();


//                if (!_ballSpooner.isBusy()) {
//                    _ballSpooner.fire();
//                    firingState = FireState.IDLE;
//                    //firingState = FireState.WAIT_FOR_FIRE;
//                }

                break;

            case WAIT_FOR_FIRE:
                if (!_ballSpooner.isBusy() && stateTimer.seconds() > 0.3) {
                    firingState = FireState.ADVANCE_TURNTABLE;
                }
                break;

            case ADVANCE_TURNTABLE:
                if (_turntable.currentSlot() < 3) {
                    _turntable.increaseIndex();
                    stateTimer.reset();
                    firingState = FireState.WAIT_FOR_TURNTABLE;
                } else {
                    firingState = FireState.DONE;
                }
                break;

            case WAIT_FOR_TURNTABLE:
                if (!_turntable.isBusy() && stateTimer.seconds() > 0.2) {
                    firingState = FireState.FIRE;
                }
                break;

            case DONE:
                // All balls fired — do nothing
                break;
        }

        // Always update mechanisms
        _ballSpooner.updateSpoonerState();
        _turntable.updateCurrentSlot();

        telemetry.addData("Fire State", firingState);
        telemetry.addData("Turntable Slot", _turntable.currentSlot());
        telemetry.addData("Spooner State", _ballSpooner.SpoonerState());
        telemetry.update();
    }


    private FireState _checkState = null;
    private long _expiration = 0;
    private void setExpiration(FireState state, long durationMs) {

        _checkState = state;
        _expiration = durationMs + System.currentTimeMillis();
    }

    private boolean isExpired(FireState checkState) {
         return (checkState == _checkState && System.currentTimeMillis() > _expiration);
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

    private void StrafeLeft(){
        frontLeftDrive.setPower(-0.5);
        backLeftDrive.setPower(0.5);
        frontRightDrive.setPower(0.5);
        backRightDrive.setPower(-0.5);
    }

    public void Forward(){
        frontLeftDrive.setPower(0.5);
        backLeftDrive.setPower(0.5);
        frontRightDrive.setPower(0.5);
        backRightDrive.setPower(0.5);
    }


    private void ShootAllBalls(){

        _ballLauncher.setLaunchPresetOne();
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
