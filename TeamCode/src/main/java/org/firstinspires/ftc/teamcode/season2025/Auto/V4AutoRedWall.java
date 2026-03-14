package org.firstinspires.ftc.teamcode.season2025.Auto;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.season2025.Components.AprilTagHone;
import org.firstinspires.ftc.teamcode.season2025.Components.BallLauncher;
import org.firstinspires.ftc.teamcode.season2025.Components.BallSpooner;
import org.firstinspires.ftc.teamcode.season2025.Components.Turntable;
import org.firstinspires.ftc.teamcode.season2025.Components.ShooterSequence;
import org.firstinspires.ftc.teamcode.season2025.FeatureFlags;

@Autonomous(name="V4AutoRedWall", group = "Robot")
public class V4AutoRedWall extends LinearOpMode {

    private SparkFunOTOS myOtos;


    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private boolean launchReady = false;
    private boolean strafeReady = false;

    private BallLauncher _ballLauncher;
    private Turntable _turntable;
    private BallSpooner _ballSpooner;
    private AprilTagHone aprilTagHone;
    private ShooterSequence shooterSequence;

    private ElapsedTime runtime = new ElapsedTime();
    private static final double RUN_TIME = 3;

    // Change this to the AprilTag ID you want to hone in on; all other tags will be ignored.
    private static final int TARGET_TAG_ID = 24;

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

        // Sequence: drive forward 2 inches using OTOS, hone to AprilTag, then shoot 3 balls
        if (opModeIsActive()) {
            // Ensure components for honing and shooting are initialized
            aprilTagHone = new AprilTagHone();
            aprilTagHone.initAll(hardwareMap, "limelight", "fl", "bl", "fr", "br");
            // Require fiducial ID 24 for positioning; hone will search until it sees ID24
            aprilTagHone.setRequiredFiducialId(TARGET_TAG_ID);
            // Use a gentle search rotation so the robot slowly scans for ID24 instead of aggressively spinning
            aprilTagHone.setSearchTurnPower(0.12);
            aprilTagHone.start();

            if (_ballLauncher != null && _turntable != null && _ballSpooner != null) {
                shooterSequence = new ShooterSequence(_ballLauncher, _turntable, _ballSpooner, telemetry);
                shooterSequence.DEFAULT_SPINUP_MS = 1500;
                shooterSequence.DEFAULT_BETWEEN_SHOTS_MS = 1000;
                shooterSequence.DEFAULT_SPOONER_TIMEOUT_MS = 1900;
                shooterSequence.DEFAULT_TURNTABLE_SETTLE_MS = 950;



            }
            if (_ballLauncher != null) {
                _ballLauncher.setLaunchPresetAuto();
            }

            // Simple sequence: drive backward 2 inches, hone to ID24 (red goal), then shoot 3 balls.
            telemetry.addLine("Driving backward 2 inches...");
            telemetry.update();
            runtime.reset();
            while (opModeIsActive()) {
                SparkFunOTOS.Pose2D pos = myOtos.getPosition();
                telemetry.addData("X", pos.x);
                telemetry.addData("Y", pos.y);
                telemetry.update();
                // Stop once we've moved roughly -2.0 inches in X (backwards)
                if (pos.x <= -4.0) break;
                Backward();
                sleep(20);
            }
            stopMoving();

            // Hone to the positioning tag (ID 24) and then shoot
            telemetry.addLine("Honing to AprilTag ID " + TARGET_TAG_ID + "...");
            telemetry.update();
            final long HONE_TIMEOUT_MS = 8000;
            long honeStart = System.currentTimeMillis();
            try {
                aprilTagHone.reset();
                aprilTagHone.setRequiredFiducialId(TARGET_TAG_ID);
                aprilTagHone.setSearchTurnPower(0.12);
            } catch (Exception ignored) {}

            boolean aimed = false;
            while (opModeIsActive() && System.currentTimeMillis() - honeStart < HONE_TIMEOUT_MS) {
                AprilTagHone.HoneResult r = aprilTagHone.honeStored();
                telemetry.addData("Hone.valid", r.valid);
                telemetry.addData("turnPower", r.turnPower);
                telemetry.addData("forwardPower", r.forwardPower);
                telemetry.addData("Motif", aprilTagHone.getDetectedMotif().name());
                telemetry.update();
                if (r.valid && r.turnPower == 0.0 && r.forwardPower == 0.0) { aimed = true; break; }
                sleep(20);
            }
            stopMoving();

            // Determine shoot order from the motif detected during honing.
            // Green ball is always in slot 1; the motif encodes which basket each color goes to.
            //   GPP (tag 21): G→basket1, P→basket2, P→basket3  → shoot slots [1, 2, 3]
            //   PGP (tag 22): P→basket1, G→basket2, P→basket3  → shoot slots [2, 1, 3]
            //   PPG (tag 23): P→basket1, P→basket2, G→basket3  → shoot slots [2, 3, 1]
            //   NONE (no motif seen): fall back to default order [1, 2, 3]
            AprilTagHone.MotifPattern motif = aprilTagHone.getDetectedMotif();
            int[] shootOrder;
            if (motif == AprilTagHone.MotifPattern.PGP) {
                shootOrder = new int[]{2, 1, 3};
            } else if (motif == AprilTagHone.MotifPattern.PPG) {
                shootOrder = new int[]{2, 3, 1};
            } else {
                // GPP or NONE — both use [1, 2, 3]
                shootOrder = new int[]{1, 2, 3};
            }

            // Shoot three balls (blocking)
            if (shooterSequence != null) {
                telemetry.addData("Motif detected", motif.name());
                telemetry.addData("Shoot order", shootOrder[0] + ", " + shootOrder[1] + ", " + shootOrder[2]);
                telemetry.update();
                shooterSequence.shootInOrder(shootOrder);
                try { aprilTagHone.stop(); } catch (Exception ignored) {}
            } else {
                telemetry.addLine("Shooter components missing; cannot shoot");
                telemetry.update();
            }

            // Drive backward 9 inches after shooting
            telemetry.addLine("Driving backward 9 inches...");
            telemetry.update();
            myOtos.resetTracking();
            while (opModeIsActive()) {
                SparkFunOTOS.Pose2D pos = myOtos.getPosition();
                telemetry.addData("X", pos.x);
                telemetry.update();
                if (pos.x <= -14.0) break;
                Backward();
                sleep(20);
            }
            stopMoving();

            telemetry.addLine("Auto sequence complete");
            telemetry.update();
            // end opmode
            while (opModeIsActive()) { stopMoving(); sleep(50); }
        }
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

    private void StrafeRight(){
        frontLeftDrive.setPower(0.5);
        backLeftDrive.setPower(-0.5);
        frontRightDrive.setPower(-0.5);
        backRightDrive.setPower(0.5);
    }

    public void Forward(){
        frontLeftDrive.setPower(0.2);
        backLeftDrive.setPower(0.2);
        frontRightDrive.setPower(0.2);
        backRightDrive.setPower(0.2);
    }

    public void Backward(){
        frontLeftDrive.setPower(-0.2);
        backLeftDrive.setPower(-0.2);
        frontRightDrive.setPower(-0.2);
        backRightDrive.setPower(-0.2);
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
