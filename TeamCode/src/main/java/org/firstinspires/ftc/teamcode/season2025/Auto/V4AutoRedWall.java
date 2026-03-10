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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

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
            aprilTagHone.setRequiredFiducialId(24);
            // Use a gentle search rotation so the robot slowly scans for ID24 instead of aggressively spinning
            aprilTagHone.setSearchTurnPower(0.12);
            aprilTagHone.start();

            if (_ballLauncher != null && _turntable != null && _ballSpooner != null) {
                shooterSequence = new ShooterSequence(_ballLauncher, _turntable, _ballSpooner, telemetry);
            }
            if (_ballLauncher != null) {
                _ballLauncher.setLaunchPresetAuto();
            }

            // STATIONARY pre-scan: stay in place and look for motif tags (21/22/23) before moving.
            int motifIdFound = -1; // 21/22/23 -> determines order; -1 = none
            try {
                long scanStart = System.currentTimeMillis();
                final long PRE_SCAN_MS = 1500; // how long we'll try to detect the motif while stationary
                telemetry.addLine("Scanning for motif (stationary)...");
                telemetry.update();
                while (opModeIsActive() && System.currentTimeMillis() - scanStart < PRE_SCAN_MS) {
                    LLResult res = null;
                    try { res = aprilTagHone.getLimelight().getLatestResult(); } catch (Exception ignored) { res = null; }
                    if (res != null && res.isValid()) {
                        try {
                            java.util.List<com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult> frs = res.getFiducialResults();
                            if (frs != null) {
                                for (com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fr : frs) {
                                    int id = fr.getFiducialId();
                                    if (id == 21 || id == 22 || id == 23) {
                                        motifIdFound = id;
                                        telemetry.addData("MotifID", motifIdFound);
                                        telemetry.addLine("Motif recorded — will NOT be used for aiming; only for shot order");
                                        telemetry.update();
                                        // Restart the hone so it focuses on the real positioning target (ID 24)
                                        try {
                                            aprilTagHone.reset();
                                            aprilTagHone.setRequiredFiducialId(24);
                                            aprilTagHone.setSearchTurnPower(0.12);
                                            // One immediate apply to ensure hone resumes search behavior promptly
                                            try { aprilTagHone.honeStored(); } catch (Exception ignored) {}
                                            telemetry.addLine("Hone restarted to search for ID24");
                                            telemetry.update();
                                        } catch (Exception ignored) {}
                                        break;
                                    }
                                }
                            }
                        } catch (Exception ignored) {}
                    }

                    if (motifIdFound != -1) break; // we found a motif while stationary
                    sleep(100);
                }
            } catch (Exception ignored) {}

            // 1) Drive forward until X >= 2 inches
            // Robot is deployed backwards on the field — drive 2 inches backward instead
            telemetry.addLine("Driving backward 2 inches...");
            telemetry.update();
            runtime.reset();
            while (opModeIsActive()) {
                SparkFunOTOS.Pose2D pos = myOtos.getPosition();
                telemetry.addData("X", pos.x);
                telemetry.addData("Y", pos.y);
                telemetry.update();
                // Stop once we've moved roughly -2.0 inches in X (backwards)
                if (pos.x <= -2.0) break;
                Backward();
                sleep(20);
            }
            stopMoving();

            // 2) Run AprilTag honing until on target (use default deadband); include a timeout
            telemetry.addLine("Honing to AprilTag...");
            telemetry.update();
            final long HONE_TIMEOUT_MS = 8000;
            long honeStart = System.currentTimeMillis();
            // Ensure hone still ignores motif tags and only targets ID24
            try { aprilTagHone.setRequiredFiducialId(24); } catch (Exception ignored) {}
            while (opModeIsActive() && System.currentTimeMillis() - honeStart < HONE_TIMEOUT_MS) {
                if (motifIdFound != -1) {
                    telemetry.addData("MotifID", motifIdFound);
                }
                 // Use the hone's update which now respects requiredFiducialId==24: when ID24 is absent
                 // the hone will report r.valid=false and will return turnPower=searchTurnPower, causing
                 // hone.honeStored() to command the stored motors to spin/search. This centralizes search
                 // behavior in AprilTagHone and avoids the opmode manually calling spin().
                 AprilTagHone.HoneResult r = aprilTagHone.honeStored();
                 telemetry.addData("Hone valid (24)", r.valid);
                 telemetry.addData("turnPower", r.turnPower);
                 telemetry.addData("forwardPower", r.forwardPower);
                 telemetry.update();
                 if (r.valid && r.turnPower == 0.0 && r.forwardPower == 0.0) break;
                 sleep(20);
             }
            // Keep limelight running so the subsequent wait for ID24 can see targets;
            // do not stop aprilTagHone() here.
            stopMoving();

            // 3) Shoot three balls (blocking)
            if (shooterSequence != null) {
                telemetry.addLine("Deciding shoot order based on motif (21/22/23), waiting to be aimed on ID 24...");
                telemetry.update();

                // Map motif id to slot order:
                // 21 -> 1,2,3
                // 22 -> 2,1,3
                // 23 -> 2,3,1
                int[] order = new int[] {1,2,3};
                if (motifIdFound == 21) order = new int[] {1,2,3};
                else if (motifIdFound == 22) order = new int[] {2,1,3};
                else if (motifIdFound == 23) order = new int[] {2,3,1};

                // Wait until aprilTagHone reports we are aimed at ID 24 specifically. We'll poll limelight
                // and check fiducials; only accept id==24 as the positioning target.
                final long WAIT_24_TIMEOUT_MS = 8000;
                long waitStart = System.currentTimeMillis();
                boolean aimedOn24 = false;
                try {
                    while (opModeIsActive() && System.currentTimeMillis() - waitStart < WAIT_24_TIMEOUT_MS) {
                        LLResult res = aprilTagHone.getLimelight().getLatestResult();
                        if (res != null && res.isValid()) {
                            java.util.List<com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult> frs = res.getFiducialResults();
                            boolean has24 = false;
                            if (frs != null) {
                                for (com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fr : frs) {
                                    if (fr.getFiducialId() == 24) { has24 = true; break; }
                                }
                            }
                            if (has24) {
                                // If limelight sees ID24 and the hone controller reports aimed, proceed
                                if (aprilTagHone.isAimed()) { aimedOn24 = true; break; }
                            }
                        }
                        sleep(50);
                    }
                } catch (Exception ignored) {}

                if (aimedOn24) {
                    telemetry.addData("Shooter", "Aimed on ID 24; shooting in order for motif %d", motifIdFound);
                    telemetry.update();
                    shooterSequence.shootInOrder(order);
                } else {
                    telemetry.addData("Shooter", "Timed out waiting to aim on ID24; aborting shoot order");
                    telemetry.update();
                }

                // Now that shooting (or abort) is complete, stop limelight processing
                try { aprilTagHone.stop(); } catch (Exception ignored) {}
            } else {
                telemetry.addLine("Shooter components missing; cannot shoot");
                telemetry.update();
            }

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
