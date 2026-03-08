package org.firstinspires.ftc.teamcode.season2025.Auto;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.season2025.Components.BallLauncher;
import org.firstinspires.ftc.teamcode.season2025.Components.BallSpooner;
import org.firstinspires.ftc.teamcode.season2025.Components.Turntable;
import org.firstinspires.ftc.teamcode.season2025.FeatureFlags;

@Autonomous(name = "V4AutoRedGoal", group = "Robot")
public class V4AutoRedGoal extends LinearOpMode {

    private SparkFunOTOS myOtos;

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    private BallLauncher ballLauncher;
    private Turntable turntable;
    private BallSpooner ballSpooner;

    private int currentStage = 0;
    private final double stageTime = 2.0;
    private final ElapsedTime runtime = new ElapsedTime();
    private int lastStage = -1;
    private boolean canShoot = false;
    private boolean strafeReady = false;
    private boolean pinPointReady = false;
    private Limelight3A limelight;



    @Override
    public void runOpMode() {

        // Drive motors (unused but initialized safely)
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "fl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "bl");
        backRightDrive  = hardwareMap.get(DcMotor.class, "br");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        // OTOS
        myOtos = hardwareMap.get(SparkFunOTOS.class, "otos");

        configureOtos();

        // Components
        if (FeatureFlags.launchEnabled()) {
            ballLauncher = new BallLauncher(hardwareMap, telemetry);
        }

        if (FeatureFlags.ballSpoonerEnabled()) {
            ballSpooner = new BallSpooner(hardwareMap, telemetry);
        }

        if (FeatureFlags.turnTableEnabled()) {
            turntable = new Turntable(hardwareMap, telemetry);
            turntable._currentPosition = turntable.MIN_POSITION;

        }

        telemetry.addLine("Auto ready");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            SparkFunOTOS.Pose2D pos = myOtos.getPosition();
            if (ballSpooner != null) {
                ballSpooner.updateSpoonerState();
            }

            // Only run stage logic ONCE per stage
            if (currentStage != lastStage && canShoot) {

                switch (currentStage) {

                    case 0:
                        ballLauncher.setLaunchPresetAuto();
                        break;

                    case 1:
                        turntable.moveToIndex(1);
                        break;

                    case 2:
                        ballSpooner.fire();
                        break;

                    case 3:
                        turntable.moveToIndex(2);
                        break;

                    case 4:
                        ballSpooner.fire();
                        break;

                    case 5:
                        turntable.increaseIndex();
                        break;

                    case 6:
                        ballSpooner.fire();
                        break;

                    default:
                        ballLauncher.turnOffLauncher();
                        strafeReady = true;
                        break;
                }

                lastStage = currentStage;

            }

            // Advance stage after time
            if (runtime.seconds() >= stageTime && canShoot) {
                currentStage++;
                runtime.reset();
            }

            if (!canShoot) {
                escapeGoal();

            } else if (strafeReady){
                escapeZone();
            }




            telemetry.addData("Stage", currentStage);
            telemetry.addData("Turntable Slot", turntable.currentSlot());
            telemetry.addData("Spooner Busy", ballSpooner.isBusy());
            telemetry.addData("Turntable Position", turntable._currentPosition);

            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("Botpose", botpose.toString());
                    telemetry.update();
                }
            }


            telemetry.update();
        }

    }

    private void configureOtos() {

        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        SparkFunOTOS.Pose2D offset =
                new SparkFunOTOS.Pose2D(0.5, -1.25, 90);
        myOtos.setOffset(offset);

        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);

        myOtos.calibrateImu();
        myOtos.resetTracking();
        myOtos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
    }

    private void escapeGoal(){

        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        if (pos.x <= 30){
            Forward();
        }
        else {
            stopMoving();
            canShoot = true;
        }
    }

    private void escapeZone(){
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        if (pos.x >= 19.5 && strafeReady){
            if (pos.y <= 15.5){
                StrafeLeft();
            }
            else {
                stopMoving();

            }

        }
    }

    private void pinPointTag(){
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();

                if (result.getTx() <= 1.5 && result.getTx() >= -1.5){
                    stopMoving();
                    pinPointReady = false;
                    canShoot = true;
                } else if(result.getTx() >= 1.5){
                    tweakRight();
                } else if (result.getTx() <= -1.5){
                    tweakLeft();
                }

                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Botpose", botpose.toString());
                telemetry.update();
            }
        }
    }
    public void stopMoving(){
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
        backLeftDrive.setPower(0);
    }
    public void Forward(){
        frontLeftDrive.setPower(0.5);
        backLeftDrive.setPower(0.5);
        frontRightDrive.setPower(0.5);
        backRightDrive.setPower(0.5);
    }

    private void StrafeLeft(){
        frontLeftDrive.setPower(-0.5);
        backLeftDrive.setPower(0.5);
        frontRightDrive.setPower(0.5);
        backRightDrive.setPower(-0.5);
    }

    private void tweakRight(){
        frontLeftDrive.setPower(0.1);
        backLeftDrive.setPower(0.1);
        frontRightDrive.setPower(-0.1);
        backRightDrive.setPower(-0.1);
    }
    private void tweakLeft(){
        frontLeftDrive.setPower(-0.1);
        backLeftDrive.setPower(-0.1);
        frontRightDrive.setPower(0.1);
        backRightDrive.setPower(0.1);
    }
}
