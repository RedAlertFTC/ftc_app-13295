package org.firstinspires.ftc.teamcode.season2025.Auto;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.season2025.Components.BallLauncher;
import org.firstinspires.ftc.teamcode.season2025.Components.BallSpooner;
import org.firstinspires.ftc.teamcode.season2025.Components.Turntable;
import org.firstinspires.ftc.teamcode.season2025.FeatureFlags;

@Autonomous(name = "OtosAutoBlueGoal", group = "Robot")
public class OtosAutoBlueGoal extends LinearOpMode {

    private SparkFunOTOS myOtos;

    private DcMotor fl, fr, bl, br;

    private BallLauncher ballLauncher;
    private Turntable turntable;
    private BallSpooner spooner;

    private enum AutoState {
        DRIVE_FORWARD,
        SHOOT,
        STRAFE,
        DONE
    }

    private AutoState state = AutoState.DRIVE_FORWARD;

    @Override
    public void runOpMode() {

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        myOtos = hardwareMap.get(SparkFunOTOS.class, "otos");

        if (FeatureFlags.launchEnabled())
            ballLauncher = new BallLauncher(hardwareMap, telemetry);

        if (FeatureFlags.turnTableEnabled())
            turntable = new Turntable(hardwareMap, telemetry);

        if (FeatureFlags.ballSpoonerEnabled())
            spooner = new BallSpooner(hardwareMap, telemetry);

        configureOtos();

        waitForStart();

        while (opModeIsActive()) {

            SparkFunOTOS.Pose2D pos = myOtos.getPosition();

            switch (state) {

                case DRIVE_FORWARD:
                    if (pos.x < 19.5) {
                        driveForward();
                    } else {
                        stopDrive();
                        state = AutoState.SHOOT;
                    }
                    break;

                case SHOOT:
                    shootAllBalls();
                    state = AutoState.STRAFE;
                    break;

                case STRAFE:
                    if (pos.y > -15.5) {
                        strafeRight();
                    } else {
                        stopDrive();
                        state = AutoState.DONE;
                    }
                    break;

                case DONE:
                    stopDrive();
                    break;
            }

            telemetry.addData("State", state);
            telemetry.addData("X", pos.x);
            telemetry.addData("Y", pos.y);
            telemetry.update();
        }
    }

    /* ---------------- DRIVE METHODS ---------------- */

    private void driveForward() {
        setPower(0.5, 0.5, 0.5, 0.5);
    }

    private void strafeRight() {
        setPower(0.5, -0.5, -0.5, 0.5);
    }

    private void stopDrive() {
        setPower(0, 0, 0, 0);
    }

    private void setPower(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        fl.setPower(frontLeftPower);
        fr.setPower(frontRightPower);
        bl.setPower(backLeftPower);
        br.setPower(backRightPower);
    }

    /* ---------------- SHOOTING ---------------- */

    private void shootAllBalls() {
        ballLauncher.setLaunchPresetOne();
        sleep(2000);

        for (int i = 1; i <= 3; i++) {
            turntable.moveToIndex(i);
            sleep(700);
            spooner.fire();
            sleep(700);
        }
    }

    /* ---------------- OTOS CONFIG ---------------- */

    private void configureOtos() {

        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0.5, -1.25, 90);
        myOtos.setOffset(offset);

        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);

        myOtos.calibrateImu();
        myOtos.resetTracking();
        myOtos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
    }
}
