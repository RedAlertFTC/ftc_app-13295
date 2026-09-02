package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.corelib.control.DebouncedButton;
import org.firstinspires.ftc.teamcode.corelib.control.DisasterGamePad;

@TeleOp(name="PIDLaunching_AutoTagDrive_Launcher", group="Linear OpMode")
@Disabled
public class PIDLaunching extends LinearOpMode {

    /* ========================= DRIVE MOTORS ========================= */
    private DcMotorEx frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;

    /* ========================= LAUNCHER ========================= */
    private DcMotorEx leftLaunchMotor, rightLaunchMotor;
    private double targetTPS = 0;
    private static final double MAX_TPS = 2800;
    private static final double MIN_TPS = 670;
    private static final double TPS_INCREMENT = 50;

    // Auto-launch speed while LB held
    private static final double AUTO_LAUNCH_TPS = 2200;

    /* ========================= PIDF COEFFICIENTS ========================= */
    private static final double kP = 0.0012;
    private static final double kI = 0.0;
    private static final double kD = 0.0001;
    private static final double kF = 0.0002;

    /* ========================= SERVOS ========================= */
    private Servo spoonServo, linearServo;
    private static final double SPOON_START = 1.0;
    private static final double SPOON_FIRE = 0.5;
    private static final double LINEAR_MIN = 0.25;

    /* ========================= GAMEPAD ========================= */
    private DisasterGamePad driver;
    private DebouncedButton increaseTPS, decreaseTPS, fireButton;

    /* ========================= FIRING STATE MACHINE ========================= */
    private enum FiringState { REST, FIRE, FIRING, RESETTING }
    private FiringState currentState = FiringState.REST;
    private long stateStartMs = 0;
    private static final long FIRE_TIME_MS = 500;

    /* ========================= LIMELIGHT ========================= */
    private Limelight3A limelight;
    private static final double SEARCH_SPIN_POWER = 0.25;

    // ===== AUTO-TAG STATES =====
    private enum AutoTagState { SEARCH, DRIVE, DONE }
    private AutoTagState autoTagState = AutoTagState.SEARCH;

    // Calibrate this with your camera at 50 inches from tag
    private static final double TARGET_TY_50_IN = -6.5;
    private static final double TY_TOLERANCE = 0.5;
    private static final double DRIVE_POWER = 0.25;

    @Override
    public void runOpMode() {

        /* ===== HARDWARE INIT ===== */
        frontLeftDrive  = hardwareMap.get(DcMotorEx.class, "fl");
        backLeftDrive   = hardwareMap.get(DcMotorEx.class, "bl");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "fr");
        backRightDrive  = hardwareMap.get(DcMotorEx.class, "br");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        leftLaunchMotor  = hardwareMap.get(DcMotorEx.class, "launchLeft");
        rightLaunchMotor = hardwareMap.get(DcMotorEx.class, "launchRight");

        leftLaunchMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightLaunchMotor.setDirection(DcMotorEx.Direction.REVERSE);

        leftLaunchMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        rightLaunchMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        spoonServo  = hardwareMap.get(Servo.class, "spooningServo");
        linearServo = hardwareMap.get(Servo.class, "linearServo");
        spoonServo.setPosition(SPOON_START);
        linearServo.setPosition(LINEAR_MIN);

        driver = new DisasterGamePad(gamepad1);
        increaseTPS = new DebouncedButton(driver.getDpadRight());
        decreaseTPS = new DebouncedButton(driver.getDpadLeft());
        fireButton  = new DebouncedButton(driver.getAButton());

        /* ===== LIMELIGHT INIT ===== */
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // choose pipeline 0
        limelight.start();

        telemetry.addLine("Ready: Hold LEFT BUMPER to search & approach AprilTag, auto-launch enabled");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            /* ===== AUTO-TAG + AUTO-LAUNCH ===== */
            if (gamepad1.left_bumper) {

                axial = 0;
                lateral = 0;

                LLResult result = limelight.getLatestResult();

                // override launcher speed automatically while LB held
                targetTPS = AUTO_LAUNCH_TPS;

                switch (autoTagState) {

                    case SEARCH:
                        boolean hasTag = (result != null && result.isValid() && !result.getFiducialResults().isEmpty());
                        if (hasTag) {
                            yaw = 0;
                            autoTagState = AutoTagState.DRIVE;
                            telemetry.addLine("AprilTag detected!");
                        } else {
                            yaw = SEARCH_SPIN_POWER;
                            telemetry.addLine("Spinning — searching for AprilTag...");
                        }
                        break;



                    case DRIVE:
                        if (result != null && result.isValid()) {
                            double ty = result.getTy();
                            double error = ty - TARGET_TY_50_IN;

                            if (Math.abs(error) > TY_TOLERANCE) {
                                axial = DRIVE_POWER;
                                yaw = 0;
                            } else {
                                axial = 0;
                                yaw = 0;
                                autoTagState = AutoTagState.DONE;
                            }

                            telemetry.addData("ty", ty);
                            telemetry.addData("Error", error);

                        } else {
                            autoTagState = AutoTagState.SEARCH;
                        }
                        break;

                    case DONE:
                        axial = 0;
                        yaw = 0;
                        telemetry.addLine("At 50 inches from AprilTag");
                        break;
                }

            } else {
                // Bumper released → reset state
                autoTagState = AutoTagState.SEARCH;
            }

            /* ===== MECANUM DRIVE CALC ===== */
            double flP = axial + lateral + yaw;
            double frP = axial - lateral - yaw;
            double blP = axial - lateral + yaw;
            double brP = axial + lateral - yaw;

            double max = Math.max(
                    Math.max(Math.abs(flP), Math.abs(frP)),
                    Math.max(Math.abs(blP), Math.abs(brP))
            );
            if (max > 1.0) {
                flP /= max;
                frP /= max;
                blP /= max;
                brP /= max;
            }

            frontLeftDrive.setPower(flP);
            frontRightDrive.setPower(frP);
            backLeftDrive.setPower(blP);
            backRightDrive.setPower(brP);

            /* ===== LAUNCHER CONTROL ===== */
            if (!gamepad1.left_bumper) {
                // only allow manual control if LB not held
                if (increaseTPS.getRise()) targetTPS = Math.min(targetTPS + TPS_INCREMENT, MAX_TPS);
                if (decreaseTPS.getRise()) targetTPS = Math.max(targetTPS - TPS_INCREMENT, MIN_TPS);
            }

            leftLaunchMotor.setVelocity(targetTPS);
            rightLaunchMotor.setVelocity(targetTPS);

            if (fireButton.getRise() && launcherAtSpeed()) fire();
            updateFiringState();

            telemetry.addData("Target TPS", targetTPS);
            telemetry.addData("Launcher Ready", launcherAtSpeed());
            telemetry.addData("Firing State", currentState);
            telemetry.update();
        }
    }

    private boolean launcherAtSpeed() {
        return Math.abs(leftLaunchMotor.getVelocity() - targetTPS) < 50 &&
                Math.abs(rightLaunchMotor.getVelocity() - targetTPS) < 50;
    }

    private void fire() {
        if (currentState == FiringState.REST) currentState = FiringState.FIRE;
    }

    private void updateFiringState() {
        switch (currentState) {
            case FIRE:
                spoonServo.setPosition(SPOON_FIRE);
                stateStartMs = System.currentTimeMillis();
                currentState = FiringState.FIRING;
                break;
            case FIRING:
                if (System.currentTimeMillis() - stateStartMs > FIRE_TIME_MS) {
                    spoonServo.setPosition(SPOON_START);
                    currentState = FiringState.REST;
                }
                break;
        }
    }
}
