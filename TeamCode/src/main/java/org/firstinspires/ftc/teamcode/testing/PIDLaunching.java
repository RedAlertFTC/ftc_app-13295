package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.corelib.control.DebouncedButton;
import org.firstinspires.ftc.teamcode.corelib.control.DisasterGamePad;

@TeleOp(name="PIDLaunching", group="Linear OpMode")
public class PIDLaunching extends LinearOpMode {

    /* =========================
       DRIVE MOTORS
       ========================= */
    private DcMotorEx frontLeftDrive;
    private DcMotorEx backLeftDrive;
    private DcMotorEx frontRightDrive;
    private DcMotorEx backRightDrive;

    /* =========================
       LAUNCHER
       ========================= */
    private DcMotorEx leftLaunchMotor;
    private DcMotorEx rightLaunchMotor;

    private double targetTPS = 0;

    private static final double MAX_TPS = 2800;
    private static final double MIN_TPS = 670;
    private static final double TPS_INCREMENT = 50;

    /* =========================
       PIDF COEFFICIENTS
       ========================= */
    private static final double kP = 0.0012;
    private static final double kI = 0.0;
    private static final double kD = 0.0001;
    private static final double kF = 0.0002;

    /* =========================
       SERVOS
       ========================= */
    private Servo spoonServo;
    private Servo linearServo;

    private static final double SPOON_START = 1.0;
    private static final double SPOON_FIRE = 0.5;

    private static final double LINEAR_MIN = 0.25;
    private static final double LINEAR_MAX = 0.75;
    private static final double LINEAR_INCREMENT = 0.05;

    private double currentLinearPos = LINEAR_MIN;

    /* =========================
       GAMEPAD
       ========================= */
    private DisasterGamePad driver;
    private DebouncedButton increaseTPS;
    private DebouncedButton decreaseTPS;
    private DebouncedButton fireButton;

    /* =========================
       FIRING STATE MACHINE
       ========================= */
    private enum FiringState {
        REST,
        FIRE,
        FIRING,
        FIRED,
        RESETTING
    }

    private FiringState currentState = FiringState.REST;
    private long stateStartMs = 0;
    private static final long FIRE_TIME_MS = 500;

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        /* =========================
           HARDWARE INIT
           ========================= */
        frontLeftDrive  = hardwareMap.get(DcMotorEx.class, "fl");
        backLeftDrive   = hardwareMap.get(DcMotorEx.class, "bl");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "fr");
        backRightDrive  = hardwareMap.get(DcMotorEx.class, "br");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftLaunchMotor  = hardwareMap.get(DcMotorEx.class, "launchLeft");
        rightLaunchMotor = hardwareMap.get(DcMotorEx.class, "launchRight");

        leftLaunchMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightLaunchMotor.setDirection(DcMotorEx.Direction.REVERSE);

        leftLaunchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLaunchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLaunchMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        rightLaunchMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        spoonServo = hardwareMap.get(Servo.class, "spooningServo");
        linearServo = hardwareMap.get(Servo.class, "linearServo");

        spoonServo.setPosition(SPOON_START);
        linearServo.setPosition(currentLinearPos);

        driver = new DisasterGamePad(gamepad1);
        increaseTPS = new DebouncedButton(driver.getBButton());
        decreaseTPS = new DebouncedButton(driver.getXButton());
        fireButton   = new DebouncedButton(driver.getRightBumper());

        telemetry.addLine("PID Launcher Ready");
        telemetry.update();

        waitForStart();
        runtime.reset();

        /* =========================
           MAIN LOOP
           ========================= */
        while (opModeIsActive()) {

            /* ===== MECANUM DRIVE ===== */
            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            double fl = axial + lateral + yaw;
            double fr = axial - lateral - yaw;
            double bl = axial - lateral + yaw;
            double br = axial + lateral - yaw;

            double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                    Math.max(Math.abs(bl), Math.abs(br)));

            if (max > 1.0) {
                fl /= max; fr /= max; bl /= max; br /= max;
            }

            frontLeftDrive.setPower(fl);
            frontRightDrive.setPower(fr);
            backLeftDrive.setPower(bl);
            backRightDrive.setPower(br);

            /* ===== LAUNCHER CONTROL ===== */
            if (increaseTPS.getRise()) increaseLauncherSpeed();
            if (decreaseTPS.getRise()) decreaseLauncherSpeed();

            if (fireButton.getRise()) fire();

            updateFiringState();

            /* ===== TELEMETRY ===== */
            telemetry.addData("Target TPS", targetTPS);
            telemetry.addData("Left TPS", leftLaunchMotor.getVelocity());
            telemetry.addData("Right TPS", rightLaunchMotor.getVelocity());
            telemetry.addData("Launcher Ready", launcherAtSpeed());
            telemetry.addData("Firing State", currentState);
            telemetry.update();
        }
    }

    /* =========================
       LAUNCHER METHODS
       ========================= */
    private void setLauncherTPS(double tps) {
        targetTPS = Math.max(MIN_TPS, Math.min(MAX_TPS, tps));
        leftLaunchMotor.setVelocity(targetTPS);
        rightLaunchMotor.setVelocity(targetTPS);
    }

    private void increaseLauncherSpeed() {
        setLauncherTPS(targetTPS + TPS_INCREMENT);
    }

    private void decreaseLauncherSpeed() {
        setLauncherTPS(targetTPS - TPS_INCREMENT);
    }

    private boolean launcherAtSpeed() {
        double tolerance = 50;
        return Math.abs(leftLaunchMotor.getVelocity() - targetTPS) < tolerance
                && Math.abs(rightLaunchMotor.getVelocity() - targetTPS) < tolerance;
    }

    /* =========================
       FIRING STATE MACHINE
       ========================= */
    private void fire() {
        if (currentState == FiringState.REST && launcherAtSpeed()) {
            currentState = FiringState.FIRE;
        }
    }

    private void updateFiringState() {
        switch (currentState) {

            case REST:
                break;

            case FIRE:
                spoonServo.setPosition(SPOON_FIRE);
                stateStartMs = System.currentTimeMillis();
                currentState = FiringState.FIRING;
                break;

            case FIRING:
                if (System.currentTimeMillis() - stateStartMs > FIRE_TIME_MS) {
                    spoonServo.setPosition(SPOON_START);
                    stateStartMs = System.currentTimeMillis();
                    currentState = FiringState.RESETTING;
                }
                break;

            case RESETTING:
                if (System.currentTimeMillis() - stateStartMs > FIRE_TIME_MS) {
                    currentState = FiringState.REST;
                }
                break;
        }
    }
}
