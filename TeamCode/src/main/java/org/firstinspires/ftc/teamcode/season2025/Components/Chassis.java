package org.firstinspires.ftc.teamcode.season2025.Components;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.testing.SnapToGoal;

public class Chassis {
    private double maxPower = 1.0;
    private double maxSpeed = 1.0;  // make this slower for outreaches

    public DcMotorEx frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    HardwareMap hardwareMap;
    private Limelight3A limelight;

    private ChassisState robotStatus = ChassisState.STOPPED;

    public Chassis(HardwareMap hardwareMap) {

        this.hardwareMap = hardwareMap;
        init();
    }
    public Chassis(HardwareMap hardwareMap, Limelight3A limelight) {
        this.hardwareMap = hardwareMap;
        this.limelight = limelight;
        init();
    }

    private void init() {

        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "fl");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "bl");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "fr");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "br");


        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower  = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower  = forward + right - rotate;
        double backLeftPower   = forward - right + rotate;

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = maxSpeed;
    }

    public void stopMotors() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
    public void moveAllMotors(double frontleftpower, double frontrightpower, double backleftpower, double backrightpower) {
        frontLeftDrive.setPower(frontleftpower);
        frontRightDrive.setPower(frontrightpower);
        backLeftDrive.setPower(backleftpower);
        backRightDrive.setPower(backrightpower);
    }
    public void turn(double power, double mseconds){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (timer.milliseconds() < mseconds) {
            frontLeftDrive.setPower(power);
            backLeftDrive.setPower(power);
            frontRightDrive.setPower(-power);
            backRightDrive.setPower(-power);
        }

        stopMotors();
    }
    public void moveForward(double power, double mseconds){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (timer.milliseconds() < mseconds) {
            frontLeftDrive.setPower(power);
            backLeftDrive.setPower(power);
            frontRightDrive.setPower(power);
            backRightDrive.setPower(power);
        }

        stopMotors();
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

    private void setState(ChassisState newState) {
        robotStatus = newState;
    }

    private void update() {

    }
    private void SnapAprilTag() {

        //StopBot();

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
                robotStatus = ChassisState.POSITIONING;
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
