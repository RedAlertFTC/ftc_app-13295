/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.season2025.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.corelib.control.DebouncedButton;
import org.firstinspires.ftc.teamcode.corelib.control.DisasterGamePad;
import org.firstinspires.ftc.teamcode.season2025.Components.AprilTagHone;
import org.firstinspires.ftc.teamcode.season2025.Components.BallAimer;
import org.firstinspires.ftc.teamcode.season2025.Components.BallIntake;
import org.firstinspires.ftc.teamcode.season2025.Components.BallLauncher;
import org.firstinspires.ftc.teamcode.season2025.Components.BallSpooner;
import org.firstinspires.ftc.teamcode.season2025.Components.GoalPositioning;
import org.firstinspires.ftc.teamcode.season2025.Components.LightColor;
import org.firstinspires.ftc.teamcode.season2025.Components.LightController;
import org.firstinspires.ftc.teamcode.season2025.Components.ShooterSequence;
import org.firstinspires.ftc.teamcode.season2025.FeatureFlags;
import org.firstinspires.ftc.teamcode.season2025.Components.Turntable;
import org.firstinspires.ftc.teamcode.testing.PedroTesting;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="DecodeV1Teleop", group="Linear OpMode")
public class DecodeV1Teleop extends LinearOpMode
{

    public teleOpAlliance _teleOpAlliance;
    public enum teleOpAlliance{
        RED,
        BLUE
    }


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backRightDrive = null;
    private Limelight3A limelight;

    private BallLauncher _ballLauncher;
    private Turntable _turntable;
    private PedroTesting _pedroTesting;
    private BallIntake _ballIntake;
    private BallSpooner _ballSpooner;
    private GoalPositioning _goalPositioning;
    private LightController _lightController;
    private ShooterSequence _shooterSequence;

    private DisasterGamePad _driverOneGamepad;
    private DisasterGamePad _driverTwoGamepad;

    private DebouncedButton _increaseLaunchPower;
    private DebouncedButton _decreaseLaunchPower;
    private DebouncedButton _stopLauncher;
    private DebouncedButton _increaseIndex;
    private DebouncedButton _decreaseIndex;
    private DebouncedButton _pedroStart;
    private DebouncedButton _popBall;
    private DebouncedButton _toggleRobotDirection;
    private DebouncedButton _aimLauncherUp;
    private DebouncedButton _aimLauncherDown;
    private DebouncedButton _launcherPresetOne;
    private DebouncedButton _launcherPresetTwo;
    private DebouncedButton _launcherPresetThree;
    private DebouncedButton _turnOffLauncher;
    private DebouncedButton _shootAllBalls;
    private double launcherIncrement = 0.05F;
    private int goalAprilTag;
    private boolean forwardDriving = true;
    private boolean greenBallDetected = false;
    private boolean purpleBallDetected = false;

    private AprilTagHone hone;

    // New fields for smoothing and hone state
    private boolean honingActive = false;
    private double prevHoneLeft = 0.0;
    private double prevHoneRight = 0.0;
    // smoothing factor: closer to 1.0 = smoother/slower, 0.0 = no smoothing
    private final double HONE_SMOOTH_ALPHA = 0.85;
    // small deadband to avoid constant tiny corrections (tweak as needed)
    private final double HONE_DEADBAND = 0.03;
    // maximum allowed change per loop (slew rate limit)
    private final double HONE_MAX_DELTA = 0.2;

    private NormalizedColorSensor colorSensor;
    // remember which hardware name we successfully used (helps debugging)
    private String colorSensorName = null;

    // Default gain — increase if readings are very small in your environment. Values up to 8 are common.
    float gain = 8;


    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);


        // Initialize color sensor: try common names used in examples/configs.
        try {
            colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");
            colorSensorName = "color_sensor";
        } catch (Exception e1) {
            try {
                colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
                colorSensorName = "colorSensor";
            } catch (Exception e2) {
                colorSensor = null;
                colorSensorName = null;
            }
        }

        /*
         * Starts polling for data.
         */
        limelight.start();

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "fl");
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeftDrive = hardwareMap.get(DcMotorEx.class, "bl");
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRightDrive = hardwareMap.get(DcMotorEx.class, "fr");
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRightDrive = hardwareMap.get(DcMotorEx.class, "br");
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, stard observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of at out with the reversals here, BUT
        //        // when you first test your robot, push the left joystick forward anny wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.

        hone = new AprilTagHone();
        // Use initLimelight so hone can read the limelight; motors are passed directly below when homing
        hone.initLimelight(hardwareMap, "limelight");
        // Default aim offset: aim to the left of the AprilTag by 5 degrees
        hone.setAimOffsetDegrees(5.0);
        // We won't call initMotors because this test already initializes motors and sets directions.
        hone.start();
        hone.reset();

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);


        hardwareInit();

        waitForStart();
        runtime.reset();

        // Wait for the game to start (driver presses START)
       // telemetry.addData("Status", "Initialized");

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Live gain adjustment: press gamepad1 A to increase, B to decrease (clamped 1..8)
            if (gamepad1.a) {
                gain = Math.min(8.0f, gain + 0.5f);
            } else if (gamepad1.x) {
                gain = Math.max(1.0f, gain - 0.5f);
            }

            // Read color sensor each loop after ensuring it's available and applying gain.
            NormalizedRGBA colors = null;
            if (colorSensor != null) {
                try {
                    // set desired gain before reading
                    colorSensor.setGain(gain);
                    colors = colorSensor.getNormalizedColors();
                } catch (Exception e) {
                    // If reading fails, null out colors so we don't dereference
                    colors = null;
                    telemetry.addData("ColorSensorError", e.getMessage());
                }
            }

            double max;

            double axial   = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  -gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            if(forwardDriving) {
                 axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
                 lateral =  gamepad1.left_stick_x;
                 yaw     =  gamepad1.right_stick_x;

            } else {
                // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
                 axial   = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
                 lateral =  -gamepad1.left_stick_x;
                 yaw     =  gamepad1.right_stick_x;
            }




            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backRightPower  = axial + lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            frontLeftPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            backLeftPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            frontRightPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            backRightPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */


            // Send calculated power to wheels (default manual values will be applied here, but may be overridden by honing)
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);


            LLResult result = limelight.getLatestResult();
            if (result != null) {


                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();


                    // Position is in meters
                    double x = botpose.getPosition().x;
                    double y = botpose.getPosition().y;
                    double z = botpose.getPosition().z;

                    // Distances
                    double forwardDistanceMeters = z;
                    double totalDistanceMeters = Math.sqrt(x*x + y*y + z*z);

                    // Convert to inches
                    double forwardDistanceInches = forwardDistanceMeters * 39.37;
                    double totalDistanceInches = totalDistanceMeters * 39.37;

                    double distance = result.getBotpose().getPosition().z;


                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("Distance cm", getDistanceIn(result.getTy()));

                  //  telemetry.addData("DesiredPower", calculateCurrentPower(result.getTy()));

                    telemetry.addData("Botpose", botpose.toString());
                    telemetry.addData("Distance", distance);
                    telemetry.addData("Forward Distance (in)", "%.1f", forwardDistanceInches);
                    telemetry.addData("Total Distance (in)", "%.1f", totalDistanceInches);



                }
                else {
                    telemetry.addLine("Not valid tag");
                }
            }
            else {
                telemetry.addLine("No tag detected");
            }

            // HOMING: engage when B pressed
            if (gamepad1.b) {
                // If we just started honing, prepare motors and reset hone state
                if (!honingActive) {
                    honingActive = true;
                    hone.reset(); // clear PID state to avoid integrator windup
                    // Switch motors to RUN_WITHOUT_ENCODER for power-controlled behavior and FLOAT for smoother small corrections
                    try {
                        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    } catch (Exception e) {
                        // ignore hardware mode exceptions
                    }
                }

                AprilTagHone.HoneResult r = hone.update();

                // Target outputs from hone
                double targetLeft = r.leftPower;
                double targetRight = r.rightPower;

                // Exponential smoothing
                double smoothedLeft = (HONE_SMOOTH_ALPHA * prevHoneLeft) + ((1.0 - HONE_SMOOTH_ALPHA) * targetLeft);
                double smoothedRight = (HONE_SMOOTH_ALPHA * prevHoneRight) + ((1.0 - HONE_SMOOTH_ALPHA) * targetRight);

                // Slew-rate limit (cap delta from previous output)
                double deltaLeft = smoothedLeft - prevHoneLeft;
                if (deltaLeft > HONE_MAX_DELTA) deltaLeft = HONE_MAX_DELTA;
                if (deltaLeft < -HONE_MAX_DELTA) deltaLeft = -HONE_MAX_DELTA;
                double leftOut = prevHoneLeft + deltaLeft;

                double deltaRight = smoothedRight - prevHoneRight;
                if (deltaRight > HONE_MAX_DELTA) deltaRight = HONE_MAX_DELTA;
                if (deltaRight < -HONE_MAX_DELTA) deltaRight = -HONE_MAX_DELTA;
                double rightOut = prevHoneRight + deltaRight;

                // Deadband: avoid tiny oscillations
                if (Math.abs(leftOut) < HONE_DEADBAND) leftOut = 0.0;
                if (Math.abs(rightOut) < HONE_DEADBAND) rightOut = 0.0;

                // Apply honed powers (override manual control)
                if (frontLeftDrive != null) frontLeftDrive.setPower(leftOut);
                if (frontRightDrive != null) frontRightDrive.setPower(rightOut);
                if (backLeftDrive != null) backLeftDrive.setPower(leftOut);
                if (backRightDrive != null) backRightDrive.setPower(rightOut);

                prevHoneLeft = leftOut;
                prevHoneRight = rightOut;

                telemetry.addData("Hone", "ENGAGED");
                telemetry.addData("Hone valid", r.valid);
                telemetry.addData("Hone dist (in)", r.distanceIn);
                telemetry.addData("Hone left/right", "%.2f / %.2f", leftOut, rightOut);

            } else {
                // If we were honing and now stopped, restore motor modes for normal driving
                if (honingActive) {
                    honingActive = false;
                    // restore RUN_USING_ENCODER and BRAKE behavior
                    try {
                        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    } catch (Exception e) {
                        // ignore hardware exceptions
                    }
                    // clear smoothing memory
                    prevHoneLeft = 0.0;
                    prevHoneRight = 0.0;
                }

                // Re-apply manual drive values (already applied above, but re-apply here to be explicit)
                // Normalize the values so no wheel power exceeds 100%
                max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
                max = Math.max(max, Math.abs(backLeftPower));
                max = Math.max(max, Math.abs(backRightPower));

                if (max > 1.0) {
                    frontLeftPower  /= max;
                    frontRightPower /= max;
                    backLeftPower   /= max;
                    backRightPower  /= max;
                }

                // Send calculated power to wheels
                frontLeftDrive.setPower(frontLeftPower);
                frontRightDrive.setPower(frontRightPower);
                backLeftDrive.setPower(backLeftPower);
                backRightDrive.setPower(backRightPower);

                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
                telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            }

            if(_popBall.getRise()) {
                //telemetry.addData("Gamepad2:A", "pressed");
            } else {
                //telemetry.addData("Gamepad2:A", "not pressed");
            }

            if (_toggleRobotDirection.getRise()){
                if (forwardDriving == true){
                    forwardDriving = false;
                }
                else {
                    forwardDriving = true;
                }
            }



            if(FeatureFlags.launchEnabled()) {

                if (_increaseLaunchPower.getRise()){
                    //_ballLauncher.increaseLauncherSpeedByRPM();
                    _ballLauncher.increaseLauncherSpeedByTPS();
                }

                if (_decreaseLaunchPower.getRise()){
                    //_ballLauncher.decreaseLauncherSpeed();
                    //_ballLauncher.decreaseLauncherSpeedByRPM();
                    _ballLauncher.decreaseLauncherSpeedByTPS();
                }
               // telemetry.addData("Launch:TPS", _ballLauncher.getCurrentTPS());
              //  telemetry.addData("Launch:RPM", _ballLauncher.getCurrentRPM());
                telemetry.addData("Launch:Left:Velocity", _ballLauncher.getLeftVelocity());
                telemetry.addData("Launch:Right:Velocity", _ballLauncher.getRightVelocity());


                if(_aimLauncherUp.getRise()){
                    _ballLauncher.aimLauncherUp();
                }
                if(_aimLauncherDown.getRise()){
                    _ballLauncher.aimLauncherDown();
                }

                telemetry.addData("Linear Servo Position", _ballLauncher.getCurrentPos());
            }

            if(FeatureFlags.turnTableEnabled()) {

                if (_increaseIndex.getRise()){
                   // telemetry.addData("Increase Index Presssed", "True");
                    //increase index
                    _turntable.increaseIndex();
                }
                else if(_decreaseIndex.getRise()){
                    //telemetry.addData("Decrease Index Presssed", "True");
                    //decrease index
                    _turntable.decreaseIndex();
                }

                //Trigger the state change if updated above

                _turntable.updateCurrentSlot();


            }

            if (FeatureFlags.ballSpoonerEnabled())
            {
                if(gamepad2.a){
                    _ballSpooner.fire();
                }
                _ballSpooner.updateSpoonerState();

            }

            if (FeatureFlags.goalPositioningEnabled()){
                if (isStopRequested()){
                    _goalPositioning.setStopRequested();
                }
                if (gamepad1.left_bumper){
                    _goalPositioning.find();
                }
            }

            if (FeatureFlags.limelightGoalPosotioningEnabled()){

            }
            if(FeatureFlags.ballIntakeEnabled()){

                if (gamepad1.right_trigger > 0){
                    _ballIntake.forward();
                }
                else{
                    _ballIntake.stop();
                }

                if (gamepad1.left_trigger > 0){
                    _ballIntake.reverse();
                }
                else {
                    stop();
                }


//                if (_toggleIntake.getRise()){
//                    _ballIntake.toggleIntake();
//                }
//                if (_reverseToggleIntake.getRise()){
//                    _ballIntake.reverseToggleIntake();
//                }
            }
            /*
            if (_pedroStart.getRise()){

                _pedroTesting.start();
            }
            */

            if (_shootAllBalls.getRise()){
                if (_shooterSequence != null) {
                    // Toggle behavior: start if not running, otherwise request stop.
                    if (!_shooterSequence.isRunning()) {
                        _shooterSequence.shootAllRapidlyAsync();
                    } else {
                        _shooterSequence.requestStop();
                    }
                }
            }




            // Determining the amount of red, green, and blue (fresh read each loop)
            if (colors != null) {
                telemetry.addData("Red", "%.3f", colors.red);
                telemetry.addData("Green", "%.3f", colors.green);
                telemetry.addData("Blue", "%.3f", colors.blue);
                telemetry.addData("Alpha", "%.3f", colors.alpha);
                // also show the raw light value if the sensor supports OpticalDistanceSensor
                try {
                    double light = ((OpticalDistanceSensor) colorSensor).getLightDetected();
                    telemetry.addData("LightDetected", "%.3f", light);
                } catch (Exception e) {
                    // not all NormalizedColorSensor implementations are OpticalDistanceSensor
                }
                if (colorSensorName != null) telemetry.addData("SensorName", colorSensorName);
                telemetry.addData("Gain", gain);


            } else if (colorSensor == null) {
                telemetry.addData("ColorSensor", "not found");
            } else {
                telemetry.addData("ColorSensor", "read error");
            }

            // Color-based ball detection (guarded by colors != null)
            if (colors != null) {
                if (colors.green > colors.blue && colors.green > 0.1){
                    telemetry.addLine("Green Ball Detected");
                    _lightController.SetLightTwo(LightColor.GREEN);
                }
                else if (colors.blue > colors.green){
                    telemetry.addLine("Purple Ball Detected");
                    _lightController.SetLightTwo(LightColor.VIOLET);
                }
                else if (colors.green < 0.1 && colors.red < 0.1 && colors.blue < 0.1) {
                    telemetry.addLine("No ball detected");
                    _lightController.SetLightTwo(LightColor.OFF);
                }
            }
             telemetry.update();

            // Show the elapsed game time and wheel power.
           // telemetry.addData("Status", "Run Time: " + runtime.toString());
          //  telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            //telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);

            if(FeatureFlags.launchEnabled())
            {

                if (_launcherPresetOne.getRise()){
                    _ballLauncher.setLaunchPresetOne();
                    _lightController.SetLightOne(LightColor.BLUE);

                }

                if (_launcherPresetTwo.getRise()){
                    _ballLauncher.setLaunchPresetTwo();
                    _lightController.SetLightOne(LightColor.YELLOW);
                }

                if (_launcherPresetThree.getRise()){
                    _ballLauncher.setLaunchPresetThree();
                    _lightController.SetLightOne(LightColor.WHITE);
                }

                if (_turnOffLauncher.getRise()){
                    _ballLauncher.turnOffLauncher();
                    _lightController.SetLightOne(LightColor.OFF);
                }




                telemetry.addData("Launch Power", _ballLauncher.getCurrentPower());
                //telemetry.addData("LauncherRPM", _ballLauncher.currentRPM());
                //telemetry.addData("Current TPS", _ballLauncher.currentTPS());

                telemetry.addData("Left Launch Power", _ballLauncher.getLeftLaunchPower());
                telemetry.addData("Right Launch Power", _ballLauncher.getRightLaunchPower());
            }

            if(FeatureFlags.turnTableEnabled())
            {
                telemetry.addData("Turntable Slot", _turntable.currentSlot());
                telemetry.addData("Turntable Pos", _turntable._currentPosition);
            }

        }
        hone.stop();
    }

    void hardwareInit() {



        _lightController = new LightController(hardwareMap, telemetry);


        if (FeatureFlags.launchEnabled()) {
            _ballLauncher = new BallLauncher(hardwareMap, telemetry);
        }

        if(FeatureFlags.turnTableEnabled()) {
            _turntable = new Turntable(hardwareMap, telemetry);
        }

        if(FeatureFlags.ballIntakeEnabled()) {
            _ballIntake = new BallIntake(hardwareMap, telemetry);
        }

        if (FeatureFlags.ballSpoonerEnabled()){
            _ballSpooner = new BallSpooner(hardwareMap, telemetry);
            _ballSpooner.init();
        }
        if(FeatureFlags.turnTableEnabled()) {
            //_turntable = new Turntable(hardwareMap, telemetry);
            if (FeatureFlags.ballSpoonerEnabled()) {
                _turntable = new Turntable(hardwareMap, telemetry, _ballSpooner);
            } else {
                _turntable = new Turntable(hardwareMap, telemetry);
            }
        }



        if (FeatureFlags.goalPositioningEnabled()){
            _goalPositioning = new GoalPositioning(hardwareMap, telemetry, goalAprilTag);
        }

        _shooterSequence = new ShooterSequence(_ballLauncher, _turntable, _ballSpooner, telemetry);

        //_pedroTesting = new PedroTesting(hardwareMap, telemetry);

        //Gamepad 1 configuration
        _driverOneGamepad = new DisasterGamePad(gamepad1);
        _toggleRobotDirection = new DebouncedButton(_driverOneGamepad.getAButton());

        //Gamepad 2 configuration
        _driverTwoGamepad = new DisasterGamePad(gamepad2);
        _decreaseLaunchPower = new DebouncedButton(_driverTwoGamepad.getDpadLeft());
        _increaseLaunchPower = new DebouncedButton(_driverTwoGamepad.getDpadRight());
        _decreaseIndex = new DebouncedButton(_driverTwoGamepad.getLeftBumper());
        _increaseIndex = new DebouncedButton(_driverTwoGamepad.getRightBumper());
        _popBall = new DebouncedButton(_driverTwoGamepad.getAButton());
        _aimLauncherDown = new DebouncedButton(_driverTwoGamepad.getDpadDown());
        _aimLauncherUp = new DebouncedButton(_driverTwoGamepad.getDpadUp());
        _launcherPresetOne = new DebouncedButton(_driverTwoGamepad.getXButton());
        _launcherPresetTwo = new DebouncedButton(_driverTwoGamepad.getYButton());
        _launcherPresetThree = new DebouncedButton(_driverTwoGamepad.getBButton());
        _turnOffLauncher = new DebouncedButton(_driverTwoGamepad.getLeftStickButton());
        _shootAllBalls = new DebouncedButton(_driverTwoGamepad.getRightStickButton());

        //_pedroStart = new DebouncedButton(_driverOneGamepad.getYButton());
    }

    public void setGoalAprilTag(int aprilTagID){
        goalAprilTag = aprilTagID;
    }

    double limelghtMountAngleDegrees = 0;

    double limelightLensHeightInches = 18;

    double goalHeightInches = 60;

    private double getDistanceIn(double ty){
        double angleToGoalDegrees = limelghtMountAngleDegrees + ty;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180);

        //distanceFromLimelightToGoalInches
        return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    }

//    private double calculateCurrentPower(double ty){
//
//            _ballAimer.calculateCurrentZone(getDistanceIn(ty));
//            _ballAimer.calculateDesiredPower();
//             return _ballAimer.calculateDesiredPower();
//
//
//    }


}
