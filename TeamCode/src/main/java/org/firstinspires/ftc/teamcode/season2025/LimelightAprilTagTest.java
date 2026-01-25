package org.firstinspires.ftc.teamcode.season2025;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.LimelightHelpers;

@TeleOp(name = "Limelight AprilTag Test", group = "Testing")
public class LimelightAprilTagTest extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Limelight name
    private static final String LIMELIGHT_NAME = "limelight";

    // Desired pose
    private static final double DESIRED_Z = 1.27; // meters (50 inches)

    // Gains
    private static final double kP_Z = 0.8;
    private static final double kP_X = 0.8;
    private static final double kP_YAW = 0.02;

    // Limits
    private static final double MAX_DRIVE = 0.5;
    private static final double MAX_STRAFE = 0.5;
    private static final double MAX_TURN = 0.4;

    // Deadbands
    private static final double Z_DEADBAND = 0.05;
    private static final double X_DEADBAND = 0.05;
    private static final double YAW_DEADBAND = 2.0;

    @Override
    public void runOpMode() {

        // -------------------- Hardware --------------------
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Limelight AprilTag Test Ready");
        telemetry.addLine("Hold LEFT BUMPER to drive to tag");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.left_bumper &&
                    LimelightHelpers.getTV(LIMELIGHT_NAME)) {

                double[] pose =
                        LimelightHelpers.getBotPose_TargetSpace(LIMELIGHT_NAME);

                // botpose_targetspace:
                // [x, y, z, roll, pitch, yaw]
                double xError = pose[0];
                double zError = pose[2] - DESIRED_Z;
                double yawError = pose[5];

                // Deadbands
                if (Math.abs(xError) < X_DEADBAND) xError = 0;
                if (Math.abs(zError) < Z_DEADBAND) zError = 0;
                if (Math.abs(yawError) < YAW_DEADBAND) yawError = 0;

                double forward = Range.clip(zError * kP_Z, -MAX_DRIVE, MAX_DRIVE);
                double strafe  = Range.clip(xError * kP_X, -MAX_STRAFE, MAX_STRAFE);
                double turn    = Range.clip(yawError * kP_YAW, -MAX_TURN, MAX_TURN);

                driveRobotCentric(forward, strafe, turn);

                telemetry.addLine("TARGET LOCKED");
                telemetry.addData("X Error (m)", xError);
                telemetry.addData("Z Error (m)", zError);
                telemetry.addData("Yaw Error (deg)", yawError);

            } else {
                driveRobotCentric(0, 0, 0);
                telemetry.addLine("NO TARGET / NOT ACTIVE");
            }

            telemetry.update();
        }
    }

    // -----------------------------------------------------
    // Mecanum Drive
    // -----------------------------------------------------

    private void driveRobotCentric(double forward, double strafe, double turn) {

        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        double max = Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))
        );

        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }
}
