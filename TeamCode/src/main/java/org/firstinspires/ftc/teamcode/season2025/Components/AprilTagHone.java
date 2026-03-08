package org.firstinspires.ftc.teamcode.season2025.Components;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/**
 * Self-contained utility component that encapsulates AprilTag "hone-in" behavior using a Limelight3A
 * and optionally owns a 4-motor tank drive. This class centralizes everything you need for honing so
 * callers only interact with this single class.
 *
 * Typical usage patterns:
 *  - Full ownership (recommended for component-style code):
 *      AprilTagHone hone = new AprilTagHone();
 *      hone.initAll(hardwareMap, "limelight", "fl", "bl", "fr", "br");
 *      hone.start();
 *      while(opModeIsActive()) { AprilTagHone.HoneResult r = hone.honeStored(); sleep(20); }
 *      hone.stop();
 *
 *  - One-shot convenience:
 *      AprilTagHone hone = new AprilTagHone();
 *      AprilTagHone.HoneResult r = hone.honeOnce(hardwareMap, "limelight", "fl", "bl", "fr", "br");
 */
public class AprilTagHone {

    // Stored hardware references (optional)
    private HardwareMap hardwareMap;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;

    private Limelight3A limelight;

    // Turn PID constants (defaults copied from existing opmode)
    private double kP_turn = 0.01;
    private double kI_turn = 0.0002;
    private double kD_turn = 0.0005;
    private double turnDeadband = 1.0; // degrees

    // Forward (range) P controller
    private double kP_forward = 0.02;
    private double distanceDeadbandIn = 2.0; // inches

    // Limits
    private double maxTurnPower = 0.7;
    private double maxDrivePower = 0.5;

    // Desired stopping distance from tag (inches)
    private double targetDistanceIn = 36.0;

    // PID state
    private double integralTurn = 0.0;
    private double lastErrorTurn = 0.0;

    // Search rotation power when no target
    private double searchTurnPower = 0.2;

    // Aim offset (degrees). Positive values move the desired crosshair to the LEFT of the tag,
    // which makes the tag appear to the RIGHT (positive tx) at the setpoint.
    // Example: setAimOffsetDegrees(5.0) will make the controller aim so the tag's tx ~= +5deg,
    // i.e., the robot points ~5 degrees to the left of the tag.
    private double aimOffsetDegrees = 0.0;

    /**
     * Set desired aim offset in degrees. Positive -> aim to the left of the tag (tag appears to the right of center).
     */
    public void setAimOffsetDegrees(double offsetDeg) { this.aimOffsetDegrees = offsetDeg; }
    /** Get current aim offset (degrees). */
    public double getAimOffsetDegrees() { return this.aimOffsetDegrees; }

    public static class HoneResult {
        public boolean valid = false; // true if limelight had a valid target
        public double forwardPower = 0.0;
        public double turnPower = 0.0;
        public double leftPower = 0.0;
        public double rightPower = 0.0;
        public double distanceIn = Double.NaN; // computed distance in inches if available
    }

    public AprilTagHone() {}

    // ---------- Hardware initialization helpers ----------

    /** Initialize limelight only. */
    public void initLimelight(HardwareMap hardwareMap, String limelightName) {
        this.hardwareMap = hardwareMap;
        if (hardwareMap != null && limelightName != null) {
            try {
                this.limelight = hardwareMap.get(Limelight3A.class, limelightName);
            } catch (Exception e) {
                this.limelight = null;
            }
        }
    }

    /** Initialize drive motors only (names may be null to skip). */
    public void initMotors(HardwareMap hardwareMap, String frontLeftName, String backLeftName, String frontRightName, String backRightName) {
        this.hardwareMap = hardwareMap;
        if (hardwareMap == null) return;
        try { this.frontLeft = hardwareMap.get(DcMotor.class, frontLeftName); } catch (Exception e) { this.frontLeft = null; }
        try { this.backLeft  = hardwareMap.get(DcMotor.class, backLeftName);  } catch (Exception e) { this.backLeft = null; }
        try { this.frontRight= hardwareMap.get(DcMotor.class, frontRightName);} catch (Exception e) { this.frontRight = null; }
        try { this.backRight = hardwareMap.get(DcMotor.class, backRightName); } catch (Exception e) { this.backRight = null; }

        // Default directions: left motors reversed to match common tank-drive conventions
        if (this.frontLeft != null) this.frontLeft.setDirection(DcMotor.Direction.REVERSE);
        if (this.backLeft  != null) this.backLeft.setDirection(DcMotor.Direction.REVERSE);
        if (this.frontRight!= null) this.frontRight.setDirection(DcMotor.Direction.FORWARD);
        if (this.backRight != null) this.backRight.setDirection(DcMotor.Direction.FORWARD);
    }

    /** Initialize limelight and motors together (optional motor names). */
    public void initAll(HardwareMap hardwareMap, String limelightName, String frontLeftName, String backLeftName, String frontRightName, String backRightName) {
        initLimelight(hardwareMap, limelightName);
        initMotors(hardwareMap, frontLeftName, backLeftName, frontRightName, backRightName);
    }

    /** Start the limelight processing (switch pipeline 0 by default). */
    public void start() {
        if (limelight != null) {
            try { limelight.pipelineSwitch(0); } catch (Exception ignored) {}
            try { limelight.start(); } catch (Exception ignored) {}
        }
    }

    /** Stop limelight processing. */
    public void stop() {
        if (limelight != null) {
            try { limelight.stop(); } catch (Exception ignored) {}
        }
    }

    /** Reset internal PID state. */
    public void reset() {
        integralTurn = 0.0;
        lastErrorTurn = 0.0;
    }

    // Configuration setters
    public void setTargetDistanceIn(double target) { this.targetDistanceIn = target; }
    public void setTurnPID(double p, double i, double d) { this.kP_turn = p; this.kI_turn = i; this.kD_turn = d; }
    public void setForwardP(double p) { this.kP_forward = p; }
    public void setTurnDeadband(double d) { this.turnDeadband = d; }
    public void setDistanceDeadbandIn(double d) { this.distanceDeadbandIn = d; }
    public void setMaxTurnPower(double p) { this.maxTurnPower = Math.abs(p); }
    public void setMaxDrivePower(double p) { this.maxDrivePower = Math.abs(p); }
    public void setSearchTurnPower(double p) { this.searchTurnPower = p; }

    // ---------- Core update logic (unchanged) ----------
    public HoneResult update() {
        HoneResult r = new HoneResult();
        if (limelight == null) {
            // No limelight: return zero outputs
            r.valid = false;
            r.forwardPower = 0.0;
            r.turnPower = 0.0;
            r.leftPower = 0.0;
            r.rightPower = 0.0;
            return r;
        }

        LLResult result = null;
        try {
            result = limelight.getLatestResult();
        } catch (Exception e) {
            result = null;
        }

        if (result != null && result.isValid()) {
            r.valid = true;
            r.distanceIn = Double.NaN;

            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                double x = botpose.getPosition().x;
                double y = botpose.getPosition().y;
                double z = botpose.getPosition().z;
                double distanceCm = Math.sqrt(x*x + y*y + z*z) * 100.0; // meters->cm
                r.distanceIn = distanceCm * 0.393701;
            }

            // Turn PID - use aimOffsetDegrees as the setpoint so we don't point exactly at the tag
            // but instead offset the crosshair by aimOffsetDegrees (see setter docs above).
            double error = result.getTx() - aimOffsetDegrees;
            integralTurn += error;
            double derivative = error - lastErrorTurn;
            r.turnPower = (kP_turn * error) + (kI_turn * integralTurn) + (kD_turn * derivative);
            lastErrorTurn = error;

            // Deadband and clamp
            if (Math.abs(error) < turnDeadband) {
                r.turnPower = 0.0;
                integralTurn = 0.0;
            }
            if (r.turnPower > maxTurnPower) r.turnPower = maxTurnPower;
            if (r.turnPower < -maxTurnPower) r.turnPower = -maxTurnPower;

            // Forward P control based on distance (if available)
            if (!Double.isNaN(r.distanceIn)) {
                double forwardError = r.distanceIn - targetDistanceIn;
                if (forwardError > distanceDeadbandIn) {
                    r.forwardPower = kP_forward * forwardError;
                    if (r.forwardPower > maxDrivePower) r.forwardPower = maxDrivePower;
                } else {
                    r.forwardPower = 0.0;
                }
            } else {
                r.forwardPower = 0.0; // cannot compute range -> don't drive forward
            }

        } else {
            // No valid tag: rotate slowly to search
            r.valid = false;
            r.forwardPower = 0.0;
            r.turnPower = searchTurnPower;
            integralTurn = 0.0;
            lastErrorTurn = 0.0;
        }

        // Combine forward and turn into left/right powers
        r.leftPower = r.forwardPower + r.turnPower;
        r.rightPower = r.forwardPower - r.turnPower;

        // Normalize so we never exceed max absolute 1.0
        double max = Math.max(Math.abs(r.leftPower), Math.abs(r.rightPower));
        if (max > 1.0) {
            r.leftPower /= max;
            r.rightPower /= max;
        }

        return r;
    }

    /** Convenience: apply the latest computed powers directly to a 4-motor tank drive. */
    public void applyToDrive(DcMotor frontLeft, DcMotor backLeft, DcMotor frontRight, DcMotor backRight) {
        HoneResult r = update();
        if (frontLeft != null) frontLeft.setPower(r.leftPower);
        if (backLeft != null) backLeft.setPower(r.leftPower);
        if (frontRight != null) frontRight.setPower(r.rightPower);
        if (backRight != null) backRight.setPower(r.rightPower);
    }

    /** Convenience that applies to the stored motors owned by this class. */
    public HoneResult honeStored() {
        HoneResult r = update();
        if (frontLeft != null) frontLeft.setPower(r.leftPower);
        if (backLeft != null) backLeft.setPower(r.leftPower);
        if (frontRight != null) frontRight.setPower(r.rightPower);
        if (backRight != null) backRight.setPower(r.rightPower);
        return r;
    }

    /** One-shot convenience method using stored motors: init/start must be performed externally. */
    public HoneResult honeOnceStored() {
        HoneResult r = update();
        if (frontLeft != null) frontLeft.setPower(r.leftPower);
        if (backLeft != null) backLeft.setPower(r.leftPower);
        if (frontRight != null) frontRight.setPower(r.rightPower);
        if (backRight != null) backRight.setPower(r.rightPower);
        return r;
    }

    /** Helper to check whether the limelight currently sees a valid tag. */
    public boolean isTargetVisible() {
        if (limelight == null) return false;
        try {
            LLResult result = limelight.getLatestResult();
            return result != null && result.isValid();
        } catch (Exception e) {
            return false;
        }
    }

    /** Returns the underlying Limelight object (may be null). */
    public Limelight3A getLimelight() { return limelight; }

    /**
     * One-shot convenience method that performs a complete hone action using the provided HardwareMap
     * and limelight + motor names: it will init/start the limelight if needed, run a single update, apply powers
     * to the given motors, then stop the limelight. Returns the HoneResult produced by the update.
     *
     * Useful for callers that want a single-call hone without managing the lifecycle.
     */
    public HoneResult honeOnce(HardwareMap hardwareMap, String limelightName,
                               String frontLeftName, String backLeftName, String frontRightName, String backRightName) {
        // If caller supplied names, initialize hardware references locally
        initAll(hardwareMap, limelightName, frontLeftName, backLeftName, frontRightName, backRightName);
        if (this.limelight != null) {
            try { this.limelight.pipelineSwitch(0); } catch (Exception ignored) {}
            try { this.limelight.start(); } catch (Exception ignored) {}
        }

        HoneResult r = update();

        if (frontLeft != null) frontLeft.setPower(r.leftPower);
        if (backLeft != null) backLeft.setPower(r.leftPower);
        if (frontRight != null) frontRight.setPower(r.rightPower);
        if (backRight != null) backRight.setPower(r.rightPower);

        if (this.limelight != null) {
            try { this.limelight.stop(); } catch (Exception ignored) {}
        }


        return r;
    }
}
