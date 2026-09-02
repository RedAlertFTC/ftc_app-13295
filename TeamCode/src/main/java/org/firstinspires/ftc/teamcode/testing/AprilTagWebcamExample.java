package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.external.samples.externalhardware.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
@Autonomous
@Disabled
public class AprilTagWebcamExample extends OpMode {

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    @Override
    public void init(){
        aprilTagWebcam.init(hardwareMap, telemetry);
    }

    public void loop(){
        //update the vision portal
        aprilTagWebcam.update();
        AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificID(24);
        if(id24 == null) {
            telemetry.addData("ID24", "nope");
        } else {
            telemetry.addData("ID24", "yep");
        }
        aprilTagWebcam.displayDetectionTelemetry(id24);
    }
}
