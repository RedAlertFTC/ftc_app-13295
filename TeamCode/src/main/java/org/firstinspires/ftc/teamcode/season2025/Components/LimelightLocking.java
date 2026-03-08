package org.firstinspires.ftc.teamcode.season2025.Components;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimelightLocking
{
    private Limelight3A limelight;
    private HardwareMap _hardwareMap;
    private Telemetry _telemetry;

    public LimelightLocking(HardwareMap hardwareMap, Telemetry telemetry){
        _hardwareMap = hardwareMap;
        _telemetry = telemetry;
        init();
    }
    public void lockAprilTag(){
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();
                _telemetry.addData("tx", result.getTx());
                _telemetry.addData("ty", result.getTy());
                _telemetry.addData("Botpose", botpose.toString());
                _telemetry.update();
            }
        }
    }
    public void init(){
        limelight = _hardwareMap.get(Limelight3A.class, "limelight");
    }

}
