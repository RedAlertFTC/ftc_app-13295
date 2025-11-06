package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.nfc.TagLostException;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PedroTesting
{
    private final Pose startPoint = new Pose(0, 0 , Math.toRadians(0));
    private final Pose endPoint = new Pose(100, 50, Math.toRadians(20));

    private PathChain triangle;
    private Path testPath;
    private Follower follower;
    private Telemetry _telemetry;
    private HardwareMap _hardwareMap;
    public PedroTesting(HardwareMap hardwareMap, Telemetry telemetry)
    {
        hardwareMap = _hardwareMap;
        telemetry = _telemetry;
    }

    public void start()
    {
        init();
        follower.update();
        follower.followPath(testPath,true);

        _telemetry.addData("x", follower.getPose().getX());
        _telemetry.addData("y", follower.getPose().getY());
        _telemetry.addData("heading", follower.getPose().getHeading());
    }
    private void buildPaths()
    {
        testPath = new Path(new BezierLine(startPoint, endPoint));
        testPath.setLinearHeadingInterpolation(startPoint.getHeading(), endPoint.getHeading());
    }


    public void init() {


        follower = Constants.createFollower(_hardwareMap);
        buildPaths();
        follower.setStartingPose(startPoint);

    }

}

