package org.firstinspires.ftc.teamcode.testing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.season2025.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

// MAKE STATE MACHINE
@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroBlue extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private static final int START = 0;
    private static final int SHOOT_1 = 1;
    private static final int PREP_SLURP_1 = 2;
    private static final int SLURP_1 = 3;
    private static final int SHOOT_2 = 4;
    private static final int PREP_SLURP_2 = 5;
    private static final int SLURP_2 = 6;
    private static final int SHOOT_3 = 7;
    private static final int PREP_SLURP_3 = 8;
    private static final int SLURP_3 = 9;
    private static final int PREP_LEVER = 10;
    private static final int LEVER_PUSH = 11;
    private static final int DONE = 12;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower);

        pathState = START;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }


    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }


    public static class Paths {
        public PathChain Shoot1;
        public PathChain PrepareSlurp1;
        public PathChain Slurp1;
        public PathChain Shoot2;
        public PathChain PrepareSlurp2;
        public PathChain Slurp2;
        public PathChain Shoot3;
        public PathChain PrepareSlurp3;
        public PathChain Slurp3;
        public PathChain PrepareLever;
        public PathChain LeverPush;

        public Paths(Follower follower) {
            Shoot1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(63.429, 8.656),

                                    new Pose(48.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))

                    .build();

            PrepareSlurp1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 96.000),

                                    new Pose(48.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            Slurp1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 84.000),

                                    new Pose(24.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Shoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.000, 84.000),

                                    new Pose(48.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            PrepareSlurp2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 96.000),

                                    new Pose(48.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            Slurp2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 60.000),

                                    new Pose(24.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Shoot3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.000, 60.000),

                                    new Pose(48.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            PrepareSlurp3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 96.000),

                                    new Pose(48.000, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            Slurp3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 36.000),

                                    new Pose(24.000, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            PrepareLever = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.000, 36.000),

                                    new Pose(24.000, 70.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            LeverPush = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.000, 70.000),

                                    new Pose(17.000, 70.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();
        }
    }


    public int autonomousPathUpdate() {

        switch (pathState) {

            case START:
                follower.followPath(paths.Shoot1, true);
                pathState = SHOOT_1;
                break;

            case SHOOT_1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PrepareSlurp1, true);
                    pathState = PREP_SLURP_1;
                }
                break;

            case PREP_SLURP_1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Slurp1, true);
                    pathState = SLURP_1;
                }
                break;

            case SLURP_1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Shoot2, true);
                    pathState = SHOOT_2;
                }
                break;

            case SHOOT_2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PrepareSlurp2, true);
                    pathState = PREP_SLURP_2;
                }
                break;

            case PREP_SLURP_2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Slurp2, true);
                    pathState = SLURP_2;
                }
                break;

            case SLURP_2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Shoot3, true);
                    pathState = SHOOT_3;
                }
                break;

            case SHOOT_3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PrepareSlurp3, true);
                    pathState = PREP_SLURP_3;
                }
                break;

            case PREP_SLURP_3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Slurp3, true);
                    pathState = SLURP_3;
                }
                break;

            case SLURP_3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PrepareLever, true);
                    pathState = PREP_LEVER;
                }
                break;

            case PREP_LEVER:
                if (!follower.isBusy()) {
                    follower.followPath(paths.LeverPush, true);
                    pathState = LEVER_PUSH;
                }
                break;

            case LEVER_PUSH:
                if (!follower.isBusy()) {
                    pathState = DONE;
                }
                break;

            case DONE:
                // Autonomous finished
                break;
        }

        return pathState;
    }



}
    