package org.firstinspires.ftc.teamcode.pedroPathing.Examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Close Position Red", group = "Examples")
public class ClosePositionRed extends OpMode {

    private Follower follower;
    private Timer timer;
    private boolean waiting;
    private int pathState;
    private Paths paths;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);
        timer = new Timer();
        waiting = false;
        pathState = 0;

        follower.setStartingPose(new Pose(123.246, 122.986, Math.toRadians(219)));

        telemetry.addData("Status", "Initialized - Paths Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.followPath(paths.Path1);
        pathState = 1;
    }

    @Override
    public void loop() {
        follower.update();

        switch (pathState) {
            case 1: // Path1 finished → wait 1s → Path2
                if (!follower.isBusy()) {
                    if (!waiting) {
                        waiting = true;
                        timer.resetTimer();
                    } else if (timer.getElapsedTimeSeconds() >= 1.0) {
                        waiting = false;
                        follower.followPath(paths.Path2);
                        pathState = 2;
                    }
                }
                break;

            case 2: // Path2 finished → Path3 immediately
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3);
                    pathState = 3;
                }
                break;

            case 3: // Path3 finished → wait 1s → Path4
                if (!follower.isBusy()) {
                    if (!waiting) {
                        waiting = true;
                        timer.resetTimer();
                    } else if (timer.getElapsedTimeSeconds() >= 1.0) {
                        waiting = false;
                        follower.followPath(paths.Path4);
                        pathState = 4;
                    }
                }
                break;

            case 4: // Path4 finished → Path5 immediately
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5);
                    pathState = 5;
                }
                break;

            case 5: // Path5 finished → wait 1s → Path6
                if (!follower.isBusy()) {
                    if (!waiting) {
                        waiting = true;
                        timer.resetTimer();
                    } else if (timer.getElapsedTimeSeconds() >= 1.0) {
                        waiting = false;
                        follower.followPath(paths.Path6);
                        pathState = 6;
                    }
                }
                break;

            case 6: // Path6 finished → done
                if (!follower.isBusy()) {
                    pathState = -1;
                }
                break;

            default:
                break;
        }

        telemetry.addData("Path State", pathState);
        telemetry.addData("Waiting", waiting);
        telemetry.addData("Timer", timer.getElapsedTimeSeconds());
        telemetry.update();
    }

    // ---------------- PATHS CLASS ----------------
    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(123.246, 122.986),
                            new Pose(96.342, 95.058)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(219))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(96.342, 95.058),
                            new Pose(103.260, 84.296),
                            new Pose(125.296, 84.552)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(125.296, 84.552),
                            new Pose(96.342, 94.801)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(219))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(96.342, 94.801),
                            new Pose(81.993, 57.392),
                            new Pose(126.833, 60.723)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(126.833, 60.723),
                            new Pose(96.086, 94.545)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(219))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(96.086, 94.545),
                            new Pose(105.566, 33.307)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();
        }
    }
}
