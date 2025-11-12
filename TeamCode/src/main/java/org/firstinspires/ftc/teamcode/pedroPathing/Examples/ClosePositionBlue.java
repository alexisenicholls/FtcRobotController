package org.firstinspires.ftc.teamcode.pedroPathing.Examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Close Position Blue", group = "Examples")
public class ClosePositionBlue extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private boolean waiting; // used to start the 1s timer only after path finishes

    // --- Define all Poses ---
    private final Pose startPose = new Pose(21.149, 122.229, Math.toRadians(322));
    private final Pose p1End = new Pose(60.337, 83.352, Math.toRadians(0));
    private final Pose p2End = new Pose(23.326, 83.974, Math.toRadians(180));
    private final Pose p3End = new Pose(60.337, 83.352, Math.toRadians(322));
    private final Pose p4End = new Pose(24.259, 59.200, Math.toRadians(180));
    private final Pose p5End = new Pose(60.337, 83.352, Math.toRadians(322));

    // --- Path objects ---
    private PathChain path1, path2, path3, path4, path5;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        pathTimer = new Timer();
        waiting = false;

        buildPaths();

        follower.setStartingPose(startPose);

        telemetry.addData("Status", "Initialized - Paths built");
        telemetry.update();
    }

    private void buildPaths() {
        // Path 1: BezierLine (start → p1End)
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, p1End))
                .setTangentHeadingInterpolation()
                .build();

        // Path 2: BezierLine (p1End → p2End)
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(p1End, p2End))
                .setTangentHeadingInterpolation()
                .build();

        // Path 3: BezierLine (p2End → p3End)
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(p2End, p3End))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(322))
                .build();

        // Path 4: BezierCurve (p3End → p4End)
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        p3End,
                        new Pose(58.471, 60.337),
                        p4End))
                .setTangentHeadingInterpolation()
                .build();

        // Path 5: BezierLine (p4End → p5End)
        path5 = follower.pathBuilder()
                .addPath(new BezierLine(p4End, p5End))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(322))
                .build();
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Timer (s)", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Waiting", waiting);
        telemetry.update();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // start path1, then enter wait-for-finish state (1)
                follower.followPath(path1);
                setPathState(1);
                break;

            case 1: // Wait for Path 1 to finish, then delay 1s
                if (!follower.isBusy()) {
                    if (!waiting) {
                        // first frame after path finished -> start delay timer
                        waiting = true;
                        pathTimer.resetTimer();
                    } else if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                        // after 1 second delay, start next path
                        waiting = false;
                        follower.followPath(path2);
                        setPathState(2);
                    }
                }
                break;

            case 2:
                // run path2 -> immediately go to path3 when finished (no delay)
                if (!follower.isBusy()) {
                    follower.followPath(path3);
                    setPathState(3);
                }
                break;

            case 3: // Wait for Path 3 to finish, then delay 1s
                if (!follower.isBusy()) {
                    if (!waiting) {
                        waiting = true;
                        pathTimer.resetTimer();
                    } else if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                        waiting = false;
                        follower.followPath(path4);
                        setPathState(4);
                    }
                }
                break;

            case 4:
                // run path4 -> immediately go to path5 when finished (no delay)
                if (!follower.isBusy()) {
                    follower.followPath(path5);
                    setPathState(5);
                }
                break;

            case 5: // Wait for Path 5 to finish, then delay 1s before finishing
                if (!follower.isBusy()) {
                    if (!waiting) {
                        waiting = true;
                        pathTimer.resetTimer();
                    } else if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                        waiting = false;
                        setPathState(-1); // Done
                    }
                }
                break;

            default:
                // do nothing (finished or unhandled state)
                break;
        }
    }

    private void setPathState(int newState) {
        pathState = newState;
        // do not pre-reset the timer for the "waiting after finish" logic;
        // reset waiting only when state changes
        waiting = false;
        pathTimer.resetTimer();
    }
}
