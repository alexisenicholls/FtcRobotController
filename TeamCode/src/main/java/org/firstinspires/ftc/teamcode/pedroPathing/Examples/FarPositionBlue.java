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

/**
 * Autonomous OpMode that follows 14 sequential paths.
 * Includes 1-second pauses after paths 2, 5, 8, 11, and 14.
 */
@Autonomous(name = "Far Position Auto Blue", group = "Examples")
public class FarPositionBlue extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private boolean isPaused;

    // --- Start Pose ---
    private final Pose startPose = new Pose(56.000, 8.000, Math.toRadians(90));

    // --- Path Endpoints ---
    private final Pose p1End = new Pose(21.267, 35.356, Math.toRadians(180));
    private final Pose p2End = new Pose(67.132, 17.933, Math.toRadians(310));
    private final Pose p3End = new Pose(24.342, 17.933, 0);
    private final Pose p4End = new Pose(12.556, 17.933, 0);
    private final Pose p5End = new Pose(66.876, 17.933, Math.toRadians(310));
    private final Pose p6End = new Pose(24.086, 18.189, 0);
    private final Pose p7End = new Pose(12.556, 17.933, 0);
    private final Pose p8End = new Pose(66.107, 17.677, Math.toRadians(310));
    private final Pose p9End = new Pose(24.086, 18.189, 0);
    private final Pose p10End = new Pose(12.300, 17.933, 0);
    private final Pose p11End = new Pose(66.620, 17.677, Math.toRadians(310));
    private final Pose p12End = new Pose(24.598, 17.933, 0);
    private final Pose p13End = new Pose(12.300, 17.933, 0);
    private final Pose p14End = new Pose(66.876, 17.677, Math.toRadians(310));

    // --- Control Point (for Path1’s BezierCurve) ---
    private final Pose p1Ctrl1 = new Pose(51.246, 32.026, 0);

    // --- PathChains ---
    private PathChain path1, path2, path3, path4, path5, path6, path7, path8,
            path9, path10, path11, path12, path13, path14;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        pathTimer = new Timer();
        buildPaths();

        follower.setStartingPose(startPose);

        telemetry.addData("Status", "Initialized - 14 paths built");
        telemetry.update();
    }

    private void buildPaths() {
        // Path 1: BezierCurve
        path1 = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, p1Ctrl1, p1End))
                .setLinearHeadingInterpolation(startPose.getHeading(), Math.toRadians(180))
                .build();

        // Path 2
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(p1End, p2End))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(310))
                .build();

        // Path 3
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(p2End, p3End))
                .setTangentHeadingInterpolation()
                .build();

        // Path 4
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(p3End, p4End))
                .setTangentHeadingInterpolation()
                .build();

        // Path 5
        path5 = follower.pathBuilder()
                .addPath(new BezierLine(p4End, p5End))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(310))
                .build();

        // Path 6
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(p5End, p6End))
                .setTangentHeadingInterpolation()
                .build();

        // Path 7
        path7 = follower.pathBuilder()
                .addPath(new BezierLine(p6End, p7End))
                .setTangentHeadingInterpolation()
                .build();

        // Path 8
        path8 = follower.pathBuilder()
                .addPath(new BezierLine(p7End, p8End))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(310))
                .build();

        // Path 9
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(p8End, p9End))
                .setTangentHeadingInterpolation()
                .build();

        // Path 10
        path10 = follower.pathBuilder()
                .addPath(new BezierLine(p9End, p10End))
                .setTangentHeadingInterpolation()
                .build();

        // Path 11
        path11 = follower.pathBuilder()
                .addPath(new BezierLine(p10End, p11End))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(310))
                .build();

        // Path 12
        path12 = follower.pathBuilder()
                .addPath(new BezierLine(p11End, p12End))
                .setTangentHeadingInterpolation()
                .build();

        // Path 13
        path13 = follower.pathBuilder()
                .addPath(new BezierLine(p12End, p13End))
                .setTangentHeadingInterpolation()
                .build();

        // Path 14
        path14 = follower.pathBuilder()
                .addPath(new BezierLine(p13End, p14End))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(310))
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
        telemetry.addData("Paused", isPaused);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    private void autonomousPathUpdate() {
        if (isPaused) {
            // wait until 1 second passes
            if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                isPaused = false;
                pathState++; // continue to next path
            }
            return;
        }

        switch (pathState) {
            case 0: follower.followPath(path1); setPathState(1); break;
            case 1: if (!follower.isBusy()) { follower.followPath(path2); setPathState(2); } break;
            case 2: if (!follower.isBusy()) { pauseBeforeNextPath(); } break; // pause after path 2
            case 3: if (!follower.isBusy()) { follower.followPath(path3); setPathState(4); } break;
            case 4: if (!follower.isBusy()) { follower.followPath(path4); setPathState(5); } break;
            case 5: if (!follower.isBusy()) { pauseBeforeNextPath(); } break; // pause after path 5
            case 6: if (!follower.isBusy()) { follower.followPath(path6); setPathState(7); } break;
            case 7: if (!follower.isBusy()) { follower.followPath(path7); setPathState(8); } break;
            case 8: if (!follower.isBusy()) { pauseBeforeNextPath(); } break; // pause after path 8
            case 9: if (!follower.isBusy()) { follower.followPath(path9); setPathState(10); } break;
            case 10: if (!follower.isBusy()) { follower.followPath(path10); setPathState(11); } break;
            case 11: if (!follower.isBusy()) { pauseBeforeNextPath(); } break; // pause after path 11
            case 12: if (!follower.isBusy()) { follower.followPath(path12); setPathState(13); } break;
            case 13: if (!follower.isBusy()) { follower.followPath(path13); setPathState(14); } break;
            case 14: if (!follower.isBusy()) { pauseBeforeNextPath(); } break; // pause after path 14
            case 15: if (!follower.isBusy()) { setPathState(-1); } break; // finished
        }
    }

    private void pauseBeforeNextPath() {
        isPaused = true;
        pathTimer.resetTimer();
    }

    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
        isPaused = false;
    }
}
