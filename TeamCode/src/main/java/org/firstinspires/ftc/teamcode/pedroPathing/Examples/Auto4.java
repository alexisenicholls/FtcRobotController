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
 * Auto with 4 paths and 1-second delays after paths 2 and 4.
 * Uses Constants.createFollower() and same architecture as the official PedroPathing example.
 */
@Autonomous(name = "Auto With Delays (4 Paths)", group = "Examples")
public class Auto4 extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    // Starting pose
    private final Pose startPose = new Pose(25.497, 129.513, Math.toRadians(324));

    // Path endpoints
    private final Pose p1End = new Pose(22.889, 83.445, Math.toRadians(180));
    private final Pose p2End = new Pose(41.722, 101.698, Math.toRadians(324));
    private final Pose p3End = new Pose(21.730, 59.396, Math.toRadians(180));
    private final Pose p4End = new Pose(41.722, 101.408, Math.toRadians(324));

    // Control points
    private final Pose p1Control = new Pose(59.107, 93.006, 0);
    private final Pose p3Control = new Pose(67.509, 70.117, 0);

    // PathChain objects
    private PathChain path1, path2, path3, path4;

    private boolean delayActive = false;  // used to track delay state

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        pathTimer = new Timer();

        buildPaths();
        follower.setStartingPose(startPose);

        telemetry.addData("Status", "Initialized - Paths Built");
        telemetry.update();
    }

    private void buildPaths() {
        // Path 1: BezierCurve startPose -> p1End
        path1 = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, p1Control, p1End))
                .setLinearHeadingInterpolation(Math.toRadians(324), Math.toRadians(180))
                .build();

        // Path 2: BezierLine p1End -> p2End
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(p1End, p2End))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(324))
                .build();

        // Path 3: BezierCurve p2End -> p3End
        path3 = follower.pathBuilder()
                .addPath(new BezierCurve(p2End, p3Control, p3End))
                .setLinearHeadingInterpolation(Math.toRadians(324), Math.toRadians(180))
                .build();

        // Path 4: BezierLine p3End -> p4End
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(p3End, p4End))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(324))
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
        telemetry.addData("Delay Active", delayActive);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(path1);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    startDelay(2); // delay after path 2
                    follower.followPath(path2);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy() && !delayActive) {
                    startDelay(3); // delay after path 4 will happen later
                    follower.followPath(path3);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(path4);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy() && !delayActive) {
                    // Delay after path 4
                    delayActive = true;
                    pathTimer.resetTimer();
                }
                if (delayActive && pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    delayActive = false;
                    setPathState(-1); // Finished
                }
                break;
        }

        // Handle delay timing globally
        if (delayActive && pathTimer.getElapsedTimeSeconds() >= 1.0) {
            delayActive = false;
        }
    }

    private void startDelay(int nextState) {
        delayActive = true;
        pathTimer.resetTimer();
    }

    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
}
