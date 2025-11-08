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

@Autonomous(name = "Close Position Blue", group = "Examples")
public class ClosePositionBlue extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    // Define all key poses
    private final Pose startPose = new Pose(21.149, 122.229, Math.toRadians(322));
    private final Pose p1End = new Pose(22.393, 83.974, Math.toRadians(180));
    private final Pose p2End = new Pose(54.739, 91.438, Math.toRadians(322));
    private final Pose p3End = new Pose(23.948, 60.337, Math.toRadians(180));
    private final Pose p4End = new Pose(54.739, 91.127, Math.toRadians(322));

    // PathChain objects
    private PathChain path1, path2, path3, path4;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        pathTimer = new Timer();

        buildPaths();

        follower.setStartingPose(startPose);

        telemetry.addData("Status", "Initialized - Paths built");
        telemetry.update();
    }

    private void buildPaths() {
        // Path 1: BezierCurve (start → p1End)
        path1 = follower.pathBuilder()
                .addPath(new BezierCurve(startPose,
                        new Pose(56.605, 92.994),
                        p1End))
                .setLinearHeadingInterpolation(
                        Math.toRadians(322),
                        Math.toRadians(180))
                .build();

        // Path 2: BezierLine (p1End → p2End)
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(p1End, p2End))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(322))
                .build();

        // Path 3: BezierCurve (p2End → p3End)
        path3 = follower.pathBuilder()
                .addPath(new BezierCurve(p2End,
                        new Pose(52.251, 65.002),
                        p3End))
                .setTangentHeadingInterpolation()
                .build();

        // Path 4: BezierLine (p3End → p4End)
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(p3End, p4End))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(322))
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
                    follower.followPath(path2);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
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
                if (!follower.isBusy()) {
                    setPathState(-1); // Done
                }
                break;
        }
    }

    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
}
