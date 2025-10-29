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
 * Auto that follows the 10-path trajectory exported from PedroPathing (.pp JSON).
 * Correctly uses setTangentHeadingInterpolation() per official Pedro docs.
 */
@Autonomous(name = "Auto From Trajectory (Fixed)", group = "Examples")
public class Auto extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    // Start pose and endpoints (from your JSON)
    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));

    private final Pose p1End = new Pose(55.7, 36, Math.toRadians(180));
    private final Pose p2End = new Pose(13.333333333333332, 35.911111111111104, Math.toRadians(0));
    private final Pose p3End = new Pose(37.15555555555556, 115.37777777777778, Math.toRadians(320));
    private final Pose p4End = new Pose(53.33333333333333, 60.62222222222222, Math.toRadians(0));
    private final Pose p5End = new Pose(13.68888888888889, 60.266666666666666, Math.toRadians(0));
    private final Pose p6End = new Pose(37.33333333333333, 115.02222222222221, Math.toRadians(320));
    private final Pose p7End = new Pose(45.86666666666667, 84.8, Math.toRadians(0));
    private final Pose p8End = new Pose(11.911111111111111, 83.91111111111111, Math.toRadians(0));
    private final Pose p9End = new Pose(36.97777777777778, 115.37777777777778, Math.toRadians(320));
    private final Pose p10End = new Pose(38.577777777777776, 33.42222222222222, Math.toRadians(0));

    // Control points from JSON
    private final Pose p3Control = new Pose(61.68888888888889, 40.7111111111111, 0);
    private final Pose p6Control = new Pose(72, 81, 0);

    // PathChain objects (pathBuilder().build() returns PathChain)
    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path10;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        pathTimer = new Timer();

        buildPaths();

        follower.setStartingPose(startPose);

        telemetry.addData("Status", "Initialized - paths built");
        telemetry.update();
    }

    private void buildPaths() {
        // Path 1: BezierLine startPose -> p1End, linear heading interpolation
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, p1End))
                .setLinearHeadingInterpolation(startPose.getHeading(), p1End.getHeading())
                .build();

        // Path 2: BezierLine p1End -> p2End, use TANGENT interpolation (correct method name)
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(p1End, p2End))
                .setTangentHeadingInterpolation()   // <--- correct method
                .build();

        // Path 3: BezierCurve p2End -> p3End with control point, constant heading 320 deg
        path3 = follower.pathBuilder()
                .addPath(new BezierCurve(p2End, p3Control, p3End))
                .setConstantHeadingInterpolation(Math.toRadians(320))
                .build();

        // Path 4: BezierLine p3End -> p4End, tangential heading
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(p3End, p4End))
                .setTangentHeadingInterpolation()
                .build();

        // Path 5: BezierLine p4End -> p5End, tangential heading
        path5 = follower.pathBuilder()
                .addPath(new BezierLine(p4End, p5End))
                .setTangentHeadingInterpolation()
                .build();

        // Path 6: BezierCurve p5End -> p6End with control point, constant heading 320 deg
        path6 = follower.pathBuilder()
                .addPath(new BezierCurve(p5End, p6Control, p6End))
                .setConstantHeadingInterpolation(Math.toRadians(320))
                .build();

        // Path 7: BezierLine p6End -> p7End, tangential heading
        path7 = follower.pathBuilder()
                .addPath(new BezierLine(p6End, p7End))
                .setTangentHeadingInterpolation()
                .build();

        // Path 8: BezierLine p7End -> p8End, tangential heading
        path8 = follower.pathBuilder()
                .addPath(new BezierLine(p7End, p8End))
                .setTangentHeadingInterpolation()
                .build();

        // Path 9: BezierLine p8End -> p9End, constant 320 deg
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(p8End, p9End))
                .setConstantHeadingInterpolation(Math.toRadians(320))
                .build();

        // Path 10: BezierLine p9End -> p10End, constant 0 deg
        path10 = follower.pathBuilder()
                .addPath(new BezierLine(p9End, p10End))
                .setConstantHeadingInterpolation(Math.toRadians(0))
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
                if (!follower.isBusy()) { follower.followPath(path2); setPathState(2); }
                break;
            case 2:
                if (!follower.isBusy()) { follower.followPath(path3); setPathState(3); }
                break;
            case 3:
                if (!follower.isBusy()) { follower.followPath(path4); setPathState(4); }
                break;
            case 4:
                if (!follower.isBusy()) { follower.followPath(path5); setPathState(5); }
                break;
            case 5:
                if (!follower.isBusy()) { follower.followPath(path6); setPathState(6); }
                break;
            case 6:
                if (!follower.isBusy()) { follower.followPath(path7); setPathState(7); }
                break;
            case 7:
                if (!follower.isBusy()) { follower.followPath(path8); setPathState(8); }
                break;
            case 8:
                if (!follower.isBusy()) { follower.followPath(path9); setPathState(9); }
                break;
            case 9:
                if (!follower.isBusy()) { follower.followPath(path10); setPathState(10); }
                break;
            case 10:
                if (!follower.isBusy()) { setPathState(-1); } // finished
                break;
        }
    }

    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
}
