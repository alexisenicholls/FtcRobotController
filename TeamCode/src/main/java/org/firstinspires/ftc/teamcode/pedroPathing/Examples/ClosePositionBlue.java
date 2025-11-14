package org.firstinspires.ftc.teamcode.pedroPathing.Examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

@Autonomous(name = "Close Position Blue", group = "Examples")
public class ClosePositionBlue extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private boolean waiting;
    private boolean aligning;

    // --- Define all Poses ---
    private final Pose startPose = new Pose(21.149, 122.229, Math.toRadians(322));
    private final Pose p1End = new Pose(60.337, 83.352, Math.toRadians(0));
    private final Pose p2End = new Pose(23.326, 83.974, Math.toRadians(180));
    private final Pose p3End = new Pose(60.337, 83.352, Math.toRadians(322));
    private final Pose p4End = new Pose(24.259, 59.200, Math.toRadians(180));
    private final Pose p5End = new Pose(60.337, 83.352, Math.toRadians(322));

    // --- Path objects ---
    private PathChain path1, path2, path3, path4, path5;

    // Motors for Limelight alignment
    private DcMotor fl, fr, bl, br;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        pathTimer = new Timer();
        waiting = false;
        aligning = false;

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        buildPaths();
        follower.setStartingPose(startPose);

        telemetry.addLine("Initialized - Paths built and Limelight active");
        telemetry.update();
    }

    private void buildPaths() {
        path1 = follower.pathBuilder().addPath(new BezierLine(startPose, p1End))
                .setTangentHeadingInterpolation().build();
        path2 = follower.pathBuilder().addPath(new BezierLine(p1End, p2End))
                .setTangentHeadingInterpolation().build();
        path3 = follower.pathBuilder().addPath(new BezierLine(p2End, p3End))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(322)).build();
        path4 = follower.pathBuilder().addPath(new BezierCurve(p3End, new Pose(58.471, 60.337), p4End))
                .setTangentHeadingInterpolation().build();
        path5 = follower.pathBuilder().addPath(new BezierLine(p4End, p5End))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(322)).build();
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
        telemetry.addData("Waiting", waiting);
        telemetry.addData("Aligning", aligning);
        telemetry.addData("Timer (s)", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(path1);
                setPathState(1);
                break;

            case 1:
                // After Path 1: Align + wait
                if (!follower.isBusy() && !aligning && !waiting) {
                    aligning = true;
                    alignWithAprilTag();
                } else if (aligning && !isAligningActive()) {
                    aligning = false;
                    waiting = true;
                    pathTimer.resetTimer();
                } else if (waiting && pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    waiting = false;
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
                // After Path 3: Align + wait
                if (!follower.isBusy() && !aligning && !waiting) {
                    aligning = true;
                    alignWithAprilTag();
                } else if (aligning && !isAligningActive()) {
                    aligning = false;
                    waiting = true;
                    pathTimer.resetTimer();
                } else if (waiting && pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    waiting = false;
                    follower.followPath(path4);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(path5);
                    setPathState(5);
                }
                break;

            case 5:
                // After Path 5: Align + wait, then finish
                if (!follower.isBusy() && !aligning && !waiting) {
                    aligning = true;
                    alignWithAprilTag();
                } else if (aligning && !isAligningActive()) {
                    aligning = false;
                    waiting = true;
                    pathTimer.resetTimer();
                } else if (waiting && pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    waiting = false;
                    setPathState(-1);
                }
                break;

            default:
                break;
        }
    }

    /**
     * Uses Limelight to rotate until the AprilTag is centered (tx near 0°)
     */
    private void alignWithAprilTag() {
        LLResult result = Constants.getLatestVisionResult();
        if (result == null || !result.isValid()) {
            stopAllMotors();
            return;
        }

        FiducialResult tag = Constants.getPrimaryAprilTag();
        if (tag == null) {
            stopAllMotors();
            return;
        }

        double tx = tag.getTargetXDegrees();
        double rotatePower = 0.0;

        if (Math.abs(tx) > Constants.TX_DEADBAND) {
            rotatePower = -tx * Constants.ROTATE_KP;
            rotatePower = Math.max(-Constants.MAX_ROTATE_SPEED, Math.min(Constants.MAX_ROTATE_SPEED, rotatePower));
        }

        double flPower = -rotatePower;
        double blPower = -rotatePower;
        double frPower = rotatePower;
        double brPower = rotatePower;

        fl.setPower(flPower);
        bl.setPower(blPower);
        fr.setPower(frPower);
        br.setPower(brPower);

        // If tag is centered, stop alignment
        if (Math.abs(tx) <= Constants.TX_DEADBAND) {
            stopAllMotors();
        }
    }

    private boolean isAligningActive() {
        LLResult result = Constants.getLatestVisionResult();
        if (result == null || !result.isValid()) return false;
        FiducialResult tag = Constants.getPrimaryAprilTag();
        if (tag == null) return false;
        return Math.abs(tag.getTargetXDegrees()) > Constants.TX_DEADBAND;
    }

    private void stopAllMotors() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    private void setPathState(int newState) {
        pathState = newState;
        waiting = false;
        aligning = false;
        pathTimer.resetTimer();
    }
}
