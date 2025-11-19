package org.firstinspires.ftc.teamcode.pedroPathing.Examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "15-Path Auto With Intake", group = "Examples")
public class FarPositionBlue extends OpMode {

    private Follower follower;
    private Timer delayTimer;
    private int pathState = 1;
    private boolean pathStarted = false;

    private DcMotor intake;

    // === PATHS ===
    private PathChain p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15;

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        delayTimer = new Timer();

        // === Intake motor setup ===
        intake = hardwareMap.dcMotor.get("intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        buildPaths();

        // Set ANY starting pose you want — you're using static paths already
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    private void buildPaths() {

        p1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.000, 8.000), new Pose(60.086, 21.065)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(295))
                .build();

        p2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(60.086, 21.065),
                        new Pose(60.777, 35.741),
                        new Pose(41.266, 35.741)))
                .setTangentHeadingInterpolation()
                .build();

        p3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(41.266, 35.741), new Pose(37.640, 35.741)))
                .setTangentHeadingInterpolation()
                .build();

        p4 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(37.640, 35.741), new Pose(33.151, 35.741)))
                .setTangentHeadingInterpolation()
                .build();

        p5 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(33.151, 35.741), new Pose(60.086, 20.892)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(295))
                .build();

        p6 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60.086, 20.892), new Pose(26.245, 32.978)))
                .setTangentHeadingInterpolation()
                .build();

        p7 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(26.245, 32.978), new Pose(60.086, 20.892)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(295))
                .build();

        p8 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60.086, 20.892), new Pose(15.367, 13.813)))
                .setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(180))
                .build();

        p9 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(15.367, 13.813), new Pose(11.050, 13.468)))
                .setTangentHeadingInterpolation()
                .build();

        p10 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(11.050, 13.468), new Pose(16.921, 11.568)))
                .setTangentHeadingInterpolation()
                .build();

        p11 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(16.921, 11.568), new Pose(10.705, 10.878)))
                .setTangentHeadingInterpolation()
                .build();

        p12 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(10.705, 10.878), new Pose(60.259, 20.892)))
                .setLinearHeadingInterpolation(Math.toRadians(195), Math.toRadians(295))
                .build();

        p13 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60.259, 20.892), new Pose(9.669, 22.273)))
                .setTangentHeadingInterpolation()
                .build();

        p14 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(9.669, 22.273), new Pose(10.878, 11.396)))
                .setTangentHeadingInterpolation()
                .build();

        p15 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(10.878, 11.396), new Pose(60.432, 20.892)))
                .setLinearHeadingInterpolation(Math.toRadians(274), Math.toRadians(295))
                .build();
    }

    @Override
    public void start() {
        pathState = 1;
        pathStarted = false;
        delayTimer.resetTimer();
    }

    @Override
    public void loop() {

        follower.update();

        switch (pathState) {

            case 1:
                runPath(p1, 0, 1.0);  // Delay path
                break;

            case 2:
                runPath(p2, 0, 0);
                break;

            case 3:
                runPath(p3, 1.0, 0);  // Intake ON
                break;

            case 4:
                runPath(p4, 1.0, 0);  // Intake ON
                break;

            case 5:
                runPath(p5, 0, 1.0);  // Delay path
                break;

            case 6:
                runPath(p6, 1.0, 0);  // Intake ON
                break;

            case 7:
                runPath(p7, 0, 1.0);  // Delay path
                break;

            case 8:
                runPath(p8, 0, 0);
                break;

            case 9:
                runPath(p9, 1.0, 0);  // Intake ON
                break;

            case 10:
                runPath(p10, 0, 0);
                break;

            case 11:
                runPath(p11, 1.0, 0);  // Intake ON
                break;

            case 12:
                runPath(p12, 0, 1.0);  // Delay path
                break;

            case 13:
                runPath(p13, 0, 0);
                break;

            case 14:
                runPath(p14, 1.0, 0);  // Intake ON
                break;

            case 15:
                runPath(p15, 0, 1.0);  // Delay path
                break;

            case 16:
                intake.setPower(0);
                telemetry.addData("Status", "Auto Complete");
                break;
        }

        telemetry.addData("Path", pathState);
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.update();
    }

    // === CLEAN HELPER ===
    private void runPath(PathChain path, double intakePower, double delaySeconds) {

        if (!pathStarted) {
            follower.followPath(path);
            intake.setPower(intakePower);
            pathStarted = true;
            delayTimer.resetTimer();
        }

        if (follower.isBusy()) {
            delayTimer.resetTimer();
        } else {

            if (delayTimer.getElapsedTimeSeconds() >= delaySeconds) {
                pathState++;
                pathStarted = false;
            }
        }
    }
}
