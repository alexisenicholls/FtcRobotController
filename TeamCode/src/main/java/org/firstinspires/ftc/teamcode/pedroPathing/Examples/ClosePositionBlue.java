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

import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Close Position Blue", group = "Examples")
public class ClosePositionBlue extends OpMode {

    private Follower follower;
    private Timer delayTimer;

    private int pathState = 0;
    private boolean delayActive = false;

    // Intake motor reference
    private DcMotor intake;

    // Starting pose
    private final Pose startPose = new Pose(20.719, 122.763, Math.toRadians(323));

    // All 13 PathChains
    private PathChain p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        delayTimer = new Timer();

        // === Intake Initialization ===
        intake = hardwareMap.dcMotor.get(Constants.INTAKE_MOTOR_NAME);
        intake.setDirection(Constants.INTAKE_DIRECTION);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        buildPaths();
        follower.setStartingPose(startPose);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    private void buildPaths() {
        p1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(20.719, 122.763), new Pose(60.086, 83.396)))
                .setLinearHeadingInterpolation(Math.toRadians(323), Math.toRadians(316))
                .build();

        p2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60.086, 83.396), new Pose(30.734, 83.396)))
                .setTangentHeadingInterpolation()
                .build();

        p3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(30.734, 83.396), new Pose(60.086, 83.396)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(316))
                .build();

        p4 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60.086, 83.396), new Pose(21.755, 83.396)))
                .setTangentHeadingInterpolation()
                .build();

        p5 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(21.755, 83.396), new Pose(60.086, 83.396)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(316))
                .build();

        p6 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(60.086, 83.396),
                        new Pose(54.216, 59.568),
                        new Pose(31.079, 59.568)))
                .setTangentHeadingInterpolation()
                .build();

        p7 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(31.079, 59.568), new Pose(60.086, 83.568)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(316))
                .build();

        p8 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60.086, 83.568), new Pose(22.446, 62.331)))
                .setTangentHeadingInterpolation()
                .build();

        p9 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(22.446, 62.331), new Pose(59.914, 83.741)))
                .setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(316))
                .build();

        p10 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(59.914, 83.741),
                        new Pose(66.820, 36.777),
                        new Pose(31.424, 36.432)))
                .setTangentHeadingInterpolation()
                .build();

        p11 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(31.424, 36.432), new Pose(59.914, 83.568)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(316))
                .build();

        p12 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(59.914, 83.568), new Pose(22.791, 39.194)))
                .setTangentHeadingInterpolation()
                .build();

        p13 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(22.791, 39.194), new Pose(59.741, 83.568)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(316))
                .build();
    }

    @Override
    public void start() {
        pathState = 1;
        follower.followPath(p1);
        intake.setPower(0);  // Off for path 1
    }

    @Override
    public void loop() {
        follower.update();
        updateStateMachine();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Delay Active", delayActive);
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.update();
    }

    private void updateStateMachine() {

        // Handle global delay
        if (delayActive && delayTimer.getElapsedTimeSeconds() >= 1.0) {
            delayActive = false;
            pathState++;
        }

        if (delayActive) {
            intake.setPower(0); // always off during delays
            return;
        }

        // === Turn intake ON for paths 2,4,6,8,10,12 ===
        boolean intakeShouldRun =
                (pathState == 2 || pathState == 4 || pathState == 6 ||
                        pathState == 8 || pathState == 10 || pathState == 12);

        intake.setPower(intakeShouldRun ? -1.0 : 0);

        switch (pathState) {

            case 1:
                if (!follower.isBusy()) triggerDelay();
                break;

            case 2:
                follower.followPath(p2);
                pathState++;
                break;

            case 3:
                if (!follower.isBusy()) triggerDelay();
                break;

            case 4:
                follower.followPath(p3);
                pathState++;
                break;

            case 5:
                if (!follower.isBusy()) triggerDelay();
                break;

            case 6:
                follower.followPath(p4);
                pathState++;
                break;

            case 7:
                if (!follower.isBusy()) triggerDelay();
                break;

            case 8:
                follower.followPath(p5);
                pathState++;
                break;

            case 9:
                if (!follower.isBusy()) triggerDelay();
                break;

            case 10:
                follower.followPath(p6);
                pathState++;
                break;

            case 11:
                if (!follower.isBusy()) triggerDelay();
                break;

            case 12:
                follower.followPath(p7);
                pathState++;
                break;

            case 13:
                if (!follower.isBusy()) triggerDelay();
                break;

            case 14:
                follower.followPath(p8);
                pathState++;
                break;

            case 15:
                if (!follower.isBusy()) triggerDelay();
                break;

            case 16:
                follower.followPath(p9);
                pathState++;
                break;

            case 17:
                if (!follower.isBusy()) triggerDelay();
                break;

            case 18:
                follower.followPath(p10);
                pathState++;
                break;

            case 19:
                if (!follower.isBusy()) triggerDelay();
                break;

            case 20:
                follower.followPath(p11);
                pathState++;
                break;

            case 21:
                if (!follower.isBusy()) triggerDelay();
                break;

            case 22:
                follower.followPath(p12);
                pathState++;
                break;

            case 23:
                if (!follower.isBusy()) triggerDelay();
                break;

            case 24:
                follower.followPath(p13);
                pathState++;
                break;

            case 25:
                if (!follower.isBusy()) triggerDelay();
                break;

            case 26:
                intake.setPower(0); // Stop intake at end
                break;
        }
    }

    private void triggerDelay() {
        delayActive = true;
        intake.setPower(0);  // Turn off during delay
        delayTimer.resetTimer();
    }
}
