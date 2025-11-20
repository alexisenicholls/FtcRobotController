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

@Autonomous(name = "ClosePositionRed", group = "Examples")
public class ClosePositionRed extends OpMode {

    private Follower follower;
    private Timer delayTimer;
    private int pathState = 0;  // Current path index (0-9)

    private boolean pathStarted = false;

    private DcMotor intake;

    // Updated start pose to match red logic
    private final Pose startPose = new Pose(123.626, 122.763, Math.toRadians(216));

    private PathChain[] paths;
    private double[] intakePowers;
    private double[] delays;

    private final double normalSpeed = 1.0;
    private final double intakeSpeed = 0.6;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        delayTimer = new Timer();

        intake = hardwareMap.dcMotor.get("intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower.setStartingPose(startPose);

        // --- Build paths ---
        buildPaths();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    private void buildPaths() {

        paths = new PathChain[10];
        intakePowers = new double[10];
        delays = new double[10];

        // ---- PATH 1 ----
        paths[0] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(123.626, 122.763), new Pose(83.914, 83.396)))
                .setLinearHeadingInterpolation(Math.toRadians(216), Math.toRadians(229))
                .build();
        intakePowers[0] = 0.0;
        delays[0] = 1.0;

        // ---- PATH 2 ----
        paths[1] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(83.914, 83.396), new Pose(111.540, 83.396)))
                .setTangentHeadingInterpolation()
                .build();
        intakePowers[1] = 0.5;
        delays[1] = 0.0;

        // ---- PATH 3 ----
        paths[2] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(111.540, 83.396), new Pose(83.914, 83.223)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(229))
                .build();
        intakePowers[2] = 0.0;
        delays[2] = 1.0;

        // ---- PATH 4 ----
        paths[3] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(83.914, 83.223), new Pose(117.237, 83.223)))
                .setTangentHeadingInterpolation()
                .build();
        intakePowers[3] = 0.5;
        delays[3] = 0.0;

        // ---- PATH 5 ----
        paths[4] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(117.237, 83.223), new Pose(84.086, 83.396)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(229))
                .build();
        intakePowers[4] = 0.0;
        delays[4] = 1.0;

        // ---- PATH 6 ----
        paths[5] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(84.086, 83.396), new Pose(83.914, 59.050)))
                .setTangentHeadingInterpolation()
                .build();
        intakePowers[5] = 0.0;
        delays[5] = 0.0;

        // ---- PATH 7 ----
        paths[6] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(83.914, 59.050), new Pose(111.022, 59.223)))
                .setTangentHeadingInterpolation()
                .build();
        intakePowers[6] = 0.5;
        delays[6] = 0.0;

        // ---- PATH 8 ----
        paths[7] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(111.022, 59.223), new Pose(83.914, 83.223)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(229))
                .build();
        intakePowers[7] = 0.0;
        delays[7] = 1.0;

        // ---- PATH 9 ----
        paths[8] = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(83.914, 83.223),
                        new Pose(73.554, 59.223),
                        new Pose(116.719, 59.223)))
                .setTangentHeadingInterpolation()
                .build();
        intakePowers[8] = 0.5;
        delays[8] = 0.0;

        // ---- PATH 10 ----
        paths[9] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(116.719, 59.223), new Pose(83.914, 83.396)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(229))
                .build();
        intakePowers[9] = 0.0;
        delays[9] = 1.0;
    }

    @Override
    public void start() {
        pathState = 0;
        pathStarted = false;
        delayTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();

        if (pathState < paths.length) {
            double intakePower = intakePowers[pathState];
            double delaySec = delays[pathState];
            double speed = (intakePower > 0) ? intakeSpeed : normalSpeed;

            runPath(paths[pathState], intakePower, delaySec, speed);
        } else {
            intake.setPower(0);
            telemetry.addData("Status", "Auto Complete");
        }

        telemetry.addData("Path", pathState + 1);
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.update();
    }

    private void runPath(PathChain path, double intakePower, double delaySeconds, double speed) {

        follower.setMaxPower(speed);

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
