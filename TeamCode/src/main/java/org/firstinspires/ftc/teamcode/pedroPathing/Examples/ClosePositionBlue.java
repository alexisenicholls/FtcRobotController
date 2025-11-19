package org.firstinspires.ftc.teamcode.pedroPathing.Examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose; // Using Pose instead of Point
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Close Position Blue - 19 Path Auto", group = "Examples")
public class ClosePositionBlue extends OpMode {

    private Follower follower;
    private Timer delayTimer;

    private int pathState = 1;
    private boolean pathStarted = false;

    private DcMotor intake;

    // Poses (Positions + Headings)
    private final Pose startPose = new Pose(20.719, 122.763, Math.toRadians(323));
    private final Pose scorePose = new Pose(60.086, 83.396, Math.toRadians(316));

    // Maintain 19 Paths as requested
    private PathChain p1, p2, p3, p4, p5, p6, p7, p8, p9, p10;
    private PathChain p11, p12, p13, p14, p15, p16, p17, p18, p19;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        delayTimer = new Timer();

        intake = hardwareMap.dcMotor.get("intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower.setStartingPose(startPose);
        buildPaths();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    private void buildPaths() {

        // P1: Start -> Score Preload
        // Note: We use "new Pose()" instead of "new Point()" to ensure compatibility
        p1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(startPose.getX(), startPose.getY(), 0), new Pose(scorePose.getX(), scorePose.getY(), 0)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        // P2: Score -> Strafe Out (Setup for Push)
        p2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(scorePose.getX(), scorePose.getY(), 0), new Pose(42.820, 83.396, 0)))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();

        // P3: Continue Strafe
        p3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(42.820, 83.396, 0), new Pose(36.950, 83.396, 0)))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();

        // P4: Continue Strafe to Setup
        p4 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(36.950, 83.396, 0), new Pose(31.942, 83.396, 0)))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();

        // P5: Setup -> Score
        p5 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(31.942, 83.396, 0), new Pose(scorePose.getX(), scorePose.getY(), 0)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), scorePose.getHeading())
                .build();

        // P6: Score -> Back up to prepare for Sample 1
        p6 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(scorePose.getX(), scorePose.getY(), 0), new Pose(22.446, 83.741, 0)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(90))
                .build();

        // P7: Redundant move
        p7 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(22.446, 83.741, 0), new Pose(scorePose.getX(), scorePose.getY(), 0)))
                .setLinearHeadingInterpolation(Math.toRadians(90), scorePose.getHeading())
                .build();

        // P8: Curve to Sample 1
        p8 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(scorePose.getX(), scorePose.getY(), 0),
                        new Pose(63.022, 60.259, 0),
                        new Pose(43.511, 60.604, 0)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(90))
                .build();

        // P9: Inch forward to Grab Sample 1
        p9 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(43.511, 60.604, 0), new Pose(36.604, 60.604, 0)))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        // P10: Inch forward more
        p10 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(36.604, 60.604, 0), new Pose(31.942, 60.604, 0)))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        // P11: Sample 1 -> Score
        p11 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(31.942, 60.604, 0), new Pose(scorePose.getX(), scorePose.getY(), 0)))
                .setLinearHeadingInterpolation(Math.toRadians(90), scorePose.getHeading())
                .build();

        // P12: Score -> Curve to Sample 2
        p12 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(scorePose.getX(), scorePose.getY(), 0),
                        new Pose(63.022, 59.914, 0),
                        new Pose(23.482, 59.914, 0)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(90))
                .build();

        // P13: Sample 2 -> Score
        p13 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(23.482, 59.914, 0), new Pose(scorePose.getX(), scorePose.getY(), 0)))
                .setLinearHeadingInterpolation(Math.toRadians(90), scorePose.getHeading())
                .build();

        // P14: Score -> Curve to Sample 3
        p14 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(scorePose.getX(), scorePose.getY(), 0),
                        new Pose(58.360, 36.086, 0),
                        new Pose(45.065, 35.914, 0)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(90))
                .build();

        // P15: Inch forward to Grab Sample 3
        p15 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(45.065, 35.914, 0), new Pose(37.813, 35.914, 0)))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        // P16: Inch forward more
        p16 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(37.813, 35.914, 0), new Pose(32.460, 35.914, 0)))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        // P17: Sample 3 -> Score
        p17 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(32.460, 35.914, 0), new Pose(scorePose.getX(), scorePose.getY(), 0)))
                .setLinearHeadingInterpolation(Math.toRadians(90), scorePose.getHeading())
                .build();

        // P18: Score -> Park Ascent
        p18 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(scorePose.getX(), scorePose.getY(), 0), new Pose(22.446, 39.885, 0)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(229))
                .build();

        // P19: Park Final Adjustment
        p19 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(22.446, 39.885, 0), new Pose(60.086, 83.050, 0)))
                .setLinearHeadingInterpolation(Math.toRadians(229), Math.toRadians(316))
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
            // Delay 1s
            case 1: runPath(p1, 0.0, 1.0, 1.0); break;

            // Normal
            case 2: runPath(p2, 0.0, 0.0, 1.0); break;

            // Intake On
            case 3: runPath(p3, 1.0, 0.0, 1.0); break;

            // Intake On
            case 4: runPath(p4, 1.0, 0.0, 1.0); break;

            // Delay 1s
            case 5: runPath(p5, 0.0, 1.0, 1.0); break;

            // Intake On, Slow Mode (0.60)
            case 6: runPath(p6, 1.0, 0.0, 0.60); break;

            // Delay 1s
            case 7: runPath(p7, 0.0, 1.0, 1.0); break;

            // Normal
            case 8: runPath(p8, 0.0, 0.0, 1.0); break;

            // Intake On
            case 9: runPath(p9, 1.0, 0.0, 1.0); break;

            // Intake On
            case 10: runPath(p10, 1.0, 0.0, 1.0); break;

            // Delay 1s
            case 11: runPath(p11, 0.0, 1.0, 1.0); break;

            // Intake On, Slow Mode (0.60)
            case 12: runPath(p12, 1.0, 0.0, 0.60); break;

            // Delay 1s
            case 13: runPath(p13, 0.0, 1.0, 1.0); break;

            // Normal
            case 14: runPath(p14, 0.0, 0.0, 1.0); break;

            // Intake On
            case 15: runPath(p15, 1.0, 0.0, 1.0); break;

            // Intake On
            case 16: runPath(p16, 1.0, 0.0, 1.0); break;

            // Delay 1s
            case 17: runPath(p17, 0.0, 1.0, 1.0); break;

            // Intake On
            case 18: runPath(p18, 1.0, 0.0, 1.0); break;

            // Delay 1s
            case 19: runPath(p19, 0.0, 1.0, 1.0); break;

            case 20:
                intake.setPower(0);
                telemetry.addData("Status", "Auto Complete");
                break;
        }

        telemetry.addData("State", pathState);
        telemetry.addData("Intake", intake.getPower());
        telemetry.update();
    }

    private void runPath(PathChain path, double intakePower, double delaySeconds, double speedMultiplier) {
        if (!pathStarted) {
            // Changed setMovementSpeed to setMaxPower
            follower.setMaxPower(speedMultiplier);
            follower.followPath(path);
            intake.setPower(intakePower);
            pathStarted = true;
            delayTimer.resetTimer();
        }

        if (follower.isBusy()) {
            // Reset the timer while we are still moving
            delayTimer.resetTimer();
        } else {
            // Only increment state if we are done moving AND the delay has passed
            if (delayTimer.getElapsedTimeSeconds() >= delaySeconds) {
                pathState++;
                pathStarted = false;
            }
        }
    }
}