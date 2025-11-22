package org.firstinspires.ftc.teamcode.pedroPathing.Examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "FarPositionRed", group = "Examples")
public class FarPositionRed extends OpMode {

    private Follower follower;
    private Timer delayTimer;

    private int pathState = 0;
    private boolean pathStarted = false;
    private boolean shootingPhase = false;

    private DcMotorEx shooter;

    // START POSE set exactly to the start of Path1
    private final Pose startPose = new Pose(87.540, 8.115, Math.toRadians(90));

    private PathChain[] paths;
    private boolean[] shooterPaths;

    // Robot speed (50%)
    private final double robotSpeed = 0.5;

    // Shooter timing
    private final double PRE_SPIN_TIME = 0;
    private final double SHOOT_TIME = 7.0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        delayTimer = new Timer();

        shooter = hardwareMap.get(DcMotorEx.class, Constants.SHOOTER_MOTOR_NAME);
        shooter.setDirection(Constants.SHOOTER_DIRECTION);
        shooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        follower.setStartingPose(startPose);

        buildPaths();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    private void buildPaths() {
        paths = new PathChain[2];
        shooterPaths = new boolean[2];

        // Path1 (far position red)
        paths[0] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(87.540, 8.115), new Pose(87.540, 18.129)))
                .setTangentHeadingInterpolation()
                .build();
        shooterPaths[0] = false;

        // Path2 (far position red)
        paths[1] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(87.540, 18.129), new Pose(82.014, 18.129)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(249))
                .build();
        shooterPaths[1] = true; // shoot on Path2
    }

    @Override
    public void start() {
        pathState = 0;
        pathStarted = false;
        shootingPhase = false;
        delayTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();

        if (pathState < paths.length) {
            boolean isShooterPath = shooterPaths[pathState];

            if (!pathStarted) {
                follower.setMaxPower(robotSpeed);
                follower.followPath(paths[pathState]);
                shooter.setPower(0.0);

                pathStarted = true;
                delayTimer.resetTimer();
            }

            if (!follower.isBusy()) {
                double elapsed = delayTimer.getElapsedTimeSeconds();

                if (isShooterPath) {
                    if (!shootingPhase) {
                        shooter.setPower(1.0);
                        if (elapsed >= PRE_SPIN_TIME) {
                            shootingPhase = true;
                            delayTimer.resetTimer();
                        }
                    } else {
                        if (elapsed < SHOOT_TIME) {
                            shooter.setPower(1.0);
                        } else {
                            shooter.setPower(0.0);
                            pathState++;
                            pathStarted = false;
                            shootingPhase = false;
                            delayTimer.resetTimer();
                        }
                    }
                } else {
                    pathState++;
                    pathStarted = false;
                    delayTimer.resetTimer();
                }
            }
        } else {
            shooter.setPower(0.0);
            telemetry.addData("Status", "Complete");
        }

        telemetry.addData("Active Path", Math.min(pathState + 1, paths.length));
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Shooter Power", shooter.getPower());
        telemetry.update();
    }
}
