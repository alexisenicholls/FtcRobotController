package org.firstinspires.ftc.teamcode.pedroPathing.Examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "FarPositionRed", group = "Examples")
public class FarPositionRed extends OpMode {

    private Follower follower;
    private Timer delayTimer;
    private Timer spinUpTimer;
    private Timer fireTimer;
    private boolean shotFiredInThisDelay = false;

    private int pathState = 0;
    private boolean pathStarted = false;

    private DcMotor intake;
    private DcMotorEx shooter;

    private final Pose startPose = new Pose(88.058, 8.115, 0.0);

    private PathChain[] paths;
    private boolean[] intakePaths;
    private double[] delays;

    private final double normalSpeed = 1.0;
    private final double intakePathSpeed = 0.6;

    private final double TARGET_WHEEL_RPM = 3200.0;
    private final double READY_PERCENT = 0.95;
    private final double SPINUP_FAILSAFE_SEC = 2.0;
    private final double FIRE_DURATION_SEC = 0.75;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        delayTimer = new Timer();
        spinUpTimer = new Timer();
        fireTimer = new Timer();

        intake = hardwareMap.dcMotor.get("intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter = hardwareMap.get(DcMotorEx.class, Constants.SHOOTER_MOTOR_NAME);
        shooter.setDirection(Constants.SHOOTER_DIRECTION);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(Constants.SHOOTER_kP,
                        Constants.SHOOTER_kI,
                        Constants.SHOOTER_kD,
                        Constants.SHOOTER_kF));

        follower.setStartingPose(startPose);
        buildPaths();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    private void spinShooterWheelRPM(double wheelRpm) {
        double motorRpm = wheelRpm / Constants.SHOOTER_GEAR_RATIO;
        double ticksPerRev = shooter.getMotorType().getTicksPerRev();
        double velocityTicksPerSec = (motorRpm / 60.0) * ticksPerRev;
        shooter.setVelocity(velocityTicksPerSec);
    }

    private void stopShooter() {
        shooter.setVelocity(0);
    }

    private double getCurrentWheelRPM() {
        double ticksPerSec = shooter.getVelocity();
        double ticksPerRev = shooter.getMotorType().getTicksPerRev();
        double motorRpm = (ticksPerSec * 60.0) / ticksPerRev;
        return motorRpm * Constants.SHOOTER_GEAR_RATIO;
    }

    private void buildPaths() {
        paths = new PathChain[10];
        intakePaths = new boolean[10];
        delays = new double[10];

        // ---- PATH 1 ----
        paths[0] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(88.058, 8.115), new Pose(88.230, 18.820)))
                .setTangentHeadingInterpolation()
                .build();
        intakePaths[0] = false;
        delays[0] = 0.0;

        // ---- PATH 2 ---- Shooter
        paths[1] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(88.230, 18.820), new Pose(84.604, 18.820)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(249))
                .build();
        intakePaths[1] = false;
        delays[1] = SPINUP_FAILSAFE_SEC;

        // ---- PATH 3 ----
        paths[2] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(84.604, 18.820), new Pose(84.950, 34.705)))
                .setTangentHeadingInterpolation()
                .build();
        intakePaths[2] = false;
        delays[2] = 0.0;

        // ---- PATH 4 ---- Intake
        paths[3] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(84.950, 34.705), new Pose(111.022, 34.705)))
                .setTangentHeadingInterpolation()
                .build();
        intakePaths[3] = true;
        delays[3] = 0.0;

        // ---- PATH 5 ---- Shooter
        paths[4] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(111.022, 34.705), new Pose(84.604, 18.820)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(249))
                .build();
        intakePaths[4] = false;
        delays[4] = SPINUP_FAILSAFE_SEC;

        // ---- PATH 6 ----
        paths[5] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(84.604, 18.820), new Pose(84.950, 34.705)))
                .setTangentHeadingInterpolation()
                .build();
        intakePaths[5] = false;
        delays[5] = 0.0;

        // ---- PATH 7 ---- Intake
        paths[6] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(84.950, 34.705), new Pose(116.719, 35.050)))
                .setTangentHeadingInterpolation()
                .build();
        intakePaths[6] = true;
        delays[6] = 0.0;

        // ---- PATH 8 ---- Shooter
        paths[7] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(116.719, 35.050), new Pose(84.777, 18.820)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(249))
                .build();
        intakePaths[7] = false;
        delays[7] = SPINUP_FAILSAFE_SEC;

        // ---- PATH 9 ---- Intake
        paths[8] = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(84.777, 18.820), new Pose(110.504, 11.223), new Pose(131.741, 11.050)))
                .setTangentHeadingInterpolation()
                .build();
        intakePaths[8] = true;
        delays[8] = 0.0;

        // ---- PATH 10 ---- Shooter
        paths[9] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(131.741, 11.050), new Pose(84.777, 18.820)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(249))
                .build();
        intakePaths[9] = false;
        delays[9] = SPINUP_FAILSAFE_SEC;
    }

    @Override
    public void start() {
        pathState = 0;
        pathStarted = false;
        delayTimer.resetTimer();
        spinUpTimer.resetTimer();
        fireTimer.resetTimer();
        shotFiredInThisDelay = false;
    }

    @Override
    public void loop() {
        follower.update();

        if (pathState < paths.length) {
            boolean isIntakePath = intakePaths[pathState];
            double delaySec = delays[pathState];
            double speed = isIntakePath ? intakePathSpeed : normalSpeed;

            runPath(paths[pathState], isIntakePath, delaySec, speed);
        } else {
            intake.setPower(0);
            stopShooter();
            telemetry.addData("Status", "Auto Complete");
        }

        telemetry.addData("Path", pathState + 1);
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.addData("Shooter RPM", String.format("%.1f", getCurrentWheelRPM()));
        telemetry.update();
    }

    private void runPath(PathChain path, boolean isIntakePath, double delaySeconds, double speed) {

        follower.setMaxPower(speed);

        if (!pathStarted) {
            follower.followPath(path);
            intake.setPower(isIntakePath ? 1.0 : 0.0);
            stopShooter();

            pathStarted = true;
            delayTimer.resetTimer();
            spinUpTimer.resetTimer();
            fireTimer.resetTimer();
            shotFiredInThisDelay = false;
        }

        if (follower.isBusy()) {
            delayTimer.resetTimer();
            stopShooter();
            shotFiredInThisDelay = false;
        } else {
            if (delaySeconds <= 0.0) {
                pathState++;
                pathStarted = false;
                stopShooter();
                intake.setPower(0);
                return;
            }

            double elapsed = delayTimer.getElapsedTimeSeconds();

            if (!shotFiredInThisDelay) {
                spinShooterWheelRPM(TARGET_WHEEL_RPM);
                boolean atTarget = getCurrentWheelRPM() >= (READY_PERCENT * TARGET_WHEEL_RPM);
                boolean spinUpDone = spinUpTimer.getElapsedTimeSeconds() >= SPINUP_FAILSAFE_SEC;
                boolean delayExpired = elapsed >= delaySeconds;

                if (atTarget || spinUpDone || delayExpired) {
                    intake.setPower(1.0);
                    fireTimer.resetTimer();
                    shotFiredInThisDelay = true;
                }
            } else {
                if (fireTimer.getElapsedTimeSeconds() >= FIRE_DURATION_SEC) {
                    stopShooter();
                    intake.setPower(0);
                    pathState++;
                    pathStarted = false;
                } else {
                    spinShooterWheelRPM(TARGET_WHEEL_RPM);
                }
            }
        }
    }
}
