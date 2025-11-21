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

@Autonomous(name = "ClosePositionBlue", group = "Examples")
public class ClosePositionBlue extends OpMode {

    private Follower follower;
    private Timer delayTimer;

    private Timer spinUpTimer;
    private Timer fireTimer;
    private boolean shotFiredInThisDelay = false;

    private int pathState = 0;
    private boolean pathStarted = false;

    private DcMotor intake;
    private DcMotorEx shooter;

    private final Pose startPose = new Pose(20.719, 122.763, Math.toRadians(323));

    private PathChain[] paths;
    private boolean[] intakePaths; // true if intake should run
    private double[] delays;

    private final double normalSpeed = 1.0;
    private final double intakePathSpeed = 0.6; // 50% speed on intake paths

    // Shooter control settings
    private final double TARGET_WHEEL_RPM = 3000.0;       // wheel RPM goal
    private final double READY_PERCENT = 0.95;           // 95% threshold
    private final double SPINUP_FAILSAFE_SEC = 3.0;      // after 3s, shoot anyway
    private final double FIRE_DURATION_SEC = 0.75;        // how long to run intake to feed projectile

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        delayTimer = new Timer();
        spinUpTimer = new Timer();
        fireTimer = new Timer();

        // Intake
        intake = hardwareMap.dcMotor.get("intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Shooter
        shooter = hardwareMap.get(DcMotorEx.class, Constants.SHOOTER_MOTOR_NAME);
        shooter.setDirection(Constants.SHOOTER_DIRECTION);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(
                        Constants.SHOOTER_kP,
                        Constants.SHOOTER_kI,
                        Constants.SHOOTER_kD,
                        Constants.SHOOTER_kF
                )
        );

        follower.setStartingPose(startPose);
        buildPaths();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    // Helper: set motor velocity to achieve desired wheel RPM (converts through gear ratio)
    private void spinShooterWheelRPM(double wheelRpm) {
        double motorRpm = wheelRpm / Constants.SHOOTER_GEAR_RATIO;
        double ticksPerRev = shooter.getMotorType().getTicksPerRev();
        double velocityTicksPerSec = (motorRpm / 60.0) * ticksPerRev;  // motor ticks/sec
        shooter.setVelocity(velocityTicksPerSec);
    }

    // Helper: stop shooter (set velocity to 0)
    private void stopShooter() {
        shooter.setVelocity(0);
    }

    // Helper: read current wheel RPM (convert from motor ticks/sec -> motor RPM -> wheel RPM)
    private double getCurrentWheelRPM() {
        double ticksPerSec = shooter.getVelocity(); // motor shaft ticks/sec
        double ticksPerRev = shooter.getMotorType().getTicksPerRev();
        double motorRpm = (ticksPerSec * 60.0) / ticksPerRev;
        double wheelRpm = motorRpm * Constants.SHOOTER_GEAR_RATIO;
        return wheelRpm;
    }

    private void buildPaths() {
        paths = new PathChain[10];
        intakePaths = new boolean[10];
        delays = new double[10];

        // ---- PATH 1 ----
        paths[0] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(20.719, 122.763), new Pose(60.086, 83.396)))
                .setLinearHeadingInterpolation(Math.toRadians(323), Math.toRadians(316))
                .build();
        intakePaths[0] = false;
        delays[0] = 1.5;

        // ---- PATH 2 ----
        paths[1] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60.086, 83.396), new Pose(33.151, 83.568)))
                .setTangentHeadingInterpolation()
                .build();
        intakePaths[1] = true;    // intake path
        delays[1] = 0.0;

        // ---- PATH 3 ----
        paths[2] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(33.151, 83.568), new Pose(60.086, 83.396)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(316))
                .build();
        intakePaths[2] = false;
        delays[2] = 1.5;

        // ---- PATH 4 ----
        paths[3] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60.086, 83.396), new Pose(25.554, 84.086)))
                .setTangentHeadingInterpolation()
                .build();
        intakePaths[3] = true;    // intake path
        delays[3] = 0.0;

        // ---- PATH 5 ----
        paths[4] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(25.554, 84.086), new Pose(60.086, 83.396)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(316))
                .build();
        intakePaths[4] = false;
        delays[4] = 1.5;

        // ---- PATH 6 ----
        paths[5] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60.086, 83.396), new Pose(59.568, 59.914)))
                .setTangentHeadingInterpolation()
                .build();
        intakePaths[5] = true;    // intake path
        delays[5] = 0.0;

        // ---- PATH 7 ----
        paths[6] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(59.568, 59.914), new Pose(33.151, 59.741)))
                .setTangentHeadingInterpolation()
                .build();
        intakePaths[6] = true;    // intake path
        delays[6] = 0.0;

        // ---- PATH 8 ----
        paths[7] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(33.151, 59.741), new Pose(60.086, 83.396)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(316))
                .build();
        intakePaths[7] = false;
        delays[7] = 1.5;

        // ---- PATH 9 ----
        paths[8] = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(60.086, 83.396),
                        new Pose(69.065, 59.223),
                        new Pose(27.799, 59.741)))
                .setTangentHeadingInterpolation()
                .build();
        intakePaths[8] = true;    // intake path
        delays[8] = 0.0;

        // ---- PATH 10 ----
        paths[9] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(27.799, 59.741), new Pose(60.086, 83.223)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(316))
                .build();
        intakePaths[9] = false;
        delays[9] = 1.5;
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

            // dynamic speed: slow on intake paths OR while firing
            double speed = (isIntakePath || shotFiredInThisDelay) ? intakePathSpeed : normalSpeed;

            // pass the computed speed into runPath so follower.setMaxPower() uses it
            runPath(paths[pathState], isIntakePath, delaySec, speed);
        } else {
            intake.setPower(0);
            stopShooter();
            telemetry.addData("Status", "Auto Complete");
        }

        telemetry.addData("Path", pathState + 1);
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.addData("Shooter Wheel RPM", String.format("%.1f", getCurrentWheelRPM()));
        telemetry.update();
    }

    private void runPath(PathChain path, boolean isIntakePath, double delaySeconds, double speed) {

        // set follower max power according to computed speed from loop()
        follower.setMaxPower(speed);

        // Start path if not already started
        if (!pathStarted) {
            follower.followPath(path);

            // Intake: ALWAYS 100% when this path is an intake path
            intake.setPower(isIntakePath ? 1.0 : 0.0);

            // make sure shooter is off while driving
            stopShooter();

            pathStarted = true;
            delayTimer.resetTimer();

            // reset timers/flags for the upcoming delay/shoot sequence
            spinUpTimer.resetTimer();
            fireTimer.resetTimer();
            shotFiredInThisDelay = false;
        }

        // While follower is still moving, keep resetting the delay timer so that delay only counts after movement done
        if (follower.isBusy()) {
            // Reset the delay timer while we're still moving
            delayTimer.resetTimer();

            // keep shooter stopped while moving (you requested this behavior)
            stopShooter();

            // ensure shot flag cleared while moving
            shotFiredInThisDelay = false;
        }
        else {
            // follower finished the path → begin delay / shooting logic
            if (delaySeconds <= 0.0) {
                // No delay configured: advance immediately
                pathState++;
                pathStarted = false;
                stopShooter();
                intake.setPower(0);
                return;
            }

            double elapsed = delayTimer.getElapsedTimeSeconds();

            // If we haven't fired in this delay yet, attempt to get shooter ready then fire
            if (!shotFiredInThisDelay) {
                // Start spinning shooter continuously during the delay window
                spinShooterWheelRPM(TARGET_WHEEL_RPM);

                // Read current RPM, check readiness or spinup failsafe or delay expiration
                double currentWheelRpm = getCurrentWheelRPM();
                boolean atTarget = currentWheelRpm >= (READY_PERCENT * TARGET_WHEEL_RPM);
                boolean spinUpTimeout = spinUpTimer.getElapsedTimeSeconds() >= SPINUP_FAILSAFE_SEC;
                boolean delayExpired = elapsed >= delaySeconds;

                if (atTarget || spinUpTimeout || delayExpired) {
                    // Begin firing: run intake at full power to feed projectile
                    intake.setPower(1.0); // always feed at full power during firing
                    fireTimer.resetTimer();
                    shotFiredInThisDelay = true;
                }
            }
            else {
                // We already started firing; wait for firing duration to complete
                if (fireTimer.getElapsedTimeSeconds() >= FIRE_DURATION_SEC) {
                    // stop shooter + intake and proceed to next path
                    stopShooter();
                    intake.setPower(0);
                    pathState++;
                    pathStarted = false;
                } else {
                    // keep shooter spinning and intake running during firing window
                    spinShooterWheelRPM(TARGET_WHEEL_RPM);
                }
            }
        }
    }
}
