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

    // Additional timers + flags for shooter spin-up and firing
    private Timer spinUpTimer;
    private Timer fireTimer;
    private boolean shotFiredInThisDelay = false;

    private int pathState = 0;
    private boolean pathStarted = false;

    private DcMotor intake;
    private DcMotorEx shooter;

    private final Pose startPose = new Pose(20.719, 122.763, Math.toRadians(323));

    private PathChain[] paths;
    private double[] intakePowers;
    private double[] delays;

    private final double normalSpeed = 1.0;
    private final double intakeSpeed = 0.6;

    // Shooter control settings requested
    private final double TARGET_WHEEL_RPM = 3000.0;       // wheel RPM goal (user requested)
    private final double READY_PERCENT = 0.95;           // 95% threshold
    private final double SPINUP_FAILSAFE_SEC = 3.0;      // after 3s, shoot anyway
    private final double FIRE_DURATION_SEC = 0.5;        // how long to run intake to feed projectile

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

        // === SHOOTER MOTOR ===
        // Uses the name defined in Constants (you previously added this)
        shooter = hardwareMap.get(DcMotorEx.class, Constants.SHOOTER_MOTOR_NAME);
        shooter.setDirection(Constants.SHOOTER_DIRECTION);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Apply PIDF from Constants (assumes you've got those fields in Constants)
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
        // motorRPM = wheelRPM / gearRatio  (because wheelRPM = motorRPM * gearRatio)
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
        intakePowers = new double[10];
        delays = new double[10];

        // ---- PATH 1 ----
        paths[0] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(20.719, 122.763), new Pose(60.086, 83.396)))
                .setLinearHeadingInterpolation(Math.toRadians(323), Math.toRadians(316))
                .build();
        intakePowers[0] = 0.0;
        delays[0] = 1.5;

        // ---- PATH 2 ----
        paths[1] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60.086, 83.396), new Pose(33.151, 83.568)))
                .setTangentHeadingInterpolation()
                .build();
        intakePowers[1] = 0.5;
        delays[1] = 0.0;

        // ---- PATH 3 ----
        paths[2] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(33.151, 83.568), new Pose(60.086, 83.396)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(316))
                .build();
        intakePowers[2] = 0.0;
        delays[2] = 1.5;

        // ---- PATH 4 ----
        paths[3] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60.086, 83.396), new Pose(25.554, 84.086)))
                .setTangentHeadingInterpolation()
                .build();
        intakePowers[3] = 0.5;
        delays[3] = 0.0;

        // ---- PATH 5 ----
        paths[4] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(25.554, 84.086), new Pose(60.086, 83.396)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(316))
                .build();
        intakePowers[4] = 0.0;
        delays[4] = 1.5;

        // ---- PATH 6 ----
        paths[5] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60.086, 83.396), new Pose(59.568, 59.914)))
                .setTangentHeadingInterpolation()
                .build();
        intakePowers[5] = 0.0;
        delays[5] = 0.0;

        // ---- PATH 7 ----
        paths[6] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(59.568, 59.914), new Pose(33.151, 59.741)))
                .setTangentHeadingInterpolation()
                .build();
        intakePowers[6] = 0.5;
        delays[6] = 0.0;

        // ---- PATH 8 ----
        paths[7] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(33.151, 59.741), new Pose(60.086, 83.396)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(316))
                .build();
        intakePowers[7] = 0.0;
        delays[7] = 1.5;

        // ---- PATH 9 ----
        paths[8] = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(60.086, 83.396),
                        new Pose(69.065, 59.223),
                        new Pose(27.799, 59.741)))
                .setTangentHeadingInterpolation()
                .build();
        intakePowers[8] = 0.5;
        delays[8] = 0.0;

        // ---- PATH 10 ----
        paths[9] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(27.799, 59.741), new Pose(60.086, 83.223)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(316))
                .build();
        intakePowers[9] = 0.0;
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
            double intakePower = intakePowers[pathState];
            double delaySec = delays[pathState];
            double speed = (intakePower > 0) ? intakeSpeed : normalSpeed;

            runPath(paths[pathState], intakePower, delaySec, speed);
        } else {
            intake.setPower(0);
            stopShooter();
            telemetry.addData("Status", "Auto Complete");
        }

        // Telemetry helpful for tuning
        telemetry.addData("Path", pathState + 1);
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.addData("Shooter Wheel RPM", String.format("%.1f", getCurrentWheelRPM()));
        telemetry.update();
    }

    private void runPath(PathChain path, double intakePower, double delaySeconds, double speed) {

        follower.setMaxPower(speed);

        // Start path
        if (!pathStarted) {
            follower.followPath(path);
            intake.setPower(intakePower);
            stopShooter();
            pathStarted = true;
            delayTimer.resetTimer();

            // reset spin/firing flags for this path
            spinUpTimer.resetTimer();
            fireTimer.resetTimer();
            shotFiredInThisDelay = false;
        }

        // Robot still moving
        if (follower.isBusy()) {
            // If robot is still moving, reset delay and ensure shooter is not spinning
            delayTimer.resetTimer();
            stopShooter();
            shotFiredInThisDelay = false;
        }
        else {
            // Robot finished path → enter delay
            if (delaySeconds <= 0.0) {
                // No delay → immediately advance
                pathState++;
                pathStarted = false;
                stopShooter();
                intake.setPower(0);
                return;
            }

            // We are inside the delay time window
            double elapsed = delayTimer.getElapsedTimeSeconds();

            if (!shotFiredInThisDelay) {
                // 1) Spin shooter continuously during the delay
                spinShooterWheelRPM(TARGET_WHEEL_RPM);

                // 2) Check readiness: either RPM reached OR spinUp timeout reached OR overall delay expired
                double currentWheelRpm = getCurrentWheelRPM();
                boolean atTarget = currentWheelRpm >= (READY_PERCENT * TARGET_WHEEL_RPM);
                boolean spinUpTimeout = spinUpTimer.getElapsedTimeSeconds() >= SPINUP_FAILSAFE_SEC;
                boolean delayExpired = elapsed >= delaySeconds;

                if (atTarget || spinUpTimeout || delayExpired) {
                    // Start the firing action: run intake motor to feed projectile
                    intake.setPower(intakeSpeed); // feed at intakeSpeed
                    fireTimer.resetTimer();
                    shotFiredInThisDelay = true; // mark that firing started (we'll finish firing in next branch)
                }
            }
            else {
                // We already started firing; wait for firing duration to finish then stop and advance
                if (fireTimer.getElapsedTimeSeconds() >= FIRE_DURATION_SEC) {
                    // stop shooter + intake and proceed to next path
                    stopShooter();
                    intake.setPower(0);
                    pathState++;
                    pathStarted = false;
                } else {
                    // keep shooter spinning and intake running during the firing window
                    spinShooterWheelRPM(TARGET_WHEEL_RPM);
                }
            }
        }
    }
}
