package org.firstinspires.ftc.teamcode.pedroPathing.Examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "ClosePositionRed", group = "Examples")
public class ClosePositionRed extends OpMode {

    private Follower follower;
    private Timer timer;

    private DcMotor intake;
    private DcMotorEx shooter;
    private DcMotor indexer;
    private Servo servo;

    private final Pose startPose = new Pose(123.626, 122.763, Math.toRadians(216));

    private PathChain[] paths;
    private boolean[] shooterPaths;

    private final double normalSpeed = 1.0;
    private final double intakePathSpeed = 0.6;

    // Timing constants
    private final double PRE_SPIN_TIME = 2.0;  // shooter prespin
    private final double BALL_INTAKE_TIME = 1.0;  // intake + servo down
    private final double SERVO_UP_TIME = 1.0;     // servo up + indexer on + intake extra 1s
    private final double RESET_TIME = 0.3;        // reset servo down

    // Servo positions
    private final double SERVO_BOTTOM = 0.7;  // down
    private final double SERVO_TOP = 0.0;     // up

    private int pathState = 0;
    private boolean pathStarted = false;
    private boolean shootingPhase = false;

    private int ballCycle = 0;      // which ball in the cycle
    private double phaseTimer = 0;  // timing inside ball cycle

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        timer = new Timer();

        intake = hardwareMap.dcMotor.get("intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter = hardwareMap.get(DcMotorEx.class, Constants.SHOOTER_MOTOR_NAME);
        shooter.setDirection(Constants.SHOOTER_DIRECTION);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        indexer = hardwareMap.get(DcMotorEx.class, Constants.INDEXER_MOTOR_NAME);
        indexer.setDirection(Constants.INDEXER_DIRECTION);
        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo = hardwareMap.get(Servo.class, "servo");
        servo.setPosition(SERVO_BOTTOM); // start down

        follower.setStartingPose(startPose);
        buildPaths();

        telemetry.addData("Status", "Initialized");
    }

    private void buildPaths() {
        paths = new PathChain[9];
        shooterPaths = new boolean[9];

        // Path 0 is shooting path
        paths[0] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(123.626, 122.763), new Pose(83.914, 83.396)))
                .setLinearHeadingInterpolation(Math.toRadians(216), Math.toRadians(229)).build();
        shooterPaths[0] = true;

        // Path 1 intake
        paths[1] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(83.914, 83.396), new Pose(117.755, 83.396)))
                .setTangentHeadingInterpolation().build();
        shooterPaths[1] = false;

        // Path 2 shooting
        paths[2] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(117.755, 83.396), new Pose(83.914, 83.223)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(229)).build();
        shooterPaths[2] = true;

        // Path 3
        paths[3] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(83.914, 83.223), new Pose(84.604, 58.187)))
                .setTangentHeadingInterpolation().build();
        shooterPaths[3] = false;

        // Path 4 intake
        paths[4] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(84.604, 58.187), new Pose(118.273, 58.532)))
                .setTangentHeadingInterpolation().build();
        shooterPaths[4] = false;

        // Path 5 shooting
        paths[5] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(118.273, 58.532), new Pose(84.086, 83.050)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(229)).build();
        shooterPaths[5] = true;

        // Path 6
        paths[6] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(84.086, 83.050), new Pose(84.259, 33.151)))
                .setTangentHeadingInterpolation().build();
        shooterPaths[6] = false;

        // Path 7 intake
        paths[7] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(84.259, 33.151), new Pose(117.583, 33.669)))
                .setTangentHeadingInterpolation().build();
        shooterPaths[7] = false;

        // Path 8 shooting
        paths[8] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(117.583, 33.669), new Pose(84.259, 83.223)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(229)).build();
        shooterPaths[8] = true;
    }

    @Override
    public void start() {
        pathState = 0;
        pathStarted = false;
        shootingPhase = false;
        ballCycle = 0;
        phaseTimer = 0;
        timer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();

        if (pathState >= paths.length) return;

        boolean isShooting = shooterPaths[pathState];

        if (!pathStarted) {
            follower.setMaxPower(1.0);
            follower.followPath(paths[pathState]);
            pathStarted = true;

            intake.setPower(1.0);
            indexer.setPower(0);
            shooter.setPower(0);
            servo.setPosition(SERVO_BOTTOM);

            timer.resetTimer();
            shootingPhase = false;
            ballCycle = 0;
        }

        // Wait for paths to finish
        if (follower.isBusy()) {

            // Shooter pre-spin handling
            if (!shootingPhase && isShooting) {
                shooter.setPower(1.0);  // pre-spin
            }

            return;
        }

        // PATH IS FINISHED — now do SHOOOTING LOGIC
        if (isShooting) shootingLogic();
        else advancePath();
    }

    private void shootingLogic() {
        int maxBalls = (pathState == 0) ? 2 : 3;  // first path = 2 balls

        if (!shootingPhase) {
            shootingPhase = true;
            shooter.setPower(1.0); // shooter stays on
            phaseTimer = 0;
            timer.resetTimer();
        }

        phaseTimer = timer.getElapsedTimeSeconds();

        double cycleTime = BALL_INTAKE_TIME + SERVO_UP_TIME + RESET_TIME;
        double currentBallTime = phaseTimer - (ballCycle * cycleTime);

        // BALL INTAKE PHASE (0–1s)
        if (currentBallTime < BALL_INTAKE_TIME) {
            intake.setPower(1.0);
            indexer.setPower(0);
            servo.setPosition(SERVO_BOTTOM);
            return;
        }

        // INDEXER ON EARLY + SERVO UP (1–2s)
        if (currentBallTime < BALL_INTAKE_TIME + SERVO_UP_TIME) {
            intake.setPower(1.0); // intake stays on 1 extra second
            indexer.setPower(-1.0);
            servo.setPosition(SERVO_TOP);
            return;
        }

        // RESET PHASE (servo down, intake off)
        if (currentBallTime < cycleTime) {
            intake.setPower(0);
            indexer.setPower(0);
            servo.setPosition(SERVO_BOTTOM);
            return;
        }

        // Ball cycle complete
        ballCycle++;

        if (ballCycle >= maxBalls) {
            // Done shooting
            intake.setPower(0);
            indexer.setPower(0);
            servo.setPosition(SERVO_BOTTOM);
            advancePath();
        }
    }

    private void advancePath() {
        pathState++;
        pathStarted = false;
        shootingPhase = false;
        timer.resetTimer();
    }
}
