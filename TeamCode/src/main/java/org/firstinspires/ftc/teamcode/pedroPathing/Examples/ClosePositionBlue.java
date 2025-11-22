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

@Autonomous(name = "ClosePositionBlue", group = "Examples")
public class ClosePositionBlue extends OpMode {

    private Follower follower;
    private Timer timer;

    private DcMotor intake;
    private DcMotorEx shooter;
    private DcMotorEx indexer;
    private Servo servo;

    private final Pose startPose = new Pose(20.374, 122.590, Math.toRadians(323));

    private PathChain[] paths;
    private boolean[] shooterPaths;

    private final double normalSpeed = 1.0;
    private final double intakePathSpeed = 0.6;

    // Timing constants
    private final double PRE_SPIN_TIME = 2.0;  // shooter pre-spin
    private final double BALL_INTAKE_TIME = 1.0;  // intake + servo down
    private final double SERVO_UP_TIME = 1.0;     // servo up + indexer on + intake extra
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
        telemetry.update();
    }

    private void buildPaths() {
        paths = new PathChain[9];
        shooterPaths = new boolean[9];

        // Shooting paths: 0, 2, 5, 8

        // Path 0
        paths[0] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(20.374, 122.590), new Pose(60.086, 83.396)))
                .setLinearHeadingInterpolation(Math.toRadians(323), Math.toRadians(316))
                .build();
        shooterPaths[0] = true;

        // Path 1
        paths[1] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60.086, 83.396), new Pose(25.899, 83.396)))
                .setTangentHeadingInterpolation()
                .build();
        shooterPaths[1] = false;

        // Path 2
        paths[2] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(25.899, 83.396), new Pose(59.914, 83.396)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(316))
                .build();
        shooterPaths[2] = true;

        // Path 3
        paths[3] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(59.914, 83.396), new Pose(59.741, 59.223)))
                .setTangentHeadingInterpolation()
                .build();
        shooterPaths[3] = false;

        // Path 4
        paths[4] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(59.741, 59.223), new Pose(26.935, 59.223)))
                .setTangentHeadingInterpolation()
                .build();
        shooterPaths[4] = false;

        // Path 5
        paths[5] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(26.935, 59.223), new Pose(59.914, 83.396)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(312))
                .build();
        shooterPaths[5] = true;

        // Path 6
        paths[6] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(59.914, 83.396), new Pose(59.396, 34.532)))
                .setTangentHeadingInterpolation()
                .build();
        shooterPaths[6] = false;

        // Path 7
        paths[7] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(59.396, 34.532), new Pose(27.453, 34.705)))
                .setTangentHeadingInterpolation()
                .build();
        shooterPaths[7] = false;

        // Path 8
        paths[8] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(27.453, 34.705), new Pose(60.086, 83.396)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(310))
                .build();
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

        if (pathState >= paths.length) {
            shooter.setPower(0);
            indexer.setPower(0);
            intake.setPower(0);
            servo.setPosition(SERVO_BOTTOM);
            telemetry.addData("Status", "Auto Complete");
            return;
        }

        boolean isShooting = shooterPaths[pathState];

        if (!pathStarted) {
            double speed = (isShooting) ? normalSpeed : intakePathSpeed;
            follower.setMaxPower(speed);
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

        // Wait for path to finish
        if (follower.isBusy()) {
            if (!shootingPhase && isShooting) shooter.setPower(1.0); // pre-spin
            return;
        }

        // Shooting phase
        if (isShooting) shootingLogic();
        else advancePath();

        // Telemetry
        telemetry.addData("Path", pathState + 1);
        telemetry.addData("Ball Cycle", ballCycle);
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.addData("Shooter Power", shooter.getPower());
        telemetry.addData("Indexer Power", indexer.getPower());
        telemetry.addData("Servo Position", servo.getPosition());
        telemetry.update();
    }

    private void shootingLogic() {
        int maxBalls = (pathState == 0) ? 2 : 3; // first shooting path = 2 balls

        if (!shootingPhase) {
            shootingPhase = true;
            shooter.setPower(1.0);
            phaseTimer = 0;
            timer.resetTimer();
        }

        phaseTimer = timer.getElapsedTimeSeconds();
        double cycleTime = BALL_INTAKE_TIME + SERVO_UP_TIME + RESET_TIME;
        double currentBallTime = phaseTimer - (ballCycle * cycleTime);

        // Step 1 + early Step 2: intake ON, servo DOWN
        if (currentBallTime < BALL_INTAKE_TIME + 0.2) {
            intake.setPower(1.0);
            indexer.setPower(0);
            servo.setPosition(SERVO_BOTTOM);
            return;
        }

        // Step 2: Servo UP, indexer ON, intake OFF
        if (currentBallTime < BALL_INTAKE_TIME + SERVO_UP_TIME) {
            intake.setPower(0);
            indexer.setPower(-1.0);
            servo.setPosition(SERVO_TOP);
            return;
        }

        // Step 3: Reset servo DOWN
        if (currentBallTime < cycleTime) {
            intake.setPower(0);
            indexer.setPower(0);
            servo.setPosition(SERVO_BOTTOM);
            return;
        }

        // Ball cycle complete
        ballCycle++;
        if (ballCycle >= maxBalls) {
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
