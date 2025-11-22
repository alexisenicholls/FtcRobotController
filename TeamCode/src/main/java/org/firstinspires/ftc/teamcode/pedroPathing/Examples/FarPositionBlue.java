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

    @Autonomous(name = "FarPositionBlue", group = "Examples")
    public class FarPositionBlue extends OpMode {

        private Follower follower;
        private Timer delayTimer;

        private int pathState = 0;
        private boolean pathStarted = false;
        private boolean shootingPhase = false;

        private DcMotorEx shooter;

        // START POSE set exactly to the start of Path1 (no rounding)
        private final Pose startPose = new Pose(56.000, 8.000, Math.toRadians(90));

        private PathChain[] paths;
        private boolean[] shooterPaths;

        // Robot speed (50%)
        private final double robotSpeed = 0.5;

        // Shooter timing
        private final double PRE_SPIN_TIME = 0;  // pre-spin for 5 seconds
        private final double SHOOT_TIME = 7.0;     // shoot for 3 seconds

        @Override
        public void init() {
            follower = Constants.createFollower(hardwareMap);
            delayTimer = new Timer();

            shooter = hardwareMap.get(DcMotorEx.class, Constants.SHOOTER_MOTOR_NAME);
            shooter.setDirection(Constants.SHOOTER_DIRECTION);
            shooter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Ensure follower starts exactly at the path's first pose
            follower.setStartingPose(startPose);

            // Build paths exactly as you provided
            buildPaths();

            telemetry.addData("Status", "Initialized");
            telemetry.update();
        }

        private void buildPaths() {
            paths = new PathChain[2];
            shooterPaths = new boolean[2];

            // Path1 (exact coordinates you gave)
            paths[0] = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(56.000, 8.000), new Pose(55.770, 16.748)))
                    .setTangentHeadingInterpolation()
                    .build();
            shooterPaths[0] = false;

            // Path2 (exact coordinates you gave)
            paths[1] = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(55.770, 16.748), new Pose(59.741, 16.921)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(28))
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
                    // Robot always runs at 50%
                    follower.setMaxPower(robotSpeed);

                    // Follow current path
                    follower.followPath(paths[pathState]);

                    // Ensure shooter is off until pre-spin
                    shooter.setPower(0.0);

                    pathStarted = true;
                    delayTimer.resetTimer(); // start timing from when path begins (so elapsed counts after path ends)
                }

                // When path finishes...
                if (!follower.isBusy()) {
                    double elapsed = delayTimer.getElapsedTimeSeconds();

                    if (isShooterPath) {
                        // Pre-spin then shoot
                        if (!shootingPhase) {
                            // Start pre-spin
                            shooter.setPower(1.0);
                            if (elapsed >= PRE_SPIN_TIME) {
                                // move to shooting phase
                                shootingPhase = true;
                                delayTimer.resetTimer(); // reset to time shooting window
                            }
                        } else {
                            // shootingPhase == true -> we're shooting for SHOOT_TIME seconds
                            if (elapsed < SHOOT_TIME) {
                                shooter.setPower(1.0);
                            } else {
                                // finished shooting, stop and advance
                                shooter.setPower(0.0);
                                pathState++;
                                pathStarted = false;
                                shootingPhase = false;
                                delayTimer.resetTimer();
                            }
                        }
                    } else {
                        // Not a shooter path: advance immediately
                        pathState++;
                        pathStarted = false;
                        delayTimer.resetTimer();
                    }
                }
            } else {
                // Auto complete: ensure shooter off
                shooter.setPower(0.0);
                telemetry.addData("Status", "Complete");
            }

            telemetry.addData("Active Path", Math.min(pathState + 1, paths.length));
            telemetry.addData("Follower Busy", follower.isBusy());
            telemetry.addData("Shooter Power", shooter.getPower());
            telemetry.update();
        }
    }
