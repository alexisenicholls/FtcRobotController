package org.firstinspires.ftc.teamcode.pedroPathing.Examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Far Position Red", group = "Examples")
public class FarPositionRed extends LinearOpMode {

    private Follower follower;
    private ElapsedTime timer = new ElapsedTime();
    private Paths paths;

    // Intake motor
    private DcMotor intake;

    // LIST OF DELAYS
    private static final int[] ONE_SECOND = {1, 4, 7, 10, 13};
    private static final int[] HALF_SECOND = {2, 5, 8, 11};

    // Paths where intake should run
    private static final int[] INTAKE_PATHS = {3, 6, 9, 12};

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);

        // Intake initialization
        intake = hardwareMap.dcMotor.get(Constants.INTAKE_MOTOR_NAME);
        intake.setDirection(Constants.INTAKE_DIRECTION);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower.setStartingPose(new Pose(87.540, 8.115, Math.toRadians(90)));

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // RUN THROUGH ALL 13 PATHS
        for (int i = 1; i <= 13 && opModeIsActive(); i++) {

            PathChain current = paths.getPath(i);
            follower.followPath(current);

            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
                telemetry.addData("Running Path", i);

                // Run intake if path is in INTAKE_PATHS
                intake.setPower(shouldRunIntake(i) ? -1.0 : 0);

                telemetry.addData("Intake Power", intake.getPower());
                telemetry.update();
            }

            // Ensure intake is off after path completes
            intake.setPower(0);

            // Apply proper delay
            double d = getDelay(i);
            if (d > 0) {
                timer.reset();
                while (opModeIsActive() && timer.seconds() < d) {
                    follower.update();
                    telemetry.addData("Delay after Path", i);
                    telemetry.addData("Seconds", "%.2f", timer.seconds());
                    telemetry.update();
                }
            }
        }

        // Stop intake at the very end
        intake.setPower(0);

        telemetry.addData("Status", "All paths completed");
        telemetry.update();
        sleep(300);
    }

    /** Returns delay for path number */
    private double getDelay(int num) {
        for (int x : ONE_SECOND) if (x == num) return 1.0;
        for (int x : HALF_SECOND) if (x == num) return 0.5;
        return 0;
    }

    /** Checks if intake should run on this path */
    private boolean shouldRunIntake(int num) {
        for (int x : INTAKE_PATHS) if (x == num) return true;
        return false;
    }

    // ---------------- PATHS CLASS ----------------
    public static class Paths {
        public PathChain[] list = new PathChain[13];

        public Paths(Follower follower) {
            list[0] = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(87.540, 8.115),
                            new Pose(86.331, 25.727),
                            new Pose(81.842, 13.986)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(251))
                    .build();

            list[1] = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(81.842, 13.986), new Pose(120.518, 13.986)))
                    .setTangentHeadingInterpolation()
                    .build();

            list[2] = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(120.518, 13.986), new Pose(134.331, 14.158)))
                    .setTangentHeadingInterpolation()
                    .build();

            list[3] = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(134.331, 14.158), new Pose(82.014, 13.813)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(251))
                    .build();

            list[4] = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(82.014, 13.813), new Pose(120.518, 13.813)))
                    .setTangentHeadingInterpolation()
                    .build();

            list[5] = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(120.518, 13.813), new Pose(134.158, 13.986)))
                    .setTangentHeadingInterpolation()
                    .build();

            list[6] = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(134.158, 13.986), new Pose(81.842, 13.986)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(251))
                    .build();

            list[7] = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(81.842, 13.986), new Pose(120.691, 13.986)))
                    .setTangentHeadingInterpolation()
                    .build();

            list[8] = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(120.691, 13.986), new Pose(134.331, 13.986)))
                    .setTangentHeadingInterpolation()
                    .build();

            list[9] = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(134.331, 13.986), new Pose(81.669, 13.813)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(251))
                    .build();

            list[10] = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(81.669, 13.813), new Pose(120.518, 13.986)))
                    .setTangentHeadingInterpolation()
                    .build();

            list[11] = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(120.518, 13.986), new Pose(134.504, 13.986)))
                    .setTangentHeadingInterpolation()
                    .build();

            list[12] = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(134.504, 13.986), new Pose(82.014, 13.986)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(251))
                    .build();
        }

        public PathChain getPath(int index) {
            return list[index - 1];
        }
    }
}
