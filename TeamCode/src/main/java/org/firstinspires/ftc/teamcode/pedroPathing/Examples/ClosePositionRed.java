package org.firstinspires.ftc.teamcode.pedroPathing.Examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Auto 13 Paths (Selective Delays)", group = "Examples")
public class ClosePositionRed extends LinearOpMode {

    private Follower follower;
    private ElapsedTime timer = new ElapsedTime();
    private Paths paths;

    // Paths that should have a 1-second delay after finishing
    private static final int[] DELAY_PATHS = {1, 3, 5, 7, 9, 11, 13};

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);

        follower.setStartingPose(new Pose(123.108, 122.245, Math.toRadians(218)));

        telemetry.addData("Status", "Initialized - Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Run through all 13 paths
        for (int i = 1; i <= 13 && opModeIsActive(); i++) {
            PathChain current = paths.getPath(i);
            follower.followPath(current);

            // Wait until this path finishes
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
                telemetry.addData("Running Path", i);
                telemetry.update();
            }

            // Add delay only after specific paths
            if (shouldDelay(i)) {
                timer.reset();
                while (opModeIsActive() && timer.seconds() < 1.0) {
                    follower.update();
                    telemetry.addData("Delay after Path", i);
                    telemetry.addData("Time", "%.2f", timer.seconds());
                    telemetry.update();
                }
            }
        }

        telemetry.addData("Status", "All Paths Done");
        telemetry.update();
        sleep(500);
    }

    /** Checks if a given path number should have a delay */
    private boolean shouldDelay(int pathNum) {
        for (int n : DELAY_PATHS) if (n == pathNum) return true;
        return false;
    }

    // ---------------- PATHS CLASS ----------------
    public static class Paths {
        public PathChain[] list = new PathChain[13];

        public Paths(Follower follower) {
            list[0] = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(123.108, 122.245), new Pose(83.914, 83.741)))
                    .setLinearHeadingInterpolation(Math.toRadians(218), Math.toRadians(226))
                    .build();

            list[1] = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(83.914, 83.741), new Pose(113.094, 83.396)))
                    .setTangentHeadingInterpolation()
                    .build();

            list[2] = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(113.094, 83.396), new Pose(83.914, 83.741)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(226))
                    .build();

            list[3] = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(83.914, 83.741), new Pose(119.655, 83.568)))
                    .setTangentHeadingInterpolation()
                    .build();

            list[4] = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(119.655, 83.568), new Pose(83.741, 83.741)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(226))
                    .build();

            list[5] = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(83.741, 83.741),
                            new Pose(82.360, 58.878),
                            new Pose(114.129, 59.396)))
                    .setTangentHeadingInterpolation()
                    .build();

            list[6] = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(114.129, 59.396), new Pose(83.741, 83.568)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(226))
                    .build();

            list[7] = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(83.741, 83.568), new Pose(120.863, 62.158)))
                    .setTangentHeadingInterpolation()
                    .build();

            list[8] = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(120.863, 62.158), new Pose(83.741, 83.741)))
                    .setLinearHeadingInterpolation(Math.toRadians(330), Math.toRadians(226))
                    .build();

            list[9] = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(83.741, 83.741),
                            new Pose(84.259, 36.259),
                            new Pose(114.129, 35.568)))
                    .setTangentHeadingInterpolation()
                    .build();

            list[10] = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(114.129, 35.568), new Pose(83.223, 83.568)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(226))
                    .build();

            list[11] = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(83.223, 83.568), new Pose(122.245, 39.540)))
                    .setTangentHeadingInterpolation()
                    .build();

            list[12] = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(122.245, 39.540), new Pose(83.396, 83.396)))
                    .setLinearHeadingInterpolation(Math.toRadians(310), Math.toRadians(226))
                    .build();
        }

        public PathChain getPath(int index) {
            return list[index - 1];
        }
    }
}
