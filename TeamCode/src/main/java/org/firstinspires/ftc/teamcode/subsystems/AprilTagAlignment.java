package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Config
public class AprilTagAlignment extends LinearOpMode {

    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private IMU imu;
    private Limelight3A limelight;
    private FtcDashboard dashboard;

    // Tunable constants (adjustable in FTC Dashboard)
    public static double STRAFE_KP = 0.01;
    public static double DISTANCE_KP = 0.02;
    public static double TARGET_DISTANCE = 100;
    public static double DEADZONE_X = 1.5;
    public static double DEADZONE_Y = 2.0;
    public static double MAX_SPEED = 0.6;

    // Alignment flags
    public boolean isCentered = false;
    public boolean isAtDistance = false;
    public boolean isFullyAligned = false;

    @Override
    public void runOpMode() {

        // Motor setup
        frontLeftMotor = hardwareMap.dcMotor.get("fl");
        backLeftMotor = hardwareMap.dcMotor.get("bl");
        frontRightMotor = hardwareMap.dcMotor.get("fr");
        backRightMotor = hardwareMap.dcMotor.get("br");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // IMU setup
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        // Limelight setup
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        // FTC Dashboard setup
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.startCameraStream(limelight, 0);

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            LLResult llResult = limelight.getLatestResult();
            boolean hasTarget = llResult != null && llResult.isValid();

            isCentered = false;
            isAtDistance = false;
            isFullyAligned = false;

            TelemetryPacket packet = new TelemetryPacket();

            if (hasTarget) {
                Pose3D botPose = llResult.getBotpose();
                double tx = llResult.getTx();
                double ty = llResult.getTy();
                double ta = llResult.getTa();
                double distance = getDistanceFromTag(ta);

                // Strafing control
                if (Math.abs(tx) > DEADZONE_X) {
                    x = clamp(tx * STRAFE_KP, -MAX_SPEED, MAX_SPEED);
                } else {
                    x = 0;
                    isCentered = true;
                }

                // Forward/backward control
                double yError = TARGET_DISTANCE - distance;
                if (Math.abs(yError) > DEADZONE_Y) {
                    y = clamp(yError * DISTANCE_KP, -MAX_SPEED, MAX_SPEED);
                } else {
                    y = 0;
                    isAtDistance = true;
                }

                rx *= 0.3;
                isFullyAligned = isCentered && isAtDistance;
                if (isFullyAligned) {
                    x = 0;
                    y = 0;
                    rx = 0;
                }

                // Bounding box overlay
                double centerX = tx / 29.8;
                double centerY = -ty / 23.8;
                double size = Math.sqrt(ta / 100.0);
                packet.fieldOverlay()
                        .setStroke("#FF0000")
                        .strokeRect(centerX - size / 2.0, centerY - size / 2.0,
                                centerX + size / 2.0, centerY + size / 2.0);

                // Alignment arrow overlay
                double arrowLength = 0.05; // adjust arrow size
                double arrowX = clamp(tx / 60.0, -0.3, 0.3); // left/right proportional
                double arrowY = isAtDistance ? 0 : clamp(yError / 200.0, -0.3, 0.3); // forward/back

                String arrowColor = isFullyAligned ? "#00FF00" : "#FFFF00"; // green if aligned, yellow otherwise
                packet.fieldOverlay()
                        .setStroke(arrowColor)
                        .strokeLine(0, 0, arrowX, arrowY); // arrow from center of field

                // Telemetry for debugging
                telemetry.addData("Tx", tx);
                telemetry.addData("Ty", ty);
                telemetry.addData("Distance", distance);
                telemetry.addData("Centered?", isCentered);
                telemetry.addData("At Distance?", isAtDistance);
                telemetry.addData("Fully Aligned?", isFullyAligned);
                telemetry.addData("Botpose", botPose.toString());

            } else {
                telemetry.addLine("No valid AprilTag detected");
            }

            // Send dashboard overlay
            dashboard.sendTelemetryPacket(packet);

            // Field-centric drive calculation
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX *= 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            frontLeftMotor.setPower((rotY + rotX + rx) / denominator);
            backLeftMotor.setPower((rotY - rotX + rx) / denominator);
            frontRightMotor.setPower((rotY - rotX - rx) / denominator);
            backRightMotor.setPower((rotY + rotX - rx) / denominator);

            // Display tunable constants
            telemetry.addData("STRAFE_KP", STRAFE_KP);
            telemetry.addData("DISTANCE_KP", DISTANCE_KP);
            telemetry.addData("TARGET_DISTANCE", TARGET_DISTANCE);
            telemetry.addData("DEADZONE_X", DEADZONE_X);
            telemetry.addData("DEADZONE_Y", DEADZONE_Y);
            telemetry.addData("MAX_SPEED", MAX_SPEED);
            telemetry.update();
        }
    }

    private double getDistanceFromTag(double ta) {
        double scale = 180.14;
        if (ta <= 0) return Double.POSITIVE_INFINITY;
        return scale / ta;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
