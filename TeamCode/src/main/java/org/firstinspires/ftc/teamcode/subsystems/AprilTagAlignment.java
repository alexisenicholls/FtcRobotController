package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class AprilTagAlignment extends LinearOpMode {

    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private IMU imu;
    private Limelight3A limelight;

    // Tuning parameters
    private final double STRAFE_KP = 0.01;
    private final double DISTANCE_KP = 0.02;
    private final double TARGET_DISTANCE = 100;
    private final double DEADZONE_X = 1.5;
    private final double DEADZONE_Y = 2.0;
    private final double MAX_SPEED = 0.6;

    // Output flags
    public boolean isCentered = false;
    public boolean isAtDistance = false;
    public boolean isFullyAligned = false; // new flag

    @Override
    public void runOpMode() {

        // Motor initialization
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // IMU setup
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        )));

        // Limelight setup
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            LLResult llResult = limelight.getLatestResult();
            boolean hasTarget = llResult != null && llResult.isValid();

            // Reset flags
            isCentered = false;
            isAtDistance = false;
            isFullyAligned = false;

            if (hasTarget) {
                double tx = llResult.getTx();
                double ta = llResult.getTa();
                double distance = getDistanceFromTag(ta);

                // X-axis strafing with deadzone
                if (Math.abs(tx) > DEADZONE_X) {
                    x = clamp(tx * STRAFE_KP, -MAX_SPEED, MAX_SPEED);
                } else {
                    x = 0;
                    isCentered = true;
                }

                // Forward/backward control with deadzone
                double yError = TARGET_DISTANCE - distance;
                if (Math.abs(yError) > DEADZONE_Y) {
                    y = clamp(yError * DISTANCE_KP, -MAX_SPEED, MAX_SPEED);
                } else {
                    y = 0;
                    isAtDistance = true;
                }

                // Optional: reduce rotation while aligning
                rx *= 0.3;

                // Fully aligned if both conditions met
                isFullyAligned = isCentered && isAtDistance;

                // Stop robot completely if fully aligned
                if (isFullyAligned) {
                    x = 0;
                    y = 0;
                    rx = 0;
                }
            }

            // Field-centric transformation
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX *= 1.1;

            // Normalize powers
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            // Set motor powers
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Telemetry for debugging
            telemetry.addData("Has Target", hasTarget);
            if (hasTarget) {
                telemetry.addData("Tx", llResult.getTx());
                telemetry.addData("Distance", getDistanceFromTag(llResult.getTa()));
                telemetry.addData("Centered?", isCentered);
                telemetry.addData("At Distance?", isAtDistance);
                telemetry.addData("Fully Aligned?", isFullyAligned);
            } else {
                telemetry.addLine("No target detected");
            }
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
