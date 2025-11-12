package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class DriveIntakeShooter extends LinearOpMode {

    // Motor power constants
    private static final double DRIVE_POWER_MULTIPLIER = 1.0;
    private static final double INTAKE_POWER = 1.0;
    private static final double SHOOTER_POWER = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Drive motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fl");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bl");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fr");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("br");

        // Additional motors
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intake");
        DcMotor shooterMotor = hardwareMap.dcMotor.get("shooter");

        // Set directions
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Brake mode
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU initialization
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        // Reset IMU to 0 heading
        telemetry.addLine("Initializing IMU...");
        telemetry.update();
        sleep(500);
        imu.resetYaw();
        telemetry.addLine("IMU Reset — Ready to Start");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Drive inputs
            double y = -gamepad1.left_stick_y; // Forward/back
            double x = gamepad1.left_stick_x;  // Strafe
            double rx = gamepad1.right_stick_x; // Rotate

            // Manual IMU reset
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Field-centric calculation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX *= 1.1; // Correct strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator * DRIVE_POWER_MULTIPLIER;
            double backLeftPower = (rotY - rotX + rx) / denominator * DRIVE_POWER_MULTIPLIER;
            double frontRightPower = (rotY - rotX - rx) / denominator * DRIVE_POWER_MULTIPLIER;
            double backRightPower = (rotY + rotX - rx) / denominator * DRIVE_POWER_MULTIPLIER;

            // Set drive motor powers
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Intake control
            if (gamepad1.x) {
                intakeMotor.setPower(INTAKE_POWER);
            } else {
                intakeMotor.setPower(0);
            }

            // Shooter control
            if (gamepad1.y) {
                shooterMotor.setPower(SHOOTER_POWER);
            } else {
                shooterMotor.setPower(0);
            }

            // Telemetry
            telemetry.addData("Heading (deg)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}
