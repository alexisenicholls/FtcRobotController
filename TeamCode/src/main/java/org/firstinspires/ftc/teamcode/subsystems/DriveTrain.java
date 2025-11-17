package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@TeleOp
public class DriveTrain extends LinearOpMode {

    // === Constants for AprilTag alignment ===
    private static final double ROTATE_KP = 0.035;
    private static final double TX_DEADBAND = 1.5;
    private static final double MAX_ROTATE_SPEED = 0.6;
    private static final double DRIVE_SPEED = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        // ---------- DRIVE MOTORS ----------
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fl");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bl");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fr");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("br");

        // ---------- INTAKE MOTOR ----------
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Motor directions
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Brake behavior
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ---------- IMU ----------
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        // ---------- LIMELIGHT ----------
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Initializing...");
        telemetry.update();
        sleep(500);
        imu.resetYaw();
        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        boolean aWasPressed = false;

        while (opModeIsActive()) {

            // --- Default driving controls ---
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX *= 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            // --- IMU reset (A button) ---
            if (gamepad1.a && !aWasPressed) {
                imu.resetYaw();
                telemetry.addLine("IMU Reset!");
                aWasPressed = true;
            } else if (!gamepad1.a) {
                aWasPressed = false;
            }

            // --- AprilTag Alignment (B button) ---
            if (gamepad1.b) {
                LLResult result = limelight.getLatestResult();
                boolean tagVisible = false;
                double tx = 0;
                int tagId = -1;

                if (result != null && result.isValid()) {
                    List<FiducialResult> fiducials = result.getFiducialResults();
                    if (!fiducials.isEmpty()) {
                        tagVisible = true;
                        tx = fiducials.get(0).getTargetXDegrees();
                        tagId = fiducials.get(0).getFiducialId();
                    }
                }

                if (tagVisible) {
                    double rotate = 0.0;

                    if (Math.abs(tx) > TX_DEADBAND) {
                        rotate = -tx * ROTATE_KP;
                        rotate = Math.max(-MAX_ROTATE_SPEED, Math.min(MAX_ROTATE_SPEED, rotate));
                    }

                    frontLeftPower = DRIVE_SPEED - rotate;
                    backLeftPower = DRIVE_SPEED - rotate;
                    frontRightPower = DRIVE_SPEED + rotate;
                    backRightPower = DRIVE_SPEED + rotate;

                    telemetry.addData("AprilTag", "ID: %d   Tx: %.2f", tagId, tx);
                    telemetry.addLine("Aligning...");
                } else {
                    telemetry.addLine("No AprilTag detected");
                }
            }

            // ---------- INTAKE CONTROL (X BUTTON) ----------
            // X = spin backwards
            if (gamepad1.x) {
                intakeMotor.setPower(-1.0);
            } else {
                intakeMotor.setPower(0);
            }

            // --- Apply motor power ---
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // --- Telemetry ---
            telemetry.addData("Heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addLine("A = Reset IMU");
            telemetry.addLine("Hold B = AprilTag Align");
            telemetry.addLine("X = Intake Backwards");
            telemetry.update();
        }
    }
}
