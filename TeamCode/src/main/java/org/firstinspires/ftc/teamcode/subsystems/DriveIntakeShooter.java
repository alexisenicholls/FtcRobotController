package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@TeleOp(name = "Main TeleOp (Drive + Shooter)", group = "Final")
public class DriveIntakeShooter extends LinearOpMode {

    // === Drive Constants ===
    private static final double ROTATE_KP = 0.035;
    private static final double TX_DEADBAND = 1.5;
    private static final double MAX_ROTATE_SPEED = 0.6;
    private static final double DRIVE_SPEED = 0.5;

    // === Shooter Constants ===
    // 0.0 is down, 1.0 is up. Adjust to fit your build.
    double hoodPosition = 0.5;
    // REV Smart Servos = 180.0 default. Change if using goBILDA (300) or Axon.
    final double SERVO_MAX_ANGLE = 180.0;
    final double SERVO_SPEED = 0.005;

    @Override
    public void runOpMode() throws InterruptedException {

        // ------------------------------------------------------------
        // HARDWARE MAPPING
        // ------------------------------------------------------------

        // --- Drive Motors ---
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fl");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bl");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fr");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("br");

        // --- Intake Motor ---
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intake");

        // --- Shooter Hardware ---
        DcMotor shooterMotor = hardwareMap.get(DcMotor.class, "shooter_motor");
        Servo hoodServoLeft = hardwareMap.get(Servo.class, "left_servo");
        Servo hoodServoRight = hardwareMap.get(Servo.class, "right_servo");

        // ------------------------------------------------------------
        // HARDWARE SETUP
        // ------------------------------------------------------------

        // Drive Motor Directions
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Brake Behavior
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Intake Setup
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Shooter Setup
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD); // Check this!

        // Hood Servos (Mirrored)
        hoodServoLeft.setDirection(Servo.Direction.FORWARD);
        hoodServoRight.setDirection(Servo.Direction.REVERSE);

        // IMU Setup
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);

        // Limelight Setup
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Initializing...");
        telemetry.update();
        sleep(500);
        imu.resetYaw();
        telemetry.addLine("Ready! G1: Drive | G2: Shooter/Intake");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        boolean aWasPressed = false;

        while (opModeIsActive()) {

            // ============================================================
            // GAMEPAD 1: DRIVE TRAIN & ALIGNMENT
            // ============================================================

            // --- Field Centric Drive ---
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX *= 1.1; // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            double frontLeftPower  = (rotY + rotX + rx) / denominator;
            double backLeftPower   = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower  = (rotY + rotX - rx) / denominator;

            // --- Reset IMU (Button A) ---
            if (gamepad1.a && !aWasPressed) {
                imu.resetYaw();
                telemetry.addLine("IMU Reset!");
                aWasPressed = true;
            } else if (!gamepad1.a) {
                aWasPressed = false;
            }

            // --- AprilTag Alignment (Button B) ---
            if (gamepad1.b) {
                LLResult result = limelight.getLatestResult();
                boolean tagVisible = false;
                double tx = 0;

                if (result != null && result.isValid()) {
                    List<FiducialResult> fiducials = result.getFiducialResults();
                    if (!fiducials.isEmpty()) {
                        tagVisible = true;
                        tx = fiducials.get(0).getTargetXDegrees();
                    }
                }

                if (tagVisible) {
                    double rotate = 0;
                    if (Math.abs(tx) > TX_DEADBAND) {
                        rotate = -tx * ROTATE_KP;
                        rotate = Math.max(-MAX_ROTATE_SPEED, Math.min(MAX_ROTATE_SPEED, rotate));
                    }
                    // Override normal drive powers for alignment
                    frontLeftPower  = DRIVE_SPEED - rotate;
                    backLeftPower   = DRIVE_SPEED - rotate;
                    frontRightPower = DRIVE_SPEED + rotate;
                    backRightPower  = DRIVE_SPEED + rotate;
                    telemetry.addData("Auto-Align", "Tx: %.2f", tx);
                }
            }

            // Apply Powers to Drive Motors
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // ============================================================
            // GAMEPAD 2: SHOOTER, HOOD, & INTAKE
            // ============================================================

            // --- Intake Control (Left Trigger / Bumper) ---
            // Left Trigger = SUCK IN (Positive)
            // Left Bumper  = SPIT OUT (Negative)
            if (gamepad2.left_trigger > 0.1) {
                intakeMotor.setPower(1.0);
            } else if (gamepad2.left_bumper) {
                intakeMotor.setPower(-1.0);
            } else {
                intakeMotor.setPower(0);
            }

            // --- Shooter Control (Right Trigger) ---
            double shootPower = gamepad2.right_trigger;
            shooterMotor.setPower(shootPower);

            // --- Hood Angle Control (D-Pad Up/Down) ---
            if (gamepad2.dpad_up) {
                hoodPosition += SERVO_SPEED;
            } else if (gamepad2.dpad_down) {
                hoodPosition -= SERVO_SPEED;
            }
            hoodPosition = Range.clip(hoodPosition, 0.0, 1.0);

            hoodServoLeft.setPosition(hoodPosition);
            hoodServoRight.setPosition(hoodPosition);

            // ============================================================
            // TELEMETRY
            // ============================================================

            // Calculate angle for display
            double currentAngle = hoodPosition * SERVO_MAX_ANGLE;

            telemetry.addData("--- DRIVE ---", "");
            telemetry.addData("Heading", "%.1f deg", Math.toDegrees(botHeading));

            telemetry.addData("--- SHOOTER ---", "");
            telemetry.addData("Shooter Pwr", "%.2f", shootPower);
            telemetry.addData("Hood Pos", "%.3f", hoodPosition);
            telemetry.addData("Hood Angle", "%.1f deg", currentAngle);

            telemetry.update();
        }
    }
}