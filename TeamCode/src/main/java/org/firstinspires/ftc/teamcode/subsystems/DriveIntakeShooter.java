package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@TeleOp(name = "TeleOp Final Expanded (Drive + Shooter + Intake + Limelight)", group = "Final")
public class DriveIntakeShooter extends LinearOpMode {

    // ---------- DRIVE CONSTANTS ----------
    private static final double ROTATE_KP = 0.035;        // For Limelight alignment
    private static final double TX_DEADBAND = 1.5;        // Degrees deadband for auto-align
    private static final double MAX_ROTATE_SPEED = 0.6;   // Limit rotation speed
    private static final double DRIVE_SPEED = 0.5;        // Default drive speed

    // ---------- SHOOTER PIDF ----------
    public static final double kP = 20.0;
    public static final double kI = 0.0;
    public static final double kD = 1.0;
    public static final double kF = 14.0;
    public static final double SHOOTER_TARGET_VELOCITY = 3000; // Full speed

    @Override
    public void runOpMode() throws InterruptedException {

        // ---------- HARDWARE MAPPING ----------

        // Drive motors
        DcMotor frontLeftMotor  = hardwareMap.dcMotor.get("fl");
        DcMotor backLeftMotor   = hardwareMap.dcMotor.get("bl");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fr");
        DcMotor backRightMotor  = hardwareMap.dcMotor.get("br");

        // Intake motor
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intake");

        // Shooter motor (DcMotorEx for PIDF)
        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");

        // IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(imuParams);

        // Limelight
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        // ---------- MOTOR SETUP ----------

        // Drive motor directions
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Zero power behavior (brake)
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Intake setup
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Shooter setup
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE); // Reversed now
        shooterMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        telemetry.addLine("READY — G1: Drive + Align | G2: Intake + Shooter");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Button state trackers
        boolean aWasPressed = false; // IMU reset
        boolean shooter1WasPressed = false; // 1s shoot
        boolean shooter2WasPressed = false; // 2s shoot

        ElapsedTime shooterTimer = new ElapsedTime();

        while (opModeIsActive()) {

            // ---------- GAMEPAD 1: DRIVE + LIMELIGHT ALIGN ----------
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Field-centric drive calculations
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX *= 1.1; // Strafing adjustment

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            double frontLeftPower  = (rotY + rotX + rx) / denominator;
            double backLeftPower   = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower  = (rotY + rotX - rx) / denominator;

            // ---------- IMU RESET ----------
            if (gamepad1.a && !aWasPressed) {
                imu.resetYaw();
                aWasPressed = true;
                telemetry.addLine("IMU Reset!");
            } else if (!gamepad1.a) {
                aWasPressed = false;
            }

            // ---------- APRILTAG AUTO-ALIGN (Button B) ----------
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
                    double rotate = 0;
                    if (Math.abs(tx) > TX_DEADBAND) {
                        rotate = -tx * ROTATE_KP;
                        rotate = Range.clip(rotate, -MAX_ROTATE_SPEED, MAX_ROTATE_SPEED);
                    }

                    // Override drive powers for alignment
                    frontLeftPower  = DRIVE_SPEED - rotate;
                    backLeftPower   = DRIVE_SPEED - rotate;
                    frontRightPower = DRIVE_SPEED + rotate;
                    backRightPower  = DRIVE_SPEED + rotate;

                    telemetry.addData("AprilTag Align", "ID=%d | Tx=%.2f", tagId, tx);
                } else {
                    telemetry.addLine("No AprilTag detected");
                }
            }

            // ---------- APPLY DRIVE POWER ----------
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // ---------- GAMEPAD 2: INTAKE & SHOOTER ----------

            // Intake control
            if (gamepad2.left_trigger > 0.1) intakeMotor.setPower(1.0);    // Intake in
            else if (gamepad2.left_bumper) intakeMotor.setPower(-1.0);    // Outtake
            else intakeMotor.setPower(0);                                 // Stop

            // Shooter hold (Right Trigger)
            if (gamepad2.right_trigger > 0.1) {
                shooterMotor.setVelocity(SHOOTER_TARGET_VELOCITY); // Spins while held
            } else {
                shooterMotor.setVelocity(0); // Stops immediately on release
            }

            // Shooter 1s full power (Button A)
            if (gamepad2.a && !shooter1WasPressed) {
                shooter1WasPressed = true;
                shooterTimer.reset();
                while (shooterTimer.seconds() < 1 && opModeIsActive()) {
                    shooterMotor.setPower(1.0);
                }
                shooterMotor.setVelocity(SHOOTER_TARGET_VELOCITY);
            } else if (!gamepad2.a) {
                shooter1WasPressed = false;
            }

            // Shooter 2s full power (Button B)
            if (gamepad2.b && !shooter2WasPressed) {
                shooter2WasPressed = true;
                shooterTimer.reset();
                while (shooterTimer.seconds() < 2 && opModeIsActive()) {
                    shooterMotor.setPower(1.0);
                }
                shooterMotor.setVelocity(SHOOTER_TARGET_VELOCITY);
            } else if (!gamepad2.b) {
                shooter2WasPressed = false;
            }

            // ---------- TELEMETRY ----------
            telemetry.addLine("--- DRIVE ---");
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(botHeading));
            telemetry.addData("FL Power", "%.2f", frontLeftPower);
            telemetry.addData("FR Power", "%.2f", frontRightPower);
            telemetry.addData("BL Power", "%.2f", backLeftPower);
            telemetry.addData("BR Power", "%.2f", backRightPower);

            telemetry.addLine("--- INTAKE ---");
            telemetry.addData("Intake Power", "%.2f", intakeMotor.getPower());

            telemetry.addLine("--- SHOOTER ---");
            telemetry.addData("Shooter Velocity", "%.0f", shooterMotor.getVelocity());
            telemetry.addData("Target Velocity", "%.0f", SHOOTER_TARGET_VELOCITY);

            telemetry.addLine("--- BUTTON INFO ---");
            telemetry.addLine("G1: A=IMU reset | B=AprilTag Align");
            telemetry.addLine("G2: LT=Intake | LB=Outtake | RT=Hold Shooter");
            telemetry.addLine("G2: A=1s Full Shoot | B=2s Full Shoot");

            telemetry.update();
        }
    }
}
