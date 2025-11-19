package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Shooter Test Mode", group = "Test")
public class ShooterTest extends LinearOpMode {

    // Declare Hardware Members
    private DcMotor shooterMotor = null;
    private Servo hoodServoLeft = null;
    private Servo hoodServoRight = null;

    // Variables for Servo Control
    // 0.0 is usually fully down, 1.0 is fully up. Adjust depending on your build.
    double hoodPosition = 0.5;

    // CHANGE THIS to match your specific servo's range.
    // REV Smart Servos are usually 180.0 degrees by default.
    // If you used a programmer to set them to 270 mode, change this to 270.0.
    final double SERVO_MAX_ANGLE = 180.0;

    // Speed at which the servo moves when you hold the button
    final double SERVO_SPEED = 0.005;

    @Override
    public void runOpMode() {
        // ------------------------------------------------------------------
        // HARDWARE MAPPING
        // These names must match what you type in the Driver Station config
        // ------------------------------------------------------------------
        shooterMotor  = hardwareMap.get(DcMotor.class, "shooter_motor");
        hoodServoLeft = hardwareMap.get(Servo.class, "left_servo");
        hoodServoRight = hardwareMap.get(Servo.class, "right_servo");

        // MOTOR SETUP
        // Run without encoders is usually fine for a simple shooter flywheel,
        // but using RUN_USING_ENCODER keeps speed consistent if battery drops.
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Check your motor direction. If it spins the wrong way, change to REVERSE
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);

        // SERVO SETUP
        // Hoods usually have mirrored servos. We reverse one so they move together.
        // If your servos fight each other, remove or change this reverse line.
        hoodServoLeft.setDirection(Servo.Direction.FORWARD);
        hoodServoRight.setDirection(Servo.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to Shoot");
        telemetry.update();

        // Wait for the game driver to press PLAY
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // -----------------------
            // SHOOTER CONTROL
            // -----------------------
            // Use the Right Trigger for variable speed control.
            // Pressing it halfway spins motor at 50%, fully is 100%.
            double shootPower = gamepad1.right_trigger;
            shooterMotor.setPower(shootPower);

            // -----------------------
            // HOOD SERVO CONTROL
            // -----------------------
            // Press D-Pad UP to raise angle, DOWN to lower angle
            if (gamepad1.dpad_up) {
                hoodPosition += SERVO_SPEED;
            } else if (gamepad1.dpad_down) {
                hoodPosition -= SERVO_SPEED;
            }

            // "Clip" the position so it never asks the servo to go beyond 0.0 or 1.0
            // This prevents mechanical binding or code errors.
            hoodPosition = Range.clip(hoodPosition, 0.0, 1.0);

            // Set the physical servo positions
            hoodServoLeft.setPosition(hoodPosition);
            hoodServoRight.setPosition(hoodPosition);

            // -----------------------
            // TELEMETRY / DATA
            // -----------------------Z
            // Calculate angle based on the Max Range variable defined at top
            double currentAngle = hoodPosition * SERVO_MAX_ANGLE;

            telemetry.addData("Shooter Power", "%.2f", shootPower);
            telemetry.addData("Hood Position (0-1)", "%.3f", hoodPosition);
            telemetry.addData("Servo Angle", "%.1f deg", currentAngle);
            telemetry.addData("Max Range Set", "%.0f deg", SERVO_MAX_ANGLE);

            // Sim                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            ple visual helper
            if(shootPower > 0) {
                telemetry.addData("Status", "FIRING!");
            } else {
                telemetry.addData("Status", "Standby");
            }

            telemetry.update();
        }
    }
}