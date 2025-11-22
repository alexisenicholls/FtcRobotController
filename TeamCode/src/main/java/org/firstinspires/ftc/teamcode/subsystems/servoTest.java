package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Controller Example", group = "Examples")
public class servoTest extends LinearOpMode {

    private Servo myServo;
    private double servoPosition = 0.5; // start at middle position

    @Override
    public void runOpMode() {

        // Initialize the servo
        myServo = hardwareMap.get(Servo.class, "my_servo");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Move servo using gamepad
            // Increment position
            if (gamepad1.dpad_up) {
                servoPosition += 0.01;
            }
            // Decrement position
            if (gamepad1.dpad_down) {
                servoPosition -= 0.01;
            }

            // Clip the value to stay between 0 and 1
            servoPosition = Math.max(0, Math.min(1, servoPosition));

            // Set the servo position
            myServo.setPosition(servoPosition);

            // Send the current servo position to telemetry
            telemetry.addData("Servo Position", servoPosition);
            telemetry.update();

            // Small delay to prevent too fast changes
            sleep(50);
        }
    }
}
