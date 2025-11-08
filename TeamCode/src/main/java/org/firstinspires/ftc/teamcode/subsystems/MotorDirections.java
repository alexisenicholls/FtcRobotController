package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Odometry Telemetry", group = "Telemetry")
public class MotorDirections extends LinearOpMode {

    // Declare odometry pods
    private DcMotor leftPod;
    private DcMotor rightPod;
    private DcMotor strafePod;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize odometry pods
        leftPod = hardwareMap.get(DcMotor.class, "br");    // left Y pod
        rightPod = hardwareMap.get(DcMotor.class, "bl");   // right Y pod
        strafePod = hardwareMap.get(DcMotor.class, "fr");  // strafe X pod

        // Set pods to run without encoders since we're only reading encoder values
        leftPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        strafePod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Odometry Telemetry Initialized");
        telemetry.update();

        // Wait for start
        waitForStart();

        while (opModeIsActive()) {

            // Read encoder values
            int leftEncoder = leftPod.getCurrentPosition();
            int rightEncoder = rightPod.getCurrentPosition();
            int strafeEncoder = strafePod.getCurrentPosition();

            // Display encoder values on telemetry
            telemetry.addData("Left Pod (Y)", leftEncoder);
            telemetry.addData("Right Pod (Y)", rightEncoder);
            telemetry.addData("Strafe Pod (X)", strafeEncoder);
            telemetry.update();

            // Small delay to prevent spamming telemetry
            sleep(50);
        }
    }
}
