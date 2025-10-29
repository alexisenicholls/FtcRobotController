package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "PIDF Arm Control")
public class PIDF extends OpMode {

    private PIDController controller;
    private DcMotorEx arm_motor;

    // PIDF constants (tune in dashboard)
    public static double p = 0.01, i = 0, d = 0;
    public static double f = 0;

    // Target position in encoder ticks
    public static int target = 0;

    // Conversion: 1 tick = X degrees for your motor/gear setup
    private final double ticks_per_degree = 1150.0 / 360.0; // Example for 1150 ticks/rev motor

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor = hardwareMap.get(DcMotorEx.class, "arm_motor0");
        arm_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        // Update PID values from dashboard
        controller.setPID(p, i, d);

        // Get current motor position
        int armPos = arm_motor.getCurrentPosition();

        // PID calculation
        double pidOutput = controller.calculate(armPos, target);

        // Feedforward calculation
        double ffOutput = f * target;

        // Total motor power
        double power = pidOutput + ffOutput;

        // Clamp motor power to [-1, 1]
        power = Math.max(-1, Math.min(1, power));

        // Set motor power
        arm_motor.setPower(power);

        // Telemetry
        telemetry.addData("Arm Target", target);
        telemetry.addData("Arm Position", armPos);
        telemetry.addData("PID Output", pidOutput);
        telemetry.addData("FF Output", ffOutput);
        telemetry.addData("Total Power", power);
        telemetry.update();
    }
}
