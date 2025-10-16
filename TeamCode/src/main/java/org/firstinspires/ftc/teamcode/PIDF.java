package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class  PIDF extends OpMode {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    private final double tick_in_degrees = 700 / 180.0;   // todo, find amount of ticks needed the motor we are using is 1150 RPM

private DcMotorEx arm_motor;

    @Override
    public void init() {
        controller= new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FTC Dashboard.getInstance().get)
    }
     // paused video at 5;00 m

    @Override
    public void loop() {

    }
}
