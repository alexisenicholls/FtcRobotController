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

    private final double tick_in_degrees = 1 / 180.0;   // todo, find amount of ticks needed the motor we are using is 1150 RPM

private DcMotorEx arm_motor;

    @Override
    public void init() {
        controller= new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor = hardwareMap.get(DcMotorEx.class, deviceName: "arm_motor0");
    }



    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int armPos = arm_motor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        /* 6:18, this is where he makes some
        caluations using the info we had to get about our motor.
         */

    }
}
