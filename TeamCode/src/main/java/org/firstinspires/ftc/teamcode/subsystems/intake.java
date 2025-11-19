package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class intake extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

        // Make sure intake spins properly with positive power
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        // Always required for simple motors
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            // Hold X to spin the intake
            if (gamepad1.x) {
                intake.setPower(1.0);
            } else {
                intake.setPower(0.0);
            }

            telemetry.addData("Intake Power", intake.getPower());
            telemetry.update();
        }
    }
}
