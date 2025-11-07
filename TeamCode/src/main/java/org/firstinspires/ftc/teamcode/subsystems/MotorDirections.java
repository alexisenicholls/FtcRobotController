package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@TeleOp(name="Motor & Encoder Direction Test", group="Tests")
public class MotorDirections extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // === MOTOR SETUP ===
        DcMotor fl = hardwareMap.get(DcMotor.class, "fl");
        DcMotor fr = hardwareMap.get(DcMotor.class, "fr");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");

        // Set motor directions (from your constants)
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reset encoders
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        // Record initial encoder positions
        int flStart = fl.getCurrentPosition();
        int frStart = fr.getCurrentPosition();
        int blStart = bl.getCurrentPosition();
        int brStart = br.getCurrentPosition();

        // Run motors forward briefly
        fl.setPower(0.3);
        fr.setPower(0.3);
        bl.setPower(0.3);
        br.setPower(0.3);

        sleep(1000);

        // Stop all motors
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        // Read final encoder positions
        int flEnd = fl.getCurrentPosition();
        int frEnd = fr.getCurrentPosition();
        int blEnd = bl.getCurrentPosition();
        int brEnd = br.getCurrentPosition();

        telemetry.addLine("=== MOTOR DIRECTION TEST ===");
        telemetry.addData("Front Left (fl)", fl.getDirection());
        telemetry.addData("Front Right (fr)", fr.getDirection());
        telemetry.addData("Back Left (bl)", bl.getDirection());
        telemetry.addData("Back Right (br)", br.getDirection());

        telemetry.addLine("\n=== ENCODER DIRECTION TEST ===");
        telemetry.addData("fl encoder", flEnd > flStart ? "FORWARD" : "REVERSE");
        telemetry.addData("fr encoder", frEnd > frStart ? "FORWARD" : "REVERSE");
        telemetry.addData("bl encoder", blEnd > blStart ? "FORWARD" : "REVERSE");
        telemetry.addData("br encoder", brEnd > brStart ? "FORWARD" : "REVERSE");

        telemetry.update();

        // Keep telemetry on screen
        while (opModeIsActive()) {
            idle();
        }
    }
}
