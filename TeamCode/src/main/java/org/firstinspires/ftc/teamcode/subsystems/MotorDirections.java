package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="MotorAndEncoderTest", group="Test")
public class MotorDirections extends OpMode {

    private DcMotorEx fl, fr, bl, br;

    @Override
    public void init() {
        // Map motors
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        // Set motor directions (typical Mecanum setup)
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset encoders
        fl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Set to run using encoder
        fl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        double power = 0.3; // forward power

        // Apply same power to all motors
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);

        // Telemetry: motor powers
        telemetry.addData("Motor Powers", "FL: %.2f | FR: %.2f | BL: %.2f | BR: %.2f",
                fl.getPower(), fr.getPower(), bl.getPower(), br.getPower());

        // Telemetry: encoder positions
        telemetry.addData("Encoders", "FL: %d | FR: %d | BL: %d | BR: %d",
                fl.getCurrentPosition(),
                fr.getCurrentPosition(),
                bl.getCurrentPosition(),
                br.getCurrentPosition());

        // Instructions
        telemetry.addData("Instructions",
                "Forward movement should increase all encoders. " +
                        "If a motor goes backward, reverse its direction in code.");

        telemetry.update();
    }
}
