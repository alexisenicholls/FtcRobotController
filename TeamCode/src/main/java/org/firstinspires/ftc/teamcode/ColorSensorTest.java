package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp; // import this
import org.firstinspires.ftc.teamcode.TestColor.DetectedColor;

@TeleOp(name="Color Sensor Test", group="Test") // <-- required!
public class ColorSensorTest extends OpMode {
    TestColor bench = new TestColor();
    DetectedColor detectedColor;

    @Override
    public void init() {
        bench.init(hardwareMap);
    }

    @Override
    public void loop() {
        detectedColor = bench.getDetectedColor(telemetry); // get color
        telemetry.addData("Color Detected", detectedColor.toString());
        telemetry.update();

    }
}