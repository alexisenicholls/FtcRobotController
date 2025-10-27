package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="ColorDetection", group="Test")
public class ColorDetection extends OpMode {

    // --- ENUM for Detected Color ---
    public enum DetectedColor {
        GREEN,
        PURPLE,
        UNKNOWN
    }

    // --- Hardware and Variables ---
    private NormalizedColorSensor colorSensor;
    private DetectedColor detectedColor = DetectedColor.UNKNOWN;

    @Override
    public void init() {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensorColour");
        colorSensor.setGain(8);
        telemetry.addLine("Color Sensor Initialized");
    }

    @Override
    public void loop() {
        detectedColor = getDetectedColor(telemetry);
        telemetry.addData("Detected Color", detectedColor);
        telemetry.update();
    }

    // --- Main Color Detection Logic ---
    private DetectedColor getDetectedColor(Telemetry telemetry) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float normRed   = colors.red / colors.alpha;
        float normGreen = colors.green / colors.alpha;
        float normBlue  = colors.blue / colors.alpha;

        telemetry.addData("Red", normRed);
        telemetry.addData("Green", normGreen);
        telemetry.addData("Blue", normBlue);

        if (normGreen > normRed && normGreen > normBlue) {
            return DetectedColor.GREEN;
        } else if ((normRed + normBlue) / 2.0 > normGreen) {
            return DetectedColor.PURPLE;
        } else {
            return DetectedColor.UNKNOWN;
        }
    }

    // --- Getter (for use by other systems) ---
    public DetectedColor getCurrentColor() {
        return detectedColor;
    }
}
