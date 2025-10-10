package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestColor {
    NormalizedColorSensor colorSensor;

    public enum DetectedColor {
        GREEN,
        PURPLE,
        UNKNOWN,
    }

    public void init(HardwareMap hwMap) {
        colorSensor = hwMap.get(NormalizedColorSensor.class, "sensorColour");
        colorSensor.setGain(8);
    }

    public DetectedColor getDetectedColor(Telemetry telemetry) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors(); // returns 4 values


        float normRed   = colors.red / colors.alpha;
        float normGreen = colors.green / colors.alpha;
        float normBlue  = colors.blue / colors.alpha;

        telemetry.addData("Red", normRed);
        telemetry.addData("Green", normGreen);
        telemetry.addData("Blue", normBlue);


        if (normGreen > normRed && normGreen > normBlue) {
            return DetectedColor.GREEN;
        }

        else if ((normRed + normBlue) / 2.0 > normGreen) {
            return DetectedColor.PURPLE;
        }
        else {
            return DetectedColor.UNKNOWN;
        }
    }
}
