package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColorSensorHandler {
    private ColorSensor sensor;
    private final int MAXERROR = 20;

    public ColorSensorHandler(HardwareMap map) {
        sensor = map.get(ColorSensor.class, "colorSensor");
    }

    public int[] getRGBValues() {
        return new int[]{sensor.red(), sensor.green(), sensor.blue()};
    }

    public boolean testForFreight(int[] RGB) {
        //Test ball, block, and duck and see if it detects anything
        for (Objects object : Objects.values()) {
            //Test red, green, and blue values
            for (int i = 0; i < 3; i++) {
                //Passes test if current reading is between theoretical +or- MAXERROR
                int error = Math.abs(RGB[i] - object.RGB[i]);
                if (error >= MAXERROR) {
                    break;
                }
                //If it has passed all 3 tests, return true
                if (i == 2) {
                    return true;
                }
            }
        }
        return false;
    }

    public enum Objects {
        //TODO: tune these values using the ColorSensorTest opMode
        BALL(new int[]{0, 0, 0}),
        BLOCK(new int[]{0, 0, 0}),
        DUCK(new int[]{0, 0, 0});

        public final int[] RGB;

        Objects(int[] RGB) {
            this.RGB = RGB;
        }
    }
}
