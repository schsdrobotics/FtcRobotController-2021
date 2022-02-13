package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColorSensorHandler {
    private ColorSensor sensor;
    private final int MAXERROR = 420;

    public ColorSensorHandler(HardwareMap map) {
        sensor = map.get(ColorSensor.class, "colorSensor");
    }

    public int[] getRGBValues() {
        return new int[]{sensor.red(), sensor.green(), sensor.blue()};
    }

    public boolean isFreightDetected() {
        //Get current reading
        int[] RGB = getRGBValues();

        //Test ball, block, and duck and see if it detects anything
        for (Objects object : Objects.values()) {
            int error = 0;
            for (int i = 0; i < 3; i++) {
                error += Math.abs(RGB[i] - object.RGB[i]);
            }
            if (error <= MAXERROR) {
                return true;
            }
        }
        return false;
    }

    public enum Objects {
        //TODO: tune these values using ColorSensorTest
        BALL(new int[]{0, 0, 0}),
        BLOCK(new int[]{0, 0, 0}),
        DUCK(new int[]{0, 0, 0});

        public final int[] RGB;

        Objects(int[] RGB) {
            this.RGB = RGB;
        }
    }
}
