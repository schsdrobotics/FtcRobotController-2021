package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LightHandler {
    private final DigitalChannel red;
    private final DigitalChannel green;

    public LightHandler(HardwareMap map) {
        red = map.get(DigitalChannel.class, "red");
        green = map.get(DigitalChannel.class, "green");
        red.setMode(DigitalChannel.Mode.OUTPUT);
        green.setMode(DigitalChannel.Mode.OUTPUT);
    }

    public void setColor(Color color) {
        red.setState(color.red);
        green.setState(color.green);
    }

    public enum Color {
        OFF(false, false),
        RED(true, false),
        YELLOW(true, true),
        GREEN(false, true);

        public final boolean red;
        public final boolean green;

        Color(boolean red, boolean green) {
            this.red = red;
            this.green = green;
        }
    }
}
