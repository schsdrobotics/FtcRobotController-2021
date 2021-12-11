package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoWrapper {
    public static final int LIMIT = 95;
    public final Servo servo;
    public final String name;
    private double pos;

    public ServoWrapper(Servo servo, String name) {
        this.servo = servo;
        this.name = name;
    }

    public void setPos(double pos) {
        double maxMin = LIMIT / 100f;
        if (pos > maxMin) pos = maxMin;
        if (pos < 1 - maxMin) pos = 1 - maxMin;
        this.pos = pos;
    }

    public double getPos() {
        return pos;
    }

    public void update() {
        servo.setPosition(pos);
    }

    public void setAndUpdate(double pos) {
        setPos(pos);
        update();
    }

    public void max() {
        setAndUpdate(LIMIT / 100f);
    }

    public void min() {
        setAndUpdate(1 - (LIMIT / 100f));
    }

    public static ServoWrapper get(HardwareMap map, String name) {
        return new ServoWrapper(map.get(Servo.class, name), name);
    }
}
