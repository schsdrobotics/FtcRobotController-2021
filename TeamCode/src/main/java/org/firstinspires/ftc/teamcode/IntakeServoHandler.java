package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeServoHandler {
    public final ServoWrapper servo;
    private long lastMillis = System.currentTimeMillis();
    public static final double HOOKED = 0.3;
    public static final double RELEASED = 0.5;

    public IntakeServoHandler(HardwareMap map) {
        servo = ServoWrapper.get(map, "intakeServo");
    }

    public void goToPos(double pos) {
        if (pos < HOOKED) {
            servo.setPos(HOOKED);
        } else if (pos > RELEASED) {
            servo.setPos(RELEASED);
        } else {
            servo.setPos(pos);
        }
        servo.update();
    }
}