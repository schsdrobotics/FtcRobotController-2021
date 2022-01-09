package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeServoHandler {
    public final ServoWrapper servo;
    private final Gamepad controller;
    private long lastMillis = System.currentTimeMillis();
    public final double HOOKED = 0.5;
    public final double RELEASED = 0.8;

    public IntakeServoHandler(HardwareMap map, Gamepad controller) {
        this.controller = controller;
        servo = ServoWrapper.get(map, "intakeServo");
    }

    public void goToPos(double pos) {
        if (pos < HOOKED) {
            servo.setPos(HOOKED);
        }
        else if (pos > RELEASED) {
            servo.setPos(RELEASED);
        }
        else {
            servo.setPos(pos);
        }
        servo.update();
    }
}