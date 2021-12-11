package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmHandler {
    private final ServoWrapper vertical;
    private final ServoWrapper horizontal;
    private final Gamepad controller;
    private long lastMillis = System.currentTimeMillis();

    public ArmHandler(HardwareMap map, Gamepad controller) {
        this.controller = controller;
        vertical = ServoWrapper.get(map, "verticalServo");
        horizontal = ServoWrapper.get(map, "horizontalServo");
    }

    public void tick() {
        long millis = System.currentTimeMillis();
        if (millis - lastMillis > 10) { // want a constant speed
            if (controller.dpad_left) {
                horizontal.setPos(horizontal.getPos() + 0.01);
            } else if (controller.dpad_right) {
                horizontal.setPos(horizontal.getPos() - 0.01);
            }

            if (controller.dpad_up) {
                vertical.setPos(vertical.getPos() + 0.01);
            } else if (controller.dpad_down) {
                vertical.setPos(vertical.getPos() - 0.01);
            }
            horizontal.update();
            vertical.update();
            this.lastMillis = millis;
        }
    }
}