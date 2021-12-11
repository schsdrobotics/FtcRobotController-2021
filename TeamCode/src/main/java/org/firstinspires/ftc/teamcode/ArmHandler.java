package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmHandler {
    private final ServoWrapper vertical;
    private final ServoWrapper horizontal;
    private final Gamepad controller;
    private long lastMillis = -1;

    public ArmHandler(HardwareMap map, Gamepad controller) {
        this.controller = controller;
        vertical = ServoWrapper.get(map, "verticalServo");
        horizontal = ServoWrapper.get(map, "horizontalServo");
        vertical.setAndUpdate(0.5);
        horizontal.setAndUpdate(0.5);
    }

    public void tick() {
        long millis = System.currentTimeMillis();
        if (millis - lastMillis > 50) { // want a constant speed
            if (controller.dpad_left) {
                horizontal.setPos(horizontal.getPos() + 5);
            } else if (controller.dpad_right) {
                horizontal.setPos(horizontal.getPos() - 5);
            }

            if (controller.dpad_up) {
                vertical.setPos(vertical.getPos() + 5);
            } else if (controller.dpad_down) {
                vertical.setPos(vertical.getPos() - 5);
            }
            this.lastMillis = millis;
        }
    }
}