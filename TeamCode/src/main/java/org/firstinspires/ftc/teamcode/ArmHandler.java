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

    public void onStart() {
        vertical.setAndUpdate(0.3);
//        horizontal.setAndUpdate(0.4);
    }

    public void tick() {
        System.out.println("vert: " + vertical.servo.getPosition() + "; hoz: " + horizontal.servo.getPosition());

        //Horizontal servo is continuous
        if (controller.dpad_left) {  // if we want this on a joystick, change the condition to controller.left_stick_x != 0 etc.
//                horizontal.setPos(horizontal.getPos() - 0.01);
            horizontal.setPos(0.6);
        } else if (controller.dpad_right) {
//                horizontal.setPos(horizontal.getPos() + 0.01);
            horizontal.setPos(0.4);
        }
        else {
            horizontal.setPos(0.5);
        }
        horizontal.update();

        long millis = System.currentTimeMillis();
        if (millis - lastMillis > 10) {
            //Vertical servo is positional
            if (controller.dpad_up) {
//                vertical.setPos(vertical.getPos() - 0.01);
                vertical.setPos(vertical.getPos() + 0.01);
            } else if (controller.dpad_down) {
//                vertical.setPos(vertical.getPos() + 0.01);
                vertical.setPos(vertical.getPos() - 0.01);
            }

            vertical.update();
            this.lastMillis = millis;
        }
    }
}