package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeServoHandler {
    public final ServoWrapper servo;
    private final Gamepad controller;
    private long lastMillis = System.currentTimeMillis();
    public final double UP = 0.95;
    public final double DOWN = 0.05;
    public final double range = UP - DOWN;

    public IntakeServoHandler(HardwareMap map, Gamepad controller) {
        this.controller = controller;
        servo = ServoWrapper.get(map, "intakeServo");
    }

    public void tick() {
        long millis = System.currentTimeMillis();
        if (millis - lastMillis > 10) { // want a constant speed
            // Speed is based on how far the joystick is held
            servo.setPos(servo.getPos() + (0.01 * controller.left_stick_y));

            // This is the code if we want arm on d-pad
//            if (controller.dpad_left) {  // if we want this on a joystick, change the condition to controller.left_stick_x != 0 etc.
//                horizontal.setPos(horizontal.getPos() - 0.01);
//            } else if (controller.dpad_right) {
//                horizontal.setPos(horizontal.getPos() - 0.01);
//            }
//
//            if (controller.dpad_up) {
//                vertical.setPos(vertical.getPos() + 0.01);
//            } else if (controller.dpad_down) {
//                vertical.setPos(vertical.getPos() - 0.01);
//            }

            servo.update();
            this.lastMillis = millis;
        }
    }

    public void goToPos(double pos) {
        if (pos > UP) {
            servo.setPos(UP);
        }
        else if (pos < DOWN) {
            servo.setPos(DOWN);
        }
        else {
            servo.setPos(pos);
        }
        servo.update();
    }
}