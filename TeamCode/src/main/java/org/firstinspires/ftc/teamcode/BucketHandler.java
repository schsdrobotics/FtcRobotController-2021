package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BucketHandler {
    private final ServoWrapper servo;
    private final Gamepad controller;

    public BucketHandler(HardwareMap map, Gamepad controller) {
        this.controller = controller;
        servo = ServoWrapper.get(map, "bucketServo");
    }

     public void tick() {
        if (controller != null) {
            if (controller.right_trigger <= 0.1) {
                backwards();
            } else forwards();
        }
     }

     public void forwards() {
         servo.min();
     }

     public void backwards() {
        servo.max();
     }

     public void halfway() {
        servo.setAndUpdate(0.5);
     }
}