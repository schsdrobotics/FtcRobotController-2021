package org.firstinspires.ftc.teamcode;

import com.google.gson.internal.bind.util.ISO8601Utils;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoHandler {
    private final ServoWrapper servo;
    private final Gamepad controller;

    public ServoHandler(HardwareMap map, Gamepad controller) {
        this.controller = controller;
        servo = ServoWrapper.get(map, "servo1");
    }

     public void tick() {
        if (controller.right_trigger != 0) {
            servo.max();
        } else servo.min();
     }
}