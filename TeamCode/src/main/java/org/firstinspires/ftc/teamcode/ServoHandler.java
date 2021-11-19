package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoHandler {
    private OpMode opMode;
    private Servo servo;
    private Gamepad controller;

    public ServoHandler(OpMode opMode) {
        this.opMode = opMode;
        controller = this.opMode.gamepad2;
        servo = opMode.hardwareMap.get(Servo.class, "servo1");
    }

    public void tick() {
        servo.setPosition((controller.right_trigger != 0) ? 0.95 : 0.05);
    }
}