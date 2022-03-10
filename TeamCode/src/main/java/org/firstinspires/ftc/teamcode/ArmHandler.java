package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmHandler {
    // public for debug
    private final Telemetry telemetry;
    public final ServoWrapper vertical;
    public final ServoWrapper horizontal;
    public final ServoWrapper mini;
    private final Gamepad controller;
    private boolean init = true;
    private long startMillis;
    private long lastMillis = System.currentTimeMillis();

    public ArmHandler(HardwareMap map, Gamepad controller, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.controller = controller;
        vertical = ServoWrapper.get(map, "verticalServo");
        horizontal = ServoWrapper.get(map, "horizontalServo");
        mini = ServoWrapper.get(map, "miniArmServo");
    }

    public void onStartControlled() {
        onStartAuto();
        horizontal.setAndUpdate(0.5);
        startMillis = System.currentTimeMillis();
    }

    public void onStartAuto() {
        vertical.setAndUpdate(0.64);
    }

    public void onStopAuto() {
        vertical.setAndUpdate(0.4); // update this to be whatever the arm is at when the robot first starts
    }

    public void tick() {
        telemetry.addData("ArmHozPos", horizontal.servo.getPosition());
        telemetry.addData("ArmVertPos", vertical.servo.getPosition());
        if (controller != null) {
            // move arm to perfect pickup pos
            if (controller.b) {
                vertical.setAndUpdate(0.09);
            }
            // horizontal
            // Horizontal servo is continuous
            if (controller.dpad_right) {  // if we want this on a joystick, change the condition to controller.left_stick_x != 0 etc.
//                horizontal.setPos(horizontal.getPos() - 0.01);
                horizontal.setPos(0.55);
            } else if (controller.dpad_left) {
//                horizontal.setPos(horizontal.getPos() + 0.01);
                horizontal.setPos(0.45);
            } else {
                horizontal.setPos(0.5);
            }
            horizontal.update();

            long millis = System.currentTimeMillis();
            // init and vertical
            if (millis - lastMillis > 40) {
                if (init) {
                    if (millis - startMillis > 2000) init = false;
                    else horizontal.setAndUpdate(0.4);
                } else {
                    if (controller.dpad_down) {
                        vertical.setPos(vertical.getPos() - 0.01);
                    } else if (controller.dpad_up) {
                        vertical.setPos(vertical.getPos() + 0.01);
                    }
                }

                vertical.update();
                this.lastMillis = millis;
            }
            // mini arm
            mini.setAndUpdate(controller.a ? 0.1 : 1);
        }
    }
}