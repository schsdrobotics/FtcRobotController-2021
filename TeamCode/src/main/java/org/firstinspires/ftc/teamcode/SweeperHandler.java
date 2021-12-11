package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SweeperHandler {
    private final MotorWrapper motor;
    private final Gamepad controller;

    public SweeperHandler(HardwareMap map, Gamepad controller) {
        motor = MotorWrapper.get("sweeperMotor", map);
        this.controller = controller;
    }

    public void tick() {
        motor.setPower(0);
        float rt = controller.right_trigger;
        float lt = controller.left_trigger;
        if (rt > 0) { // forwards
            forwards(rt);
        } else if (lt > 0) { // reverse
            backwards(lt);
        }
        motor.update();
    }

    public void forwards(double power) {
        motor.setAndUpdate(power);
    }

    public void backwards(double power) {
        motor.setAndUpdate(-power);
    }
}
