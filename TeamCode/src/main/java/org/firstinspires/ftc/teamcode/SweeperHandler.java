package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class SweeperHandler {
    private OpMode opMode;
    private MotorWrapper motor;
    private Gamepad controller;

    public SweeperHandler(OpMode opMode) {
        this.opMode = opMode;
        motor = MotorWrapper.getMotor("sweeperMotor", this.opMode);
        controller = this.opMode.gamepad1;
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
