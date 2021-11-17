package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class LiftHandler {
    private OpMode opMode;
    private MotorWrapper motor;
    private Gamepad controller;

    public LiftHandler(OpMode opMode) {
        this.opMode = opMode;
        motor = MotorWrapper.getMotor("liftMotor", this.opMode);
        controller = this.opMode.gamepad2;
    }

    public void tick() {
        motor.setPower(0);
        boolean x = controller.x;
        boolean y = controller.y;
        boolean b = controller.b;
        if (x ^ y ^ b) {
            if(x) {
                motor.setPower(.8); //SAMPLE CODE CHANGE LATER
            }
            else if(y) {
                motor.setPower(.4); //SAMPLE CODE CHANGE LATER
            }
            else if(b) {
                motor.setPower(.2); //SAMPLE CODE CHANGE LATER
            }
        }
        // x is low, y is middle, b is high
        motor.update();
    }

    public void low(double power) {
        motor.setAndUpdate(power);
    }

    public void middle(double power) {
        motor.setAndUpdate(power);
    }

    public void high(double power) {
        motor.setAndUpdate(power);
    }
}