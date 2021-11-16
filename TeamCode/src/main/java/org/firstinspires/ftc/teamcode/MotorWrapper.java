package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * A wrapper around a motor that allows for delaying updating until later
 */
public class MotorWrapper {
    public final DcMotor motor;
    private double power;

    public MotorWrapper(DcMotor motor) {
        this.motor = motor;
    }

    public void setPower(double power) {
        // software limits of 80%
        if (power > 0.8) power = 0.8;
        if (power < -0.8) power = -0.8;
        this.power = power;
    }

    public double getPower() {
        return power;
    }

    public void setAndUpdate(double power) {
        setPower(power);
        update();
    }

    public void update() {
        motor.setPower(power);
    }
}
