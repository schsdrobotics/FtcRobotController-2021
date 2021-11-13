package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorWrapper {
    public final DcMotor motor;
    private double power;

    public MotorWrapper(DcMotor motor) {
        this.motor = motor;
    }

    public void setPower(double power) {
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
