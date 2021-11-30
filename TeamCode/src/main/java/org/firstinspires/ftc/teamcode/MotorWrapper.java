package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * A wrapper around a motor that allows for delaying updating until later
 */
public class MotorWrapper {
    public static final double POWER_CHANGE_PER_TICK = 0.05;
    public static final int PERCENT_OF_MAX = 80;
    public final DcMotor motor;
    private double power;
    private String name;

    public MotorWrapper(DcMotor motor, String name) {
        this.motor = motor;
        this.name = name;
    }

    public void setPower(double power) {
        // software limits of 80%
        double maxMin = PERCENT_OF_MAX / 100f;
        if (power > maxMin) power = maxMin;
        if (power < -maxMin) power = -maxMin;
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

    public static MotorWrapper getMotor(String name, OpMode mode) {
        return new MotorWrapper(mode.hardwareMap.get(DcMotor.class, name), name);
    }
}
