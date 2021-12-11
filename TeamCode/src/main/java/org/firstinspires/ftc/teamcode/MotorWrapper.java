package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * A wrapper around a motor that allows for delaying updating until later
 */
public class MotorWrapper {
    public static final int PERCENT_OF_MAX = 80;
    public final DcMotor motor;
    private double power;
    private final String name;

    public MotorWrapper(DcMotor motor, String name) {
        this.motor = motor;
        this.name = name;
    }

    public void setPower(double power) {
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

    public static MotorWrapper get(String name, HardwareMap map) {
        return new MotorWrapper(map.get(DcMotor.class, name), name);
    }
}
