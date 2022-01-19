package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DuckHandler {
    private final DcMotorEx motor;
    private final Gamepad controller;
    private boolean reversed = false;
    private long millisAtReverse = -1;
    private long milliTimer = 0;
    private long lastMillis = System.currentTimeMillis();
    private boolean on = false;
    private static final double MOTOR_TICKS_PER_REV = 28;
    private static final double MOTOR_GEAR_RATIO = 16;
    private static final double MILLIS_FOR_PLATE_REV = 825;

    public DuckHandler(HardwareMap map, Gamepad controller) {
        motor = map.get(DcMotorEx.class, "duckMotor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.controller = controller;
    }

    public void tick() {
        long millis = System.currentTimeMillis();
        if (controller != null) {
            on = controller.guide;
            // reversing
            if (controller.start && millis - millisAtReverse > 500) {
                reversed = !reversed;
                milliTimer = 0;
                millisAtReverse = millis;
            }
        }
        if (on) {
            double speed = rpmToTicksPerSecond(175) * (reversed ? -1 : 1);

            // ramping up speed
            long milliDiff = millis - lastMillis;
            milliTimer += milliDiff;
            if (milliTimer > MILLIS_FOR_PLATE_REV) {
                speed = rpmToTicksPerSecond(400) * (reversed ? -1 : 1);
            }
            motor.setVelocity(speed);
        } else {
            if (milliTimer > 0) milliTimer = 0;
            motor.setPower(0);
        }
        lastMillis = millis;
    }

    public void start() {
        on = true;
    }

    public void stop() {
        on = false;
    }

    public void reverse() {
        reversed = !reversed;
    }

    public static double rpmToTicksPerSecond(double rpm) {
        return (rpm * MOTOR_TICKS_PER_REV * MOTOR_GEAR_RATIO) / 60;
    }

    public static double ticksPerSecondToRpm(double ticksPerSecond) {
        return ticksPerSecond * 60 / MOTOR_GEAR_RATIO / MOTOR_TICKS_PER_REV;
    }
}
