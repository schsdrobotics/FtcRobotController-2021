package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
    private static final double MOTOR_TICKS_PER_REV = 28;
    private static final double MOTOR_GEAR_RATIO = 16;
    private static final double MILLIS_FOR_PLATE_REV = 800;

    public DuckHandler(HardwareMap map, Gamepad controller) {
        motor = map.get(DcMotorEx.class, "duckMotor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.controller = controller;
    }

    public void tick() {
        long millis = System.currentTimeMillis();
        if (controller.guide) {
            double speed = rpmToTicksPerSecond(200) * (reversed ? -1 : 1);

            // ramping up speed
            long milliDiff = millis - lastMillis;
            milliTimer += milliDiff;
            if (milliTimer > MILLIS_FOR_PLATE_REV) {
                speed = rpmToTicksPerSecond(375) * (reversed ? -1 : 1);
            }
            // reversing
            if (controller.left_stick_button && millis - millisAtReverse > 500) {
                reversed = !reversed;
                milliTimer = 0;
                millisAtReverse = millis;
            }
            motor.setVelocity(speed);
        } else {
            if (milliTimer > 0) milliTimer = 0;
            motor.setPower(0);
        }
        lastMillis = millis;
    }

    public static double rpmToTicksPerSecond(double rpm) {
        return (rpm * MOTOR_TICKS_PER_REV * MOTOR_GEAR_RATIO) / 60;
    }

    public static double ticksPerSecondToRpm(double ticksPerSecond) {
        return ticksPerSecond * 60 / MOTOR_GEAR_RATIO / MOTOR_TICKS_PER_REV;
    }
}
