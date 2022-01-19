package org.firstinspires.ftc.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.Consumer;

/**
 * Handles everything with driving the 4 main wheels.
 */
@RequiresApi(api = Build.VERSION_CODES.N)
public class DrivingHandler {
    /** motor 1 */
    private final MotorWrapper frontLeft;
    // each multiplier array holds 3 values: y (left stick y / drive), z (right stick x / rotate), and x (left stick x / strafe)
    private final int[] frontLeftMults = {+1, +1, +1};
    /** motor 2 */
    private final MotorWrapper frontRight;
    private final int[] frontRightMults = {-1, +1, +1};
    /** motor 3 */
    private final MotorWrapper backLeft;
    private final int[] backLeftMults = {+1, +1, -1};
    /** motor 4 */
    private final MotorWrapper backRight;
    private final int[] backRightMults = {-1, +1, -1};
    private final MotorWrapper[] all;
    private final Gamepad controller;

    public DrivingHandler(HardwareMap map, Gamepad controller) {
        frontLeft = MotorWrapper.get("frontLeftMotor", map);
        frontRight = MotorWrapper.get("frontRightMotor", map);
//        frontLeft.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft = MotorWrapper.get("backLeftMotor", map);
        backRight = MotorWrapper.get("backRightMotor", map);
//        backLeft.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        all = new MotorWrapper[] {frontLeft, frontRight, backLeft, backRight};
        this.controller = controller;
    }

    /**
     * Runs once every OpMode loop.
     */
    public void tick() {
        stop();
        double x = -controller.left_stick_x;
        double y = controller.left_stick_y;
        double z = -controller.right_stick_x;

        double total = Math.abs(x) + Math.abs(y) + Math.abs(z);
        if (Math.abs(total) == 0) {
            stop();
            return;
        }
        double forwardWeight = y / total;
        double strafeWeight = x / total;
        double rotWeight = z / total;
        double strength = Math.max(Math.sqrt(x * x + y * y), Math.abs(z));
        forEach(motor -> {
            int[] mults = getMultsForMotor(motor);
            double finalPower = ((mults[0] * forwardWeight) + (mults[1] * rotWeight) + (mults[2] * strafeWeight)) * strength * (MotorWrapper.PERCENT_OF_MAX/100f);
            motor.setPower(finalPower);
        });
        updateAll();
    }

    public int[] getMultsForMotor(MotorWrapper motor) {
        if (frontLeft == motor) {
            return frontLeftMults;
        } else if (frontRight == motor) {
            return frontRightMults;
        } else if (backLeft == motor) {
            return backLeftMults;
        } else if (backRight == motor) {
            return backRightMults;
        }
        throw new RuntimeException("what");
    }

    /**
     * Stops all 4 motors
     */
    public void stop() {
        forEach(motor -> motor.setAndUpdate(0));
    }

    /**
     * Updates all 4 motors with their current power
     */
    public void updateAll() {
        forEach(MotorWrapper::update);
    }

    /**
     * Runs a Consumer on every motor
     */
    public void forEach(Consumer<MotorWrapper> consumer) {
        for (MotorWrapper motor : all) {
            consumer.accept(motor);
        }
    }
}
