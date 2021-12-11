package org.firstinspires.ftc.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.joml.Vector2d;

import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Predicate;

/**
 * Handles everything with driving the 4 main wheels.
 */
@RequiresApi(api = Build.VERSION_CODES.N)
public class DrivingHandler {
    /**
     * A vector pointing straight right
     */
    public static final Vector2d RIGHT = new Vector2d(1, 0);

    /** motor 1 */
    private final MotorWrapper frontLeft;
    /** motor 2 */
    private final MotorWrapper frontRight;
    /** motor 3 */
    private final MotorWrapper backLeft;
    /** motor 4 */
    private final MotorWrapper backRight;
    private final MotorWrapper[] all;
    private final Gamepad controller;

    public DrivingHandler(HardwareMap map, Gamepad controller) {
        frontLeft = MotorWrapper.get("frontLeftMotor", map);
        frontRight = MotorWrapper.get("frontRightMotor", map);
        frontLeft.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft = MotorWrapper.get("backLeftMotor", map);
        backRight = MotorWrapper.get("backRightMotor", map);
        backLeft.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        all = new MotorWrapper[] {frontLeft, frontRight, backLeft, backRight};
        this.controller = controller;
    }

    /**
     * Runs once every OpMode loop.
     */
    public void tick() {
        stop();
        double x = controller.left_stick_x;
        double y = controller.left_stick_y;

        // --- driving ---
        if (!(x == 0 && y == 0)) { // deadzone
            Vector2d vec = new Vector2d(x, y);
            double angle = Math.toDegrees(vec.angle(RIGHT));
            Direction direction = Direction.fromAngle(angle);
            double power = vec.length();
            direction.driveIn(power, this);
        }
        // --- rotation ---
        x = controller.right_stick_x;
        if (!(x == 0)) { // deadzone
            frontLeft.setPower(frontLeft.getPower() + x);
            backLeft.setPower(backLeft.getPower() + x);
            frontRight.setPower(frontRight.getPower() - x);
            backRight.setPower(backRight.getPower() - x);
        }
        updateAll();
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

    /**
     * An enum with 1 value for each of the 8 main directions.
     * Each entry has a Predicate and a BiConsumer.
     * The Predicate determines, based on an angle (in degrees)
     * fed into it, whether or not the angle falls in its range.
     * The BiConsumer is invoked to actually move the robot in the direction.
     */
    public enum Direction implements Predicate<Double> {
        UP(angle -> angle == 90, (power, handler) -> {
            handler.forEach(motor -> motor.setPower(power));
        }),
        UP_RIGHT(angle -> angle < 90 && angle > 0, (power, handler) -> {
//            handler.frontRight.setPower(-power / 2);
            handler.backRight.setPower(power);
            handler.frontLeft.setPower(power);
//            handler.backLeft.setPower(-power / 2);
        }),
        RIGHT(angle -> angle == 0, (power, handler) -> {
            handler.frontRight.setPower(-power);
            handler.backRight.setPower(power);
            handler.frontLeft.setPower(power);
            handler.backLeft.setPower(-power);
        }),
        DOWN_RIGHT(angle -> angle < 0 && angle > -90, (power, handler) -> {
            handler.frontRight.setPower(-power);
//            handler.backRight.setPower(power / 2);
//            handler.frontLeft.setPower(power / 2);
            handler.backLeft.setPower(-power);
        }),
        DOWN(angle -> angle == -90, (power, handler) -> {
            handler.forEach(motor -> motor.setPower(-power));
        }),
        DOWN_LEFT(angle -> angle < -90 && angle > -180, (power, handler) -> {
//            handler.frontRight.setPower(power / 2);
            handler.backRight.setPower(-power);
            handler.frontLeft.setPower(-power);
//            handler.backLeft.setPower(power / 2);
        }),
        LEFT(angle -> angle == -180, (power, handler) -> {
            handler.frontRight.setPower(power);
            handler.backRight.setPower(-power);
            handler.frontLeft.setPower(-power);
            handler.backLeft.setPower(power);
        }),
        UP_LEFT(angle -> angle <= 180 && angle > 90, (power, handler) -> {
            handler.frontRight.setPower(power);
//            handler.backRight.setPower(-power / 2);
//            handler.frontLeft.setPower(-power / 2);
            handler.backLeft.setPower(power);
        });

        private final Predicate<Double> angleTest;
        private final BiConsumer<Double, DrivingHandler> mover;

        Direction(Predicate<Double> angleTest, BiConsumer<Double, DrivingHandler> mover) {
            this.angleTest = angleTest;
            this.mover = mover;
        }

        @Override
        public boolean test(Double angle) {
            return angleTest.test(angle);
        }

        public void driveIn(double power, DrivingHandler handler) {
            mover.accept(power, handler);
        }

        /**
         * Finds a Direction from the fed in angle in degrees
         */
        public static Direction fromAngle(double angle) {
            for (Direction d : values()) {
                if (d.angleTest.test(angle)) return d;
            }
            return RIGHT;
        }
    }
}
