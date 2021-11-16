package org.firstinspires.ftc.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.vuforia.Vec3F;

import org.joml.Vector2d;

import java.util.Vector;
import java.util.function.BiConsumer;
import java.util.function.BiPredicate;
import java.util.function.Consumer;
import java.util.function.Predicate;

@RequiresApi(api = Build.VERSION_CODES.N)
public class DrivingHandler {
    public static final Vector2d RIGHT = new Vector2d(1, 0);

    private final OpMode opMode;
    private final MotorWrapper frontLeft; // motor 1
    private final MotorWrapper frontRight; // motor 2
    private final MotorWrapper backLeft; // motor 3
    private final MotorWrapper backRight; // motor 4
    private final MotorWrapper[] all;
    private final Gamepad controller;

    public DrivingHandler(OpMode opMode) {
        this.opMode = opMode;
        frontLeft = new MotorWrapper(this.opMode.hardwareMap.get(DcMotor.class, "frontLeftMotor"));
        frontRight = new MotorWrapper(this.opMode.hardwareMap.get(DcMotor.class, "frontRightMotor"));
        frontRight.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft = new MotorWrapper(this.opMode.hardwareMap.get(DcMotor.class, "backLeftMotor"));
        backRight = new MotorWrapper(this.opMode.hardwareMap.get(DcMotor.class, "backRightMotor"));
        backRight.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        all = new MotorWrapper[] {frontLeft, frontRight, backLeft, backRight};
        controller = this.opMode.gamepad1;
    }

    public void tick() {
        stop();
        double x = controller.left_stick_x;
        double y = controller.left_stick_y;

        if (!(x == 0 && y == 0)) { // deadzone
            Vector2d vec = new Vector2d(x, y);
            double angle = Math.toDegrees(vec.angle(RIGHT));
            Direction direction = Direction.fromAngle(angle);
            double power = vec.length();
            direction.driveIn(power, this);
            System.out.println("angle: " + angle + "; direction: " + direction);
        }
        updateAll();
    }

    public void stop() {
        forEach(motor -> motor.setAndUpdate(0));
    }

    public void updateAll() {
        forEach(MotorWrapper::update);
    }

    public void forEach(Consumer<MotorWrapper> consumer) {
        for (MotorWrapper motor : all) {
            consumer.accept(motor);
        }
    }

    public static double map(double x, double inMin, double inMax, double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    public enum Direction implements Predicate<Double> {
        UP(angle -> angle == 90, (power, handler) -> {
            handler.forEach(motor -> motor.setPower(power));
        }),
        UP_RIGHT(angle -> angle < 90 && angle > 0, (power, handler) -> {
            handler.frontRight.setPower(power);
            handler.backRight.setPower(-power / 2);
            handler.frontLeft.setPower(-power / 2);
            handler.backLeft.setPower(power);
        }),
        RIGHT(angle -> angle == 0, (power, handler) -> {
            handler.frontRight.setPower(power);
            handler.backRight.setPower(-power);
            handler.frontLeft.setPower(-power);
            handler.backLeft.setPower(power);
        }),
        DOWN_RIGHT(angle -> angle < 0 && angle > -90, (power, handler) -> {
            handler.frontRight.setPower(-power);
            handler.backRight.setPower(power / 2);
            handler.frontLeft.setPower(power / 2);
            handler.backLeft.setPower(-power);
        }),
        DOWN(angle -> angle == -90, (power, handler) -> {
            handler.forEach(motor -> motor.setPower(-power));
        }),
        DOWN_LEFT(angle -> angle < -90 && angle > -180, (power, handler) -> {
            handler.frontRight.setPower(-power);
            handler.backRight.setPower(power / 2);
            handler.frontLeft.setPower(power / 2);
            handler.backLeft.setPower(-power);
        }),
        LEFT(angle -> angle == -180, (power, handler) -> {
            handler.frontRight.setPower(-power);
            handler.backRight.setPower(power);
            handler.frontLeft.setPower(power);
            handler.backLeft.setPower(-power);
        }),
        UP_LEFT(angle -> angle <= 180 && angle > 90, (power, handler) -> {
            handler.frontRight.setPower(-power / 2);
            handler.backRight.setPower(power);
            handler.frontLeft.setPower(power);
            handler.backLeft.setPower(-power / 2);
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

        public static Direction fromAngle(double angle) {
            for (Direction d : values()) {
                if (d.angleTest.test(angle)) return d;
            }
            return RIGHT;
        }
    }
}
