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
import java.util.function.BiPredicate;
import java.util.function.Consumer;
import java.util.function.Predicate;

@RequiresApi(api = Build.VERSION_CODES.N)
public class DrivingHandler {
    private final OpMode opMode;
    private final MotorWrapper frontLeft;
    private final MotorWrapper frontRight;
    private final MotorWrapper backLeft;
    private final MotorWrapper backRight;
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
        System.out.println("x: " + x + "; y: " + y);
        if (!(x == 0 && y == 0)) { // deadzone
            forEach(motor -> motor.setPower(-y));
            if (x > 0) { // strafe right
                frontRight.setPower(-backRight.getPower());
                backLeft.setPower(-frontLeft.getPower());
            } else if (x < 0) { // strafe left
                frontLeft.setPower(-backLeft.getPower());
                backRight.setPower(-frontRight.getPower());
            }
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
}
