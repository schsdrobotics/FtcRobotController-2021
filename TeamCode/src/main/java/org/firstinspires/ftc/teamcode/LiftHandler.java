package org.firstinspires.ftc.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.Predicate;

@RequiresApi(api = Build.VERSION_CODES.N)
public class LiftHandler {
    private final Telemetry telemetry;
    private final MotorWrapper motor;
    private final Gamepad controller;
    private final TouchSensor magneticSwitchLow;
    private final TouchSensor magneticSwitchMiddle;
    private final TouchSensor magneticSwitchHigh;
    private final TouchSensor[] sensors; // sensors[Position.ordinal()] gets the sensor for a given position
    private Position previousLocation = Position.LOW;
    private Position target = Position.LOW;
    public boolean initialized = false;
    public final int LOW = 25;
    public final int MIDDLE = 500;
    public final int HIGH = 1000;

    public LiftHandler(HardwareMap map, Gamepad controller, Telemetry telemetry) {
        this.telemetry = telemetry;
        motor = MotorWrapper.get("liftMotor", map);
        this.controller = controller;
        magneticSwitchLow = map.get(TouchSensor.class, "magneticSwitchLow");
        magneticSwitchMiddle = map.get(TouchSensor.class, "magneticSwitchMiddle");
        magneticSwitchHigh = map.get(TouchSensor.class, "magneticSwitchHigh");
        sensors = new TouchSensor[] {magneticSwitchLow, magneticSwitchMiddle, magneticSwitchHigh};
        motor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void goToStart() { // CANNOT PRESS START UNTIL THIS FINISHES OR ELSE IT BREAKS
        int motorPosition = motor.motor.getCurrentPosition();
        if (magneticSwitchLow.isPressed() || Position.LOW.test(motorPosition)) {
            motor.setAndUpdate(0);
            initialized = true;
            telemetry.addData("Ready!", "");
            return;
        }
        telemetry.addData("lift pos: ", motorPosition);
        motor.setAndUpdate(Integer.compare(25, motorPosition));
    }

    public void finishInit() {
        motor.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void tick() {
        if (controller != null) {
            boolean x = controller.x;
            boolean y = controller.y;
            boolean b = controller.b;
            if (motor.getPower() == 0 && (!(x && y) && !(y && b) && !(x && b))) { // if only 1 button is pressed
                if (x) {
                    target = Position.LOW;
                } else if (y) {
                    target = Position.MIDDLE;
                } else if (b) {
                    target = Position.HIGH;
                }
            }
        }
        pursueTarget();
    }

    public void pursueTarget() {
        if (target == previousLocation) return;

        // Go in required direction
        int speed = Position.getSpeed(previousLocation, target);
        motor.setPower(-speed);

        TouchSensor sensorToHit = sensors[target.ordinal()];

        // If the switch or the encoder finds the correct position
        if (sensorToHit.isPressed() || target.test(motor.motor.getCurrentPosition())) {
            previousLocation = target;
            motor.setPower(0);
            // Should we zero out the encoder to match the magnets when the correct position is found?
        }

        motor.update();
    }

    public void pursueTargetAuto(int pos) {
        // goToPosition will make RunMode RUN_TO_POSITION
        motor.goToPosition(pos, 1);
        motor.update();
    }

    public enum Position implements Predicate<Integer> { // FIXME test these positions
        // When we have a robot, define the low position such that when the robot starts it is always on a magnet.
        LOW(pos -> pos >= 0 && pos < 50),
        MIDDLE(pos -> pos >= 475 && pos <= 525),
        HIGH(pos -> pos > 1000);

        private final Predicate<Integer> inRange;

        Position(Predicate<Integer> inRange) {
            this.inRange = inRange;
        }

        @Override
        public boolean test(Integer integer) {
            return inRange.test(integer);
        }

        public static int getSpeed(Position current, Position target) {
            return Integer.compare(current.ordinal(), target.ordinal());
        }
    }

    public void setTarget(Position target) {
        this.target = target;
    }
}