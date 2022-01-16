package org.firstinspires.ftc.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.Predicate;

@RequiresApi(api = Build.VERSION_CODES.N)
public class LiftHandler {
    private final Telemetry telemetry;
    private final MotorWrapper motor;
    private final Gamepad gamepad;
//    private final TouchSensor magneticSwitchLow;
//    private final TouchSensor magneticSwitchMiddle;
//    private final TouchSensor magneticSwitchHigh;
//    private final TouchSensor[] sensors; // sensors[Position.ordinal()] gets the sensor for a given position
    private Position previousLocation = Position.LOW;
    private Position target = Position.LOW;
    public boolean initialized = true;
    public static final int INTAKING = 10;
    public static final int LOW = 20;
    public static final int MIDDLE = 190;
    public static final int HIGH = 310;

    public LiftHandler(HardwareMap map, Gamepad gamepad, Telemetry telemetry) {
        this.telemetry = telemetry;
        motor = MotorWrapper.get("liftMotor", map);
        this.gamepad = gamepad;
//        magneticSwitchLow = map.get(TouchSensor.class, "magneticSwitchLow");
//        magneticSwitchMiddle = map.get(TouchSensor.class, "magneticSwitchMiddle");
//        magneticSwitchHigh = map.get(TouchSensor.class, "magneticSwitchHigh");
//        sensors = new TouchSensor[] {magneticSwitchLow, magneticSwitchMiddle, magneticSwitchHigh};
        motor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void reset() {
        DcMotor.RunMode mode = motor.motor.getMode();
        motor.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.motor.setMode(mode);
        target = Position.LOW;
        previousLocation = Position.LOW;
    }

    public void tick() {
        if (gamepad != null) {
            motor.setAndUpdate(gamepad.left_stick_y);
            if (gamepad.left_stick_button && gamepad.right_stick_button &&
                    gamepad.left_bumper && gamepad.right_bumper) {
                reset();
            }

            boolean x = gamepad.x;
            boolean y = gamepad.y;
            boolean b = gamepad.b;
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
        motor.motor.setPower(speed > 0 ? 0.85 : -0.85);

        // If the switch or the encoder finds the correct position
        if (target.test(motor.motor.getCurrentPosition())) {
            previousLocation = target;
            motor.motor.setPower(0);
        }

//        motor.update();
    }

    public void pursueTargetAuto(int pos) {
        // goToPosition will make RunMode RUN_TO_POSITION
        motor.goToPosition(pos, 1);
        motor.update();
    }

    public enum Position implements Predicate<Integer> {
        LOW(pos -> pos <= 10),
        MIDDLE(pos -> pos >= 120 && pos <= 155),
        HIGH(pos -> pos > 300);

        private final Predicate<Integer> inRange;

        Position(Predicate<Integer> inRange) {
            this.inRange = inRange;
        }

        @Override
        public boolean test(Integer integer) {
            return inRange.test(integer);
        }

        public static int getSpeed(Position current, Position target) {
            return -Integer.compare(current.ordinal(), target.ordinal());
        }
    }

    public void setTarget(Position target) {
        this.target = target;
    }
}