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
    public final MotorWrapper motor;
    private final Gamepad gamepad;
    public boolean initialized = true;
    public volatile boolean shouldHoldPos = false;

    public LiftHandler(HardwareMap map, Gamepad gamepad, Telemetry telemetry) {
        this.telemetry = telemetry;
        motor = MotorWrapper.get("liftMotor", map);
        this.gamepad = gamepad;
        motor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void reset() {
        DcMotor.RunMode mode = motor.motor.getMode();
        motor.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.motor.setMode(mode);
    }

    public void tick() {
        telemetry.addData("lift pos", motor.motor.getCurrentPosition());
        if (gamepad != null && !shouldHoldPos) {
            motor.setAndUpdate(gamepad.left_stick_y);
            if (gamepad.left_stick_button && gamepad.right_stick_button &&
                    gamepad.left_bumper && gamepad.right_bumper) {
                reset();
            }

            boolean x = gamepad.x;
            boolean y = gamepad.y;
            boolean b = gamepad.b;
            if ((!(x && y) && !(y && b) && !(x && b))) { // if only 1 button is pressed
                Position target = null;
                if (x) {
                    target = Position.LOW;
                } else if (y) {
                    target = Position.MIDDLE;
                } else if (b) {
                    target = Position.HIGH;
                }
                if (target != null) pursueTarget(target);
            }
        }
    }

    private void pursueTarget(int pos) {
        // goToPosition will make RunMode RUN_TO_POSITION
        motor.goToPosition(pos, 1);
    }

    public void pursueTarget(Position position) {
        pursueTarget(position.pos);
    }

    public enum Position {
        LOW(10),
        MIDDLE(130),
        HIGH(300);

        public final int pos;

        Position(int pos) {
            this.pos = pos;
        }
    }
}