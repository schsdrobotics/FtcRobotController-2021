package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class DuckHandler {
    private final MotorWrapper motor;
    private final OpMode opMode;
    private final Gamepad controller;
    private boolean reversed = false;
    private long millisAtReverse = -1;

    public DuckHandler(OpMode opMode) {
        this.opMode = opMode;
        motor = MotorWrapper.getMotor("duckMotor", this.opMode);
        controller = this.opMode.gamepad2;
    }

    public void tick() {
        if (controller.guide) {
            ((DcMotorEx) motor.motor).setVelocity(93 * (reversed ? -1 : 1));
            //System.out.println(((DcMotorEx) motor.motor).getCurrentPosition());
        } else motor.motor.setPower(0);
        long millis = System.currentTimeMillis();
        if (controller.left_stick_button && millis - millisAtReverse > 500) {
            reversed = !reversed;
            millisAtReverse = millis;
        }
    }
}
