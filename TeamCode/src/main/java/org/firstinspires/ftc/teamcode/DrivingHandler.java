package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class DrivingHandler {
    private final OpMode opMode;
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;
    private final Gamepad controller;

    public DrivingHandler(OpMode opMode) {
        this.opMode = opMode;
        frontLeft = opMode.hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRight = opMode.hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeft = opMode.hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRight = opMode.hardwareMap.get(DcMotor.class, "backRightMotor");
        controller = opMode.gamepad1;
    }

    public void tick() {
        frontLeft.setPower(controller.left_stick_x);
        frontRight.setPower(controller.left_stick_y);
        backLeft.setPower(controller.right_stick_x);
        backRight.setPower(controller.right_stick_y);
    }
}
