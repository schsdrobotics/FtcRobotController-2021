/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opmodes

import android.os.Build
import androidx.annotation.RequiresApi
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.*

/*
Controls:
    Gamepad 1:
        Left Joystick:
            x = strafing
            y = going forward and backwards
        Right Joystick:
            x = rotating
        Right Trigger: sweeper forwards
        Left Trigger: sweeper backwards
        D-Pad: manual lift control (emergency)
    Gamepad 2:
        X: lift to low position
        Y: lift to middle position
        B: lift to high position
        Both sticks + both bumpers: reset lift encoder (emergency)
        Right Trigger: midway
        Left + Right Trigger: drop item
        Guide(button in center of controller): duck spinner
        start: reverse duck motor
        D-Pad: arm
 */

/**
 * The OpMode that runs when the robot is manually controlled.
 */
@RequiresApi(api = Build.VERSION_CODES.N)
@TeleOp(name="ControlledOpMode")
class ControlledOpMode : OpMode() {
    // Declare OpMode members.
    private val runtime = ElapsedTime()
    private lateinit var driving: DrivingHandler
    private lateinit var sweeper: SweeperHandler
    private lateinit var lift: LiftHandler
    private lateinit var bucket: BucketHandler
    private lateinit var duck: DuckHandler
    private lateinit var arm: ArmHandler
    private lateinit var cycles: CycleHandler
    private lateinit var intakeServo: IntakeServoHandler
    private lateinit var light: LightHandler
    private lateinit var xSensor: DistanceSensor

    override fun init() {
        driving = DrivingHandler(hardwareMap, gamepad1)
        sweeper = SweeperHandler(hardwareMap, gamepad1)
        lift = LiftHandler(hardwareMap, gamepad2, telemetry)
        bucket = BucketHandler(hardwareMap, gamepad2)
        duck = DuckHandler(hardwareMap, gamepad2)
        arm = ArmHandler(hardwareMap, gamepad2, telemetry)
        light = LightHandler(hardwareMap)
        xSensor = hardwareMap[DistanceSensor::class.java, "xCoordinateSensor"]
        cycles = CycleHandler(
            sweeper,
            bucket,
            lift,
            gamepad1,
            hardwareMap[
                DistanceSensor::class.java,
                "distanceSensor"
            ],
            light,
            telemetry
        )
        intakeServo = IntakeServoHandler(hardwareMap)
        intakeServo.hook()
        DuckHandler.rampUp = true
        telemetry.addData("Status", "Initialized")
    }

    override fun start() {
        intakeServo.release()
        arm.onStartControlled()
        runtime.reset()
    }

    override fun loop() {
        driving.tick()
        sweeper.tick()
        lift.tick()
        bucket.tick()
        duck.tick()
        arm.tick()
        cycles.tick()
        telemetry.addData("Status", "Run Time: $runtime")
        telemetry.addData("xSensor", xSensor.getDistance(DistanceUnit.CM))
    }
}