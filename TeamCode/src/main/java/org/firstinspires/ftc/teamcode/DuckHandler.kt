package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap

class DuckHandler(map: HardwareMap, private val controller: Gamepad?) {
    companion object {
        private const val MOTOR_TICKS_PER_REV = 28.0
        private const val MOTOR_GEAR_RATIO = 16.0
        const val MILLIS_FOR_PLATE_REV = 825.0
        const val RPMSPEED = 225.0

        @JvmField
        var rampUp = false

        private fun rpmToTicksPerSecond(rpm: Double): Double {
            return rpm * MOTOR_TICKS_PER_REV * MOTOR_GEAR_RATIO / 60
        }

        private fun ticksPerSecondToRpm(ticksPerSecond: Double): Double {
            return ticksPerSecond * 60 / MOTOR_GEAR_RATIO / MOTOR_TICKS_PER_REV
        }
    }

    private val motor = map[DcMotorEx::class.java, "duckMotor"]
    private var reversed = false
    private var millisAtReverse = -1L
    private var milliTimer = 0L
    private var lastMillis = System.currentTimeMillis()
    private var on = false

    init {
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    fun tick() {
        val millis = System.currentTimeMillis()
        val speed = rpmToTicksPerSecond(RPMSPEED) * if (reversed) -1 else 1
        if (controller != null) {
            on = controller.guide
            // reversing
            if (controller.start && millis - millisAtReverse > 500) {
                reverse()
                milliTimer = 0
                millisAtReverse = millis
            }
        }
        if (on) {
            // ramping up speed
            val milliDiff = millis - lastMillis
            milliTimer += milliDiff
            when {
                !rampUp || milliTimer <= MILLIS_FOR_PLATE_REV -> motor.velocity = speed
                else -> motor.power = if (reversed) -1.0 else 1.0 // ramp up if applicable
            }
        } else {
            if (milliTimer > 0) milliTimer = 0
            motor.power = 0.0
        }
    }

    fun start() {
        on = true
    }

    fun stop() {
        on = false
    }

    fun reverse() {
        reversed = !reversed
    }
}