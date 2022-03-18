package org.firstinspires.ftc.teamcode

import android.os.Build
import androidx.annotation.RequiresApi
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.abs
import kotlin.math.sqrt
import kotlin.math.max

/**
 * Handles everything with driving the 4 main wheels.
 */
@RequiresApi(api = Build.VERSION_CODES.N)
class DrivingHandler(map: HardwareMap, private val controller: Gamepad) {
    private val frontLeft = MotorWrapper["frontLeftMotor", map]
    // each multiplier array holds 3 values: y (left stick y / drive), z (right stick x / rotate), and x (left stick x / strafe)
    private val frontLeftMults = intArrayOf(+1, +1, +1)
    private val frontRight = MotorWrapper["frontRightMotor", map]
    private val frontRightMults = intArrayOf(-1, +1, +1)
    private val backLeft = MotorWrapper["backLeftMotor", map]
    private val backLeftMults = intArrayOf(+1, +1, -1)
    private val backRight = MotorWrapper["backRightMotor", map]
    private val backRightMults = intArrayOf(-1, +1, -1)
    private val all = arrayOf(frontLeft, frontRight, backLeft, backRight)

    /**
     * Runs once every OpMode loop.
     */
    fun tick() {
        stop()
        val x = -controller.left_stick_x.toDouble()
        val y = controller.left_stick_y.toDouble()
        val z = -controller.right_stick_x.toDouble()

        val total = abs(x) + abs(y) + abs(z)
        if (abs(total) == 0.0) {
            stop()
            return
        }
        // Normalize inputs
        val forwardWeight = y / total
        val strafeWeight = x / total
        val rotWeight = z / total
        // Account for the amount that the joystick is pushed
        val strength = max(sqrt(x * x + y * y), abs(z))
        all.forEach {
            val mults = getMultsForMotor(it)
            val finalPower = (mults[0] * forwardWeight + mults[1] * rotWeight + mults[2] * strafeWeight) * strength * (MotorWrapper.PERCENT_OF_MAX / 100f) // Power limited to 80% to match our motor limits
            it.power = finalPower
        }
        updateAll()
    }

    private fun getMultsForMotor(motor: MotorWrapper): IntArray {
        return when (motor) {
            frontLeft -> frontLeftMults
            frontRight -> frontRightMults
            backLeft -> backLeftMults
            backRight -> backRightMults
            else -> throw RuntimeException("this should never happem")
        }
    }

    /**
     * Stops all 4 motors
     */
    private fun stop() {
        all.forEach { it.setAndUpdate(0.0) }
    }

    /**
     * Updates all 4 motors with their current power
     */
    private fun updateAll() {
        all.forEach { it.update() }
    }
}