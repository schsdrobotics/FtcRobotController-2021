package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap

/**
 * A wrapper around a motor that allows for delaying updating until later
 */
class MotorWrapper(@JvmField val motor: DcMotor) {
    companion object {
        const val PERCENT_OF_MAX = 80
        fun clampPower(original: Double): Double {
            val maxMin = PERCENT_OF_MAX / 100.0
            return if (original > maxMin) maxMin else if (original < -maxMin) -maxMin else original
        }
        operator fun get(name: String, map: HardwareMap): MotorWrapper {
            return MotorWrapper(map[DcMotor::class.java, name])
        }
    }

    var power = 0.0
        set(value) {
            field = clampPower(value)
        }

    fun update() {
        motor.power = power
    }

    fun setAndUpdate(power: Double) {
        this.power = power
        update()
    }

    fun goToPosition(pos: Int, power: Double) {
        val clampedPower = clampPower(power)
        setAndUpdate(clampedPower)
        motor.targetPosition = pos
        motor.mode = DcMotor.RunMode.RUN_TO_POSITION
        // conflicting javadocs - just in case
        setAndUpdate(clampedPower)
        motor.targetPosition = pos
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
}