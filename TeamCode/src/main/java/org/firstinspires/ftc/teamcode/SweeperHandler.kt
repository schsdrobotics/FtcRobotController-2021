package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap

class SweeperHandler(map: HardwareMap, private val controller: Gamepad?) {
    private val motor: MotorWrapper = MotorWrapper["sweeperMotor", map]
    @Volatile @JvmField
    var shouldHoldSpeed = false

    fun tick() {
        if (controller === null || shouldHoldSpeed) return
        val rt = controller.right_trigger
        val lt = controller.left_trigger
        when {
            rt > 0 -> forwards(rt.toDouble())
            lt > 0 -> backwards(lt.toDouble())
            else -> stop()
        }
    }

    fun forwards(power: Double) {
        motor.setAndUpdate(power)
    }

    fun backwards(power: Double) {
        motor.setAndUpdate(-power)
    }

    fun stop() {
        motor.setAndUpdate(0.0)
    }
}