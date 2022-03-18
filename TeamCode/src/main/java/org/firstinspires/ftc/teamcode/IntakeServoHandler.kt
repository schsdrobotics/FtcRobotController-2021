package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.HardwareMap

class IntakeServoHandler(map: HardwareMap) {
    companion object {
        const val HOOKED = 0.25
        const val RELEASED = 0.5
    }

    @JvmField
    val servo = ServoWrapper[map, "intakeServo"]

    fun goToPos(pos: Double) {
        servo.pos = when {
            pos < HOOKED -> HOOKED
            pos > RELEASED -> RELEASED
            else -> pos
        }
        servo.update()
    }

    fun hook() {
        goToPos(HOOKED)
    }

    fun release() {
        goToPos(RELEASED)
    }
}