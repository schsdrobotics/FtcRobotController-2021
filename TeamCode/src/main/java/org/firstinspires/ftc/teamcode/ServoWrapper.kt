package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class ServoWrapper(@JvmField val servo: Servo) {
    companion object {
        const val LIMIT = 95
        operator fun get(map: HardwareMap, name: String): ServoWrapper {
            return ServoWrapper(map[Servo::class.java, name])
        }
    }

    var pos = 0.0
        set(value) {
            val maxMin = LIMIT / 100.0
            field = if (value > maxMin) maxMin else if (value < 1 - maxMin) 1 - maxMin else value
        }

    fun update() {
        servo.position = pos
    }

    fun setAndUpdate(pos: Double) {
        this.pos = pos
        update()
    }
}