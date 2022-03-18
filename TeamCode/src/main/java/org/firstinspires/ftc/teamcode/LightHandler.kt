package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap

class LightHandler(map: HardwareMap) {
    companion object {
        const val UNIT = 600L
    }

    private val red = map[DigitalChannel::class.java, "red"]
    private val green = map[DigitalChannel::class.java, "green"]
    init {
        red.mode = DigitalChannel.Mode.OUTPUT
        green.mode = DigitalChannel.Mode.OUTPUT
    }

    private var startTime = System.currentTimeMillis()
    private val instructions: MutableList<State> = ArrayList()
    private var currentIndex = 0

    fun dot(): LightHandler {
        instructions.add(State.DOT)
        return this
    }

    fun dash(): LightHandler {
        instructions.add(State.DASH)
        return this
    }

    fun pause(): LightHandler {
        instructions.add(State.PAUSE)
        return this
    }

    fun resetTimer() {
        startTime = System.currentTimeMillis()
    }

    fun tick() {
        when {
            System.currentTimeMillis() - startTime < instructions[currentIndex].on -> setColor(Color.GREEN)
            System.currentTimeMillis() - startTime < instructions[currentIndex].off -> setColor(Color.OFF)
            currentIndex < instructions.size - 1 -> {
                currentIndex++
                resetTimer()
            }
            else -> setColor(Color.OFF)
        }
    }

    fun setColor(color: Color) {
        red.state = color.red
        green.state = color.green
    }

    enum class State(@JvmField val on: Long, @JvmField val off: Long) {
        DOT(UNIT, UNIT *2),
        DASH(UNIT *3, UNIT *4),
        PAUSE(0, UNIT *2);
    }

    enum class Color(@JvmField val red: Boolean, @JvmField val green: Boolean) {
        OFF(false, false),
        RED(true, false),
        YELLOW(true, true),
        GREEN(false, true);
    }
}