package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.properties.Delegates

class ArmHandler(map: HardwareMap, private val controller: Gamepad?, private val telemetry: Telemetry) {
    @JvmField val vertical = ServoWrapper[map, "verticalServo"]
    @JvmField val horizontal = ServoWrapper[map, "horizontalServo"]
    @JvmField val mini = ServoWrapper[map, "miniArmServo"]

    private var init = true

    private var startMillis by Delegates.notNull<Long>()
    private var lastMillis = System.currentTimeMillis()

    fun onStartControlled() {
        onStartAuto()
        horizontal.setAndUpdate(0.5)
        startMillis = System.currentTimeMillis()
    }

    fun onStartAuto() {
        vertical.setAndUpdate(0.64)
    }

    fun onStopAuto() {
        vertical.setAndUpdate(0.4) // update this to be whatever the arm is at when the robot first starts
    }

    fun tick() {
        telemetry.addData("ArmHozPos", horizontal.servo.position)
        telemetry.addData("ArmVertPos", vertical.servo.position)
        if (controller === null) return

        // move arm to perfect pickup pos
        if (controller.b) vertical.setAndUpdate(0.09)

        // horizontal
        // Horizontal servo is continuous
        horizontal.pos = when {
            controller.dpad_right -> 0.55
            controller.dpad_left -> 0.45
            else -> 0.5
        }
        horizontal.update()

        // init and vertical
        val millis = System.currentTimeMillis()
        if (millis - lastMillis > 40) {
            if (init) {
                if (millis - startMillis > 2000) init = false
                else horizontal.setAndUpdate(0.4)
            } else {
                vertical.pos += when {
                    controller.dpad_down -> -0.01
                    controller.dpad_up -> 0.01
                    else -> 0.0
                }
            }
            vertical.update()
            lastMillis = millis
        }

        // mini arm
        mini.setAndUpdate(if (controller.a) 0.1 else 1.0)
    }
}