package org.firstinspires.ftc.teamcode

import android.os.Build
import androidx.annotation.RequiresApi
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

@RequiresApi(api = Build.VERSION_CODES.N)
class LiftHandler(map: HardwareMap, private val gamepad: Gamepad?, private val telemetry: Telemetry) {
    private val motor = MotorWrapper["liftMotor", map]
    init {
        motor.motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    @Volatile @JvmField
    var shouldHoldPos = false

    fun reset() {
        val mode = motor.motor.mode
        motor.motor.mode = RunMode.STOP_AND_RESET_ENCODER
        motor.motor.mode = mode
    }

    fun tick() {
        telemetry.addData("lift pos", motor.motor.currentPosition)
        if (gamepad === null || shouldHoldPos) return

        motor.setAndUpdate(gamepad.left_stick_y.toDouble())
        if (gamepad.left_stick_button && gamepad.right_stick_button &&
                gamepad.left_bumper && gamepad.right_bumper) reset()

        if (gamepad.x) pursueTarget(Position.LOW)
    }

    fun pursueTarget(position: Position) {
        pursueTarget(position.pos)
    }

    private fun pursueTarget(pos: Int) {
        // goToPosition will make RunMode RUN_TO_POSITION
        motor.goToPosition(pos, 1.0)
    }

    enum class Position(@JvmField val pos: Int) {
        LOW(10),
        MIDDLE(130),
        HIGH(300);
    }
}