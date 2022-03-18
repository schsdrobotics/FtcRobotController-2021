package org.firstinspires.ftc.teamcode

import android.os.Build
import androidx.annotation.RequiresApi
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@RequiresApi(Build.VERSION_CODES.N)
class CycleHandler(private val sweeper: SweeperHandler, private val bucket: BucketHandler, private val lift: LiftHandler,
                   private val controller: Gamepad, private val distanceSensor: DistanceSensor,
                   private val light: LightHandler, private val telemetry: Telemetry) {
    private var targetPosition = LiftHandler.Position.HIGH
    private var currentCycle: Cycle? = null
    private var cycleData = ""

    fun tick() {
        handleTelemetry()
        // first handle current cycle
        if (currentCycle != null && !currentCycle!!.stage.isBusy) handleCurrentCycleFinishStage()

        // if (controller != null) should always be true
        findTarget()
        handleCycleStatus()
    }

    private fun handleTelemetry() {
        telemetry.addData("Cycle active", currentCycle != null)
        if (currentCycle != null) telemetry.addData("Stage", currentCycle!!.stage)
        telemetry.addData("Target", targetPosition)
        telemetry.addData("Current detected distance (cm)", distanceSensor.getDistance(DistanceUnit.CM))
        if (cycleData.isNotEmpty()) telemetry.addData("Cycle data", cycleData)
    }

    private fun handleCurrentCycleFinishStage() {
        val failed = currentCycle!!.errorMessage.isNotEmpty()
        if (failed) {
            controller.rumble(300)
            cycleData = "Cycle error: " + currentCycle!!.errorMessage
            light.setColor(LightHandler.Color.RED)
        }

        if (currentCycle!!.stage === Cycle.Stage.COMPLETE) {
            currentCycle = null
            light.setColor(LightHandler.Color.OFF)
        } else if (!failed) {
            light.setColor(LightHandler.Color.YELLOW)
            cycleData = ""
        }

        if (cycleData.isEmpty()) cycleData += " | last result success: " + !failed
        telemetry.addData("Last cycle result successful", !failed)
    }

    private fun handleCycleStatus() {
        if (controller.a) {
            if (currentCycle !== null && currentCycle!!.stage === Cycle.Stage.BETWEEN) {
                currentCycle!!.finish()
                light.setColor(LightHandler.Color.GREEN)
            } else {
                currentCycle = Cycle(sweeper, bucket, lift, targetPosition, distanceSensor)
                currentCycle!!.start()
                light.setColor(LightHandler.Color.GREEN)
            }
        }
    }

    private fun findTarget() {
        val x = controller.x
        val y = controller.y
        val b = controller.b
        if (!(x && y) && !(y && b) && !(x && b)) { // if only 1 button is pressed
            targetPosition = when {
                x -> LiftHandler.Position.LOW
                y -> LiftHandler.Position.MIDDLE
                b -> LiftHandler.Position.HIGH
                else -> throw RuntimeException("this should never happen")
            }
        }
    }
}