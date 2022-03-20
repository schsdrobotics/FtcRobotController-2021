package org.firstinspires.ftc.teamcode

import android.os.Build
import androidx.annotation.RequiresApi
import com.qualcomm.robotcore.hardware.DistanceSensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean

// FIXME This code is probably trash
/**
 * Represents a cycle of:
 * <br>- pickup
 * <br>- drop extra objects
 * <br>- tilt bucket
 * <br>- lift... lift
 * <br>- drop item
 * <br>- retract lift
 * @see CycleHandler
 */
@RequiresApi(api = Build.VERSION_CODES.N)
class Cycle(private val sweeper: SweeperHandler, private val bucket: BucketHandler, private val lift: LiftHandler,
            private val targetPosition: LiftHandler.Position, private val distanceSensor: DistanceSensor) {
    companion object {
        private val executor: ExecutorService = Executors.newSingleThreadExecutor()
    }

    @Volatile @JvmField
    var stage = Stage.WAITING

    @Volatile @JvmField
    var errorMessage = ""

    val isBusy: Boolean
        get() = stage.isBusy

    val softIsBusy: Boolean
        get() = stage.softIsBusy

    val isLowering: Boolean
        get() = stage.isLowering

    val shouldCancel: Boolean
        get() = stage.shouldCancel

    /**
     * @return true if COMPLETE, false if BETWEEN or WAITING
     */
    fun await(): Boolean {
        waitFor(100) // Make sure that the other thread has a chance to set the state
        while (softIsBusy && !isLowering && !shouldCancel) {
            waitFor(20)
        }
        return stage == Stage.COMPLETE
    }

    private fun waitFor(millis: Long) {
        val endTime = System.currentTimeMillis() + millis
        while (System.currentTimeMillis() < endTime);
    }

    private fun holdValues(value: Boolean) {
        lift.shouldHoldPos = value
        bucket.shouldHoldPos = value
        sweeper.shouldHoldSpeed = value
    }

    /**
     * Begin process: intake, lift
     */
    fun start() {
        stage = Stage.IN_START
        executor.submit {
            holdValues(true)
            var distanceCm = distanceSensor.getDistance(DistanceUnit.CM)
            val preFilled = distanceCm < 9
            if (!preFilled) {
                sweeper.forwards(1.0)
                val startTime = System.currentTimeMillis()
                var lastTime = startTime
                var bucketFilledFor = 0L
                var runtime = 0L

                // give 5 seconds for object to enter bucket
                while (runtime < 5000) {
                    distanceCm = distanceSensor.getDistance(DistanceUnit.CM)
                    runtime = System.currentTimeMillis() - startTime

                    if (distanceCm < 9) { // if item in bucket
                        stage = Stage.SHOULD_CANCEL
                        // keep track of how long an item is in the bucket to prevent
                        // stuff bouncing out but still triggering loop exit
                        val deltaMillis = System.currentTimeMillis() - lastTime
                        bucketFilledFor += deltaMillis
                    } else bucketFilledFor = 0 // reset timer if item has exited

                    // if bucket has consistently held an item for 400 millis, consider it secure
                    if (bucketFilledFor > 700) {
                        sweeper.stop()
                        break
                    }

                    lastTime = System.currentTimeMillis()
                    waitFor(20)
                }
            }
            val objectPickedUp = distanceCm < 9
            if (objectPickedUp) {
                if (!preFilled) {
                    bucket.halfway()
                    sweeper.stop()
                    waitFor(200)
                    sweeper.backwards(1.0) // spit out extras
                    waitFor(300)
                }
                bucket.halfway()
                waitFor(100) // give bucket time to rotate
                lift.pursueTarget(targetPosition)
                stage = Stage.BETWEEN
            } else {
                holdValues(false)
                stage = Stage.COMPLETE // finish early to allow for new cycle
                errorMessage = "Failed to pick up object; detected distance: $distanceCm"
            }
            waitFor(600)
            sweeper.stop()
            sweeper.shouldHoldSpeed = false
            objectPickedUp
        }
    }

    /**
     * finish process: dump, retract
     */
    fun finish() { // FIXME There is a VERY high chance that this code sucks.
        executor.submit {
            stage = Stage.IN_FINISH
            errorMessage = ""
            bucket.forwards()
            waitFor(350)

            // wiggle bucket to encourage item to drop
            var distanceCm = Double.MIN_VALUE
            val dropped = AtomicBoolean(false)
            bucket.wiggleUntil {
                distanceCm = distanceSensor.getDistance(DistanceUnit.CM)
                val shouldStop = distanceCm > 12
                if (shouldStop) dropped.set(true)
                shouldStop
            }
            bucket.backwards()
            lift.pursueTarget(LiftHandler.Position.LOW)
            stage = Stage.LOWERING_LIFT
            waitFor(targetPosition.pos * 5L)
            holdValues(false)
            stage = Stage.COMPLETE
            if (!dropped.get()) errorMessage = "Failed to drop item - detected distance: $distanceCm"
            dropped.get()
        }
    }

    enum class Stage {
        WAITING, IN_START, SHOULD_CANCEL, BETWEEN, IN_FINISH, LOWERING_LIFT, COMPLETE;

        val isBusy: Boolean
            get() = this == IN_START || this == IN_FINISH || this == SHOULD_CANCEL || this == LOWERING_LIFT

        val softIsBusy: Boolean
            get() = this == IN_START || this == IN_FINISH

        val isLowering: Boolean
            get() = this == LOWERING_LIFT

        val shouldCancel: Boolean
            get() = this == SHOULD_CANCEL
    }
}