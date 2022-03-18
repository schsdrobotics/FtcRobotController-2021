package org.firstinspires.ftc.teamcode

import android.os.Build
import androidx.annotation.RequiresApi
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import java.util.function.Supplier

@RequiresApi(api = Build.VERSION_CODES.N)
class BucketHandler(map: HardwareMap, private val controller: Gamepad?) {
    private val servo = ServoWrapper[map, "bucketServo"]

    @Volatile @JvmField
    var shouldHoldPos = false

    fun tick() {
        if (controller === null || shouldHoldPos) return
        when {
            controller.right_trigger <= 0.1 -> backwards()
            controller.left_trigger <= 0.1 -> halfway()
            else -> forwards()
        }
    }

    fun forwards() {
        servo.setAndUpdate(0.2)
    }

    fun backwards() {
        servo.setAndUpdate(0.85)
    }

    fun halfway() {
        servo.setAndUpdate(0.5)
    }

    /**
     * This method is blocking
     */
    fun wiggleUntil(test: Supplier<Boolean>) {
        for (i in 0..29) {
            halfway()
            waitFor(100)
            forwards()
            waitFor(100)
            if (test.get()) return
        }
    }

    private fun waitFor(millis: Long) {
        val endTime = System.currentTimeMillis() + millis
        while (System.currentTimeMillis() < endTime);
    }
}