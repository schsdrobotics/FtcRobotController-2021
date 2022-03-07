package org.firstinspires.ftc.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.Supplier;

@RequiresApi(api = Build.VERSION_CODES.N)
public class BucketHandler {
    private final ServoWrapper servo;
    private final Gamepad controller;
    public volatile boolean shouldHoldPos = false;

    public BucketHandler(HardwareMap map, Gamepad controller) {
        this.controller = controller;
        servo = ServoWrapper.get(map, "bucketServo");
    }

     public void tick() {
        if (controller != null && !shouldHoldPos) {
            if (controller.right_trigger <= 0.1) backwards();
            else {
                if (controller.left_trigger <= 0.1) halfway();
                else forwards();
            }
        }
     }

     public void forwards() {
         servo.setAndUpdate(0.2);
     }

     public void backwards() {
        servo.setAndUpdate(0.85);
     }

     public void halfway() {
        servo.setAndUpdate(0.5);
     }

    /**
     * This method is blocking
     */
    public void wiggleUntil(Supplier<Boolean> test) {
         for (int i = 0; i < 30; i++) {
             halfway();
             waitFor(100);
             forwards();
             waitFor(100);
             if (test.get()) return;
         }
     }

    private void waitFor(long millis) {
        long endTime = System.currentTimeMillis() + millis;
        while (System.currentTimeMillis() < endTime);
    }
}