package org.firstinspires.ftc.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.Supplier;

@RequiresApi(api = Build.VERSION_CODES.N)
public class BucketHandler {
    public final ServoWrapper servo;
    private final Gamepad controller;
    public volatile boolean shouldHoldPos = false;

    public BucketHandler(HardwareMap map, Gamepad controller) {
        this.controller = controller;
        servo = ServoWrapper.get(map, "bucketServo");
    }

    public void tick() {
        if (controller != null && !shouldHoldPos) {
            System.out.println("tick");
            if (controller.right_trigger <= 0.1) backwards();
            else {
                if (controller.left_trigger <= 0.1) halfway();
                else forwards();
            }
        }
        servo.update();
    }

    public void forwards() {
        servo.setAndUpdate(0);
        System.out.println("forwards");
    }

    public void backwards() {
        servo.setAndUpdate(0.9);
        System.out.println("backwards");
    }

    public void halfway() {
        System.out.println("halfway");
        servo.setAndUpdate(0.5);
    }

    /**
     * This method is blocking
     */
    public void wiggleUntil(Supplier<Boolean> test) {
        System.out.println("wiggling");
        for (int i = 0; i < 15; i++) {
            halfway();
            waitFor(25);
            forwards();
            waitFor(25);
            if (test.get()) return;
        }
    }

    private void waitFor(long millis) {
        long endTime = System.currentTimeMillis() + millis;
        while (System.currentTimeMillis() < endTime);
    }
}