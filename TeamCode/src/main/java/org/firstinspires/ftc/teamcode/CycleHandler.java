package org.firstinspires.ftc.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.LiftHandler.Position;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;

@RequiresApi(api = Build.VERSION_CODES.N)
public class CycleHandler {
    private final SweeperHandler sweeper;
    private final BucketHandler bucket;
    private final LiftHandler lift;
    private final Gamepad controller;
    private Position targetPosition = null;
    private Cycle currentCycle = null;
    private Future<Boolean> currentAwaitingResult = null;

    public CycleHandler(SweeperHandler sweeper, BucketHandler bucket, LiftHandler lift,
                        Gamepad controller) {
        this.sweeper = sweeper;
        this.bucket = bucket;
        this.lift = lift;
        this.controller = controller;
    }

    public void tick() {
        // first handle current cycle
        if (currentAwaitingResult != null) {
            if (currentAwaitingResult.isDone()) {
                try {
                    boolean out = currentAwaitingResult.get();
                    if (!out) {
                        // todo error handling - did not get an object
                    }
                } catch (ExecutionException | InterruptedException e) {
                    e.printStackTrace();
                    // todo error handling - exception during execution, probably should assume failed to pickup
                }
                currentCycle = null;
                currentAwaitingResult = null;
            }
        } else if (controller != null) {
            findTarget();

            if (controller.a && targetPosition != null) {
                if (currentCycle != null) {

                }
                currentCycle = new Cycle(sweeper, bucket, lift, targetPosition);
                currentAwaitingResult = currentCycle.start();
            }
        }
    }

    private void findTarget() {
        boolean x = controller.x;
        boolean y = controller.y;
        boolean b = controller.b;
        if ((!(x && y) && !(y && b) && !(x && b))) { // if only 1 button is pressed and motor stopped
            if (x) {
                targetPosition = Position.LOW;
            } else if (y) {
                targetPosition = Position.MIDDLE;
            } else if (b) {
                targetPosition = Position.HIGH;
            } else {
                throw new IllegalArgumentException("Impossible target");
            }
        }
    }
}
