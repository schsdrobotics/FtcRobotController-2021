package org.firstinspires.ftc.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.DistanceSensor;
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
    private final DistanceSensor distanceSensor;
    private Position targetPosition = Position.LOW;
    private Cycle currentCycle = null;
    private Future<Boolean> currentAwaitingResult = null;

    public CycleHandler(SweeperHandler sweeper, BucketHandler bucket, LiftHandler lift,
                        Gamepad controller, DistanceSensor distanceSensor) {
        this.sweeper = sweeper;
        this.bucket = bucket;
        this.lift = lift;
        this.controller = controller;
        this.distanceSensor = distanceSensor;
    }

    public void tick() {
        // first handle current cycle
        if (currentAwaitingResult != null) {
            if (currentAwaitingResult.isDone()) {
                handleCurrentCycleFinishStage();
            }
            // else handle controller inputs
        } else if (controller != null) {
            findTarget();
            handleController();
        }
    }

    private void handleCurrentCycleFinishStage() {
        try {
            boolean success = currentAwaitingResult.get();
            if (!success) {
                controller.rumble(300);
                // todo error handling
            } else {
                if (currentCycle.stage == Cycle.Stage.COMPLETE) {
                    currentCycle = null;
                }
            }
        } catch (ExecutionException | InterruptedException e) {
            e.printStackTrace();
            controller.rumble(300);
            // todo error handling - exception during execution, probably should assume failed
        }
        currentAwaitingResult = null;
    }

    private void handleController() {
        if (controller.a) {
            if (currentCycle != null) {
                currentAwaitingResult = currentCycle.finish();
            } else {
                currentCycle = new Cycle(sweeper, bucket, lift, targetPosition, distanceSensor);
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
