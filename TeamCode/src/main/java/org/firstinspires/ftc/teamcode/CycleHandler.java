package org.firstinspires.ftc.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.LiftHandler.Position;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;

@RequiresApi(api = Build.VERSION_CODES.N)
public class CycleHandler {
    private final Telemetry telemetry;
    private final SweeperHandler sweeper;
    private final BucketHandler bucket;
    private final LiftHandler lift;
    private final Gamepad controller;
    private final DistanceSensor distanceSensor;
    private Position targetPosition = Position.LOW;
    private Cycle currentCycle = null;
    private Future<Boolean> currentAwaitingResult = null;

    public CycleHandler(SweeperHandler sweeper, BucketHandler bucket, LiftHandler lift,
                        Gamepad controller, DistanceSensor distanceSensor, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.sweeper = sweeper;
        this.bucket = bucket;
        this.lift = lift;
        this.controller = controller;
        this.distanceSensor = distanceSensor;
    }

    public void tick() {
        handleTelemetry();
        // first handle current cycle
        if (currentAwaitingResult != null) {
            if (currentAwaitingResult.isDone()) {
                handleCurrentCycleFinishStage();
            }
            // else handle controller inputs
        } else if (controller != null) {
            findTarget();
            handleCycleStatus();
        }
    }

    private void handleTelemetry() {
        telemetry.addData("Cycle active: ", currentCycle != null);
        telemetry.addData("Awaiting result: ", currentAwaitingResult != null);
        telemetry.addData("Target: ", targetPosition);
        telemetry.addData("Current detected distance (cm): ", distanceSensor.getDistance(DistanceUnit.CM));
    }

    private void handleCurrentCycleFinishStage() {
        try {
            boolean failed = !currentAwaitingResult.get();
            if (failed) {
                controller.rumble(300);
                telemetry.addData("Cycle error: ", currentCycle.errorMessage);
            }

            if (currentCycle.stage == Cycle.Stage.COMPLETE) {
                currentCycle = null;
            }
            telemetry.addData("Last cycle result successful: ", !failed);
        } catch (ExecutionException | InterruptedException e) {
            controller.rumble(300);
            telemetry.addData("Cycle exception!", e);
            e.printStackTrace();
        }
        currentAwaitingResult = null;
    }

    private void handleCycleStatus() {
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
        if ((x || y || b) && (!(x && y) && !(y && b) && !(x && b))) { // if only 1 button is pressed
            if (x) {
                targetPosition = Position.LOW;
            } else if (y) {
                targetPosition = Position.MIDDLE;
            } else {
                targetPosition = Position.HIGH;
            }
        }
    }
}
