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
    private final LightHandler light;
    private final Gamepad controller;
    private final DistanceSensor distanceSensor;
    private Position targetPosition = Position.HIGH;
    private Cycle currentCycle = null;
    private String cycleData = "";

    public CycleHandler(SweeperHandler sweeper, BucketHandler bucket, LiftHandler lift,
                        Gamepad controller, DistanceSensor distanceSensor,
                        LightHandler light, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.sweeper = sweeper;
        this.bucket = bucket;
        this.lift = lift;
        this.controller = controller;
        this.distanceSensor = distanceSensor;
        this.light = light;
    }

    public void tick() {
        handleTelemetry();
        // first handle current cycle
        if (currentCycle != null) {
            if (!currentCycle.stage.isBusy()) {
                handleCurrentCycleFinishStage();
            }
        }

        if (controller != null) {
            findTarget();
            handleCycleStatus();
        }
    }

    private void handleTelemetry() {
        telemetry.addData("Cycle active", currentCycle != null);
        if (currentCycle != null) {
            telemetry.addData("Stage", currentCycle.stage);
        }
        telemetry.addData("Target", targetPosition);
        telemetry.addData("Current detected distance (cm)", distanceSensor.getDistance(DistanceUnit.CM));
        if (!cycleData.isEmpty()) {
            telemetry.addData("Cycle data", cycleData);
        }
    }

    private void handleCurrentCycleFinishStage() {
        boolean failed = !currentCycle.errorMessage.isEmpty();
        if (failed) {
            controller.rumble(300);
            cycleData = "Cycle error: " + currentCycle.errorMessage;
            light.setColor(LightHandler.Color.RED);
        }

        if (currentCycle.stage == Cycle.Stage.COMPLETE) {
            currentCycle = null;
            light.setColor(LightHandler.Color.OFF);
        } else if (!failed) {
            light.setColor(LightHandler.Color.YELLOW);
            cycleData = "";
        }
        if (cycleData.isEmpty()) cycleData += " | last result success: " + !failed;
        telemetry.addData("Last cycle result successful", !failed);
    }

    private void handleCycleStatus() {
        if (controller.a) {
            if (currentCycle != null && currentCycle.stage == Cycle.Stage.BETWEEN) {
                currentCycle.finish();
                light.setColor(LightHandler.Color.GREEN);
            } else if (currentCycle == null) {
                currentCycle = new Cycle(sweeper, bucket, lift, targetPosition, distanceSensor);
                currentCycle.start();
                light.setColor(LightHandler.Color.GREEN);
            }
        }
    }

    private void findTarget() {
        boolean x = controller.x;
        boolean y = controller.y;
        boolean b = controller.b;
        if ((!(x && y) && !(y && b) && !(x && b))) { // if only 1 button is pressed
            if (x) {
                targetPosition = Position.LOW;
            } else if (y) {
                targetPosition = Position.MIDDLE;
            } else if (b) {
                targetPosition = Position.HIGH;
            }
        }
    }
}