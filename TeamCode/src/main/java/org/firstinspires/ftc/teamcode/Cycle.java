package org.firstinspires.ftc.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import org.firstinspires.ftc.teamcode.LiftHandler.Position;

import java.util.Arrays;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;

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
public class Cycle {
    private static final ExecutorService executor = Executors.newSingleThreadExecutor();
    private final SweeperHandler sweeper;
    private final BucketHandler bucket;
    private final LiftHandler lift;
    private final Position targetPosition;
    private final ColorSensorHandler colorSensor;
    public volatile Stage stage = Stage.WAITING;
    public volatile String errorMessage = "";

    public Cycle(SweeperHandler sweeper, BucketHandler bucket, LiftHandler lift,
                 Position targetPosition, ColorSensorHandler colorSensor) {
        this.sweeper = sweeper;
        this.bucket = bucket;
        this.lift = lift;
        this.targetPosition = targetPosition;
        this.colorSensor = colorSensor;
    }

    /**
     * Begin process: intake, lift
     */
    public void start() {
        executor.submit(() -> {
            stage = Stage.IN_START;
            holdValues(true);
            int[] RGB = colorSensor.getRGBValues();
            boolean preFilled = colorSensor.testForFreight(RGB);
            if (!preFilled) {
                sweeper.forwards(1);

                long startTime = System.currentTimeMillis();
                long lastTime = startTime;
                long bucketFilledFor = 0;
                long runtime = 0;

                // give 4 seconds for object to enter bucket
                while (runtime < 4000) {
                    RGB = colorSensor.getRGBValues();
                    runtime = System.currentTimeMillis() - startTime;

                    if (colorSensor.testForFreight(RGB)) { // if item in bucket

                        // keep track of how long an item is in the bucket to prevent
                        // stuff bouncing out but still triggering loop exit
                        long deltaMillis = System.currentTimeMillis() - lastTime;
                        bucketFilledFor += deltaMillis;

                    } else bucketFilledFor = 0; // reset timer if item has exited

                    // if bucket has consistently held an item for 300 millis, consider it secure
                    if (bucketFilledFor > 300) {
                        sweeper.stop();
                        break;
                    }

                    lastTime = System.currentTimeMillis();
                    waitFor(20);
                }
            }

            boolean objectPickedUp = colorSensor.testForFreight(RGB);
            if (objectPickedUp) {
                if (!preFilled) {
                    sweeper.backwards(1); // spit out extras
                    waitFor(300);
                }
                bucket.halfway();
                waitFor(100); // give bucket time to rotate
                lift.pursueTarget(targetPosition);
                stage = Stage.BETWEEN;
            } else {
                holdValues(false);
                stage = Stage.COMPLETE; // finish early to allow for new cycle
                errorMessage = "Failed to pick up object; detected RGB: " + Arrays.toString(RGB);
            }
            waitFor(600);
            sweeper.stop();
            sweeper.shouldHoldSpeed = false;
            return objectPickedUp;
        });
    }

    /**
     * finish process: dump, retract
     */
    public void finish() {
        executor.submit(() -> {
            stage = Stage.IN_FINISH;
            errorMessage = "";
            bucket.forwards();
            waitFor(350);

            // wiggle bucket to encourage item to drop
            int[] RGB = new int[]{0, 0, 0};
            AtomicBoolean dropped = new AtomicBoolean(false);
            bucket.wiggleUntil(() -> {
                int[] current = colorSensor.getRGBValues();
                RGB[0] = current[0];
                RGB[1] = current[1];
                RGB[2] = current[2];
                boolean shouldStop = colorSensor.testForFreight(current);
                if (shouldStop) {
                    dropped.set(true);
                }
                return shouldStop;
            });

            bucket.backwards();

            lift.pursueTarget(Position.LOW);
            waitFor(targetPosition.pos * 5L);

            holdValues(false);
            stage = Stage.COMPLETE;
            if (!dropped.get()) {
                errorMessage = "Failed to drop item - detected RGB: " + Arrays.toString(RGB);
            }
            return dropped.get();
        });
    }

    public boolean isBusy() {
        return stage.isBusy();
    }

    /**
     * @return true if COMPLETE, false if BETWEEN or WAITING
     */
    public boolean await() {
        waitFor(100); // Make sure that the other thread has a chance to set the state
        while (isBusy()) {
            waitFor(20);
        }
        return stage == Stage.COMPLETE;
    }

    private void waitFor(long millis) {
        long endTime = System.currentTimeMillis() + millis;
        while (System.currentTimeMillis() < endTime);
    }

    private void holdValues(boolean value) {
        lift.shouldHoldPos = value;
        bucket.shouldHoldPos = value;
        sweeper.shouldHoldSpeed = value;
    }

    public enum Stage {
        WAITING,
        IN_START,
        BETWEEN,
        IN_FINISH,
        COMPLETE;

        public boolean isBusy() {
            return this == IN_START || this == IN_FINISH;
        }
    }
}
