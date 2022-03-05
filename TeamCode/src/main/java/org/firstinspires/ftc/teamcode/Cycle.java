package org.firstinspires.ftc.teamcode;

import android.os.Build;
import android.util.MutableDouble;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.LiftHandler.Position;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

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
    private final DistanceSensor distanceSensor;
    public volatile Stage stage = Stage.WAITING;
    public volatile String errorMessage = "";

    public Cycle(SweeperHandler sweeper, BucketHandler bucket, LiftHandler lift,
                 Position targetPosition, DistanceSensor distanceSensor) {
        this.sweeper = sweeper;
        this.bucket = bucket;
        this.lift = lift;
        this.targetPosition = targetPosition;
        this.distanceSensor = distanceSensor;
    }

    /**
     * Begin process: intake, lift
     */
    public void start() {
        stage = Stage.IN_START;
        executor.submit(() -> {
            holdValues(true);
            double distanceCm = distanceSensor.getDistance(DistanceUnit.CM);
            boolean preFilled = distanceCm < 9;
            if (!preFilled) {
                sweeper.forwards(1);

                long startTime = System.currentTimeMillis();
                long lastTime = startTime;
                long bucketFilledFor = 0;
                long runtime = 0;

                // give 5 seconds for object to enter bucket
                while (runtime < 5000) {
                    distanceCm = distanceSensor.getDistance(DistanceUnit.CM);
                    runtime = System.currentTimeMillis() - startTime;

                    if (distanceCm < 9) { // if item in bucket
                        stage = Stage.SHOULD_CANCEL;
                        // keep track of how long an item is in the bucket to prevent
                        // stuff bouncing out but still triggering loop exit
                        long deltaMillis = System.currentTimeMillis() - lastTime;
                        bucketFilledFor += deltaMillis;

                    } else bucketFilledFor = 0; // reset timer if item has exited

                    // if bucket has consistently held an item for 400 millis, consider it secure
                    if (bucketFilledFor > 400) {
                        sweeper.stop();
                        break;
                    }

                    lastTime = System.currentTimeMillis();
                    waitFor(20);
                }
            }

            boolean objectPickedUp = distanceCm < 9;
            if (objectPickedUp) {
                if (!preFilled) {
                    bucket.halfway();
                    waitFor(100);
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
                errorMessage = "Failed to pick up object; detected distance: " + distanceCm;
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
            MutableDouble distanceCm = new MutableDouble(Double.MIN_VALUE);
            AtomicBoolean dropped = new AtomicBoolean(false);
            bucket.wiggleUntil(() -> {
                distanceCm.value = distanceSensor.getDistance(DistanceUnit.CM);
                boolean shouldStop = distanceCm.value > 12;
                if (shouldStop) {
                    dropped.set(true);
                }
                return shouldStop;
            });

            bucket.backwards();

            lift.pursueTarget(Position.LOW);
            stage = Stage.LOWERING_LIFT;
            waitFor(targetPosition.pos * 5L);

            holdValues(false);
            stage = Stage.COMPLETE;
            if (!dropped.get()) {
                errorMessage = "Failed to drop item - detected distance: " + distanceCm.value;
            }
            return dropped.get();
        });
    }

    public boolean isBusy() {
        return stage.isBusy();
    }

    public boolean softIsBusy() {
        return stage.softIsBusy();
    }

    public boolean isLowering() {
        return stage.isLowering();
    }

    public boolean shouldCancel() {
        return stage.shouldCancel();
    }

    /**
     * @return true if COMPLETE, false if BETWEEN or WAITING
     */
    public boolean await() {
        waitFor(100); // Make sure that the other thread has a chance to set the state
        while (softIsBusy() && !isLowering() && !shouldCancel()) {
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
        SHOULD_CANCEL,
        BETWEEN,
        IN_FINISH,
        LOWERING_LIFT,
        COMPLETE;

        public boolean isBusy() {
            return this == IN_START || this == IN_FINISH || this == SHOULD_CANCEL || this == LOWERING_LIFT;
        }

        public boolean softIsBusy() {
            return this == IN_START || this == IN_FINISH;
        }

        public boolean isLowering() {
            return this == LOWERING_LIFT;
        }

        public boolean shouldCancel() {
            return this == SHOULD_CANCEL;
        }
    }
}