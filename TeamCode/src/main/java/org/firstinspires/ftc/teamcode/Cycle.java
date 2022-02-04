package org.firstinspires.ftc.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.LiftHandler.Position;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.function.Supplier;

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
    private final LightHandler light;
    public volatile Stage stage = Stage.WAITING;
    public volatile String errorMessage = "";

    public Cycle(SweeperHandler sweeper, BucketHandler bucket, LiftHandler lift,
                 Position targetPosition, DistanceSensor distanceSensor, LightHandler light) {
        this.sweeper = sweeper;
        this.bucket = bucket;
        this.lift = lift;
        this.targetPosition = targetPosition;
        this.distanceSensor = distanceSensor;
        this.light = light;
    }

    /**
     * Begin process: intake, lift
     */
    public void start() {
        executor.submit(() -> {
            stage = Stage.IN_START;
            holdValues(true);
            sweeper.forwards(1);

            long startTime = System.currentTimeMillis();
            long lastTime = startTime;
            double distanceCm = 0;
            long bucketFilledFor = 0;
            long runtime = 0;

            // give 2 seconds for object to enter bucket
            while (runtime < 2000) {
                distanceCm = distanceSensor.getDistance(DistanceUnit.CM);
                runtime = System.currentTimeMillis() - startTime;

                if (distanceCm < 5) { // if item in bucket

                    // keep track of how long an item is in the bucket to prevent
                    // stuff bouncing out but still triggering loop exit
                    long deltaMillis = System.currentTimeMillis() - lastTime;
                    bucketFilledFor += deltaMillis;

                } else bucketFilledFor = 0; // reset timer if item has exited

                // if bucket has consistently held an item for 300 millis, consider it secure
                if (bucketFilledFor > 300) break;

                lastTime = System.currentTimeMillis();
                waitFor(20);
            }

            boolean objectPickedUp = distanceCm < 5;
            if (objectPickedUp) {
                bucket.halfway();
                waitFor(300); // give bucket time to rotate
                lift.pursueTarget(targetPosition);
                sweeper.backwards(1); // spit out extras
                waitFor(500);
                sweeper.stop();
            }

            if (!objectPickedUp) {
                holdValues(false);
                stage = Stage.COMPLETE; // finish early to allow for new cycle
                errorMessage = "Failed to pickup object; detected distance: " + distanceCm;
            } else {
                stage = Stage.BETWEEN;
            }
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

            // wiggle bucket to encourage item to drop
            double distanceCm = Double.MIN_VALUE;
            boolean dropped = false;
            for (int i = 0; i < 10; i++) {
                bucket.halfway();
                waitFor(10);
                bucket.forwards();
                waitFor(10);
                distanceCm = distanceSensor.getDistance(DistanceUnit.CM);
                if (distanceCm > 12) {
                    dropped = true;
                    break;
                }
            }

            bucket.backwards();

            lift.pursueTarget(Position.LOW);
            waitFor(targetPosition.pos * 10L);

            holdValues(false);
            stage = Stage.COMPLETE;
            if (!dropped) {
                errorMessage = "Failed to drop item - detected distance: " + distanceCm;
            }
            return dropped;
        });
    }

    /**
     * @return true if COMPLETE, false if BETWEEN or WAITING
     */
    public boolean await() {
        while (stage == Stage.IN_START || stage == Stage.IN_FINISH) {
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
