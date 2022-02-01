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
    public volatile Stage stage = Stage.WAITING;
    public volatile String errorMessage = "error setting error message???";

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
     * @return Future holding a boolean - true if successfully stored an object
     */
    public Future<Boolean> start() {
        return executor.submit(() -> {
            stage = Stage.IN_START;
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
                waitFor(500); // give bucket time to rotate // TODO adjust bucket to only hold 1 item
                lift.pursueTarget(targetPosition.pos);
                sweeper.backwards(1); // spit out extras
            }
            stage = Stage.BETWEEN;
            if (!objectPickedUp) errorMessage = "Failed to pickup object; detected distance: " + distanceCm;
            return objectPickedUp;
        });
    }

    /**
     * finish process: dump, retract
     * @return Future holding a boolean - true if successfully dropped an object
     */
    public Future<Boolean> finish() {
        return executor.submit(() -> {
            stage = Stage.IN_FINISH;
            bucket.forwards();

            double distanceCm = distanceSensor.getDistance(DistanceUnit.CM);
            long startTime = System.currentTimeMillis();

            // give 1 second to drop
            while (distanceCm < 5 && System.currentTimeMillis() - startTime < 1000) {
                waitFor(20);
                // shake out items
                for (int i = 0; i < 5; i++) {
                    bucket.halfway();
                    waitFor(10);
                    bucket.forwards();
                    waitFor(10);
                }
                distanceCm = distanceSensor.getDistance(DistanceUnit.CM);
            }

            boolean dropped = distanceCm > 7;
            if (dropped) {
                bucket.backwards();
            }

            lift.pursueTarget(Position.LOW);
            while (lift.isBusy()) {
                waitFor(20);
            }

            stage = Stage.COMPLETE;
            if (!dropped) errorMessage = "failed to drop object; detected distance: " + distanceCm;
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
        try {
            wait(millis);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public enum Stage {
        WAITING,
        IN_START,
        BETWEEN,
        IN_FINISH,
        COMPLETE
    }
}
