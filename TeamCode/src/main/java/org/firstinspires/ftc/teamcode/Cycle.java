package org.firstinspires.ftc.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

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
 * - pickup
 * - drop extra objects
 * - tilt bucket
 * - lift... lift
 * - drop item
 * - retract lift
 */
@RequiresApi(api = Build.VERSION_CODES.N)
public class Cycle {
    private static final ExecutorService executor = Executors.newSingleThreadExecutor();
    private final SweeperHandler sweeper;
    private final BucketHandler bucket;
    private final LiftHandler lift;
    private final Position targetPosition;
    private volatile boolean busy = false;

    public Cycle(SweeperHandler sweeper, BucketHandler bucket, LiftHandler lift, // todo add object sensor
                 Position targetPosition) {
        this.sweeper = sweeper;
        this.bucket = bucket;
        this.lift = lift;
        this.targetPosition = targetPosition;
    }

    /**
     * Begin process: intake, lift
     * @return Future holding a boolean - true if successfully stored an object
     */
    public Future<Boolean> start() {
        return executor.submit(() -> {
            busy = true;
            // todo pickup - object sensor, timeout backup
            boolean objectPickedUp = false;
            if (objectPickedUp) {
                bucket.halfway();
                waitFor(500); // give bucket time to rotate
                lift.pursueTarget(targetPosition.pos);
            }
            busy = false;
            return objectPickedUp;
        });
    }

    /**
     * finish process: dump, retract
     * @return Future holding a boolean - true if successfully dropped an object
     */
    public Future<Boolean> finish() {
        return executor.submit(() -> {
            busy = true;
            bucket.forwards();
            // todo object sensor - make sure it's dropped
            boolean dropped = true;
            waitFor(1000);
            bucket.backwards();
            waitFor(500);
            lift.pursueTarget(Position.LOW);
            busy = false;
            return dropped;
        });
    }

    public void await() {
        while (busy) {
            waitFor(20);
        }
    }

    private void waitFor(long millis) {
        try {
            wait(millis);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
