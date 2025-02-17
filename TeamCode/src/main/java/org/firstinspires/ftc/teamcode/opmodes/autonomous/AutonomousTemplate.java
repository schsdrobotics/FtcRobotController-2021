package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ArmHandler;
import org.firstinspires.ftc.teamcode.BucketHandler;
import org.firstinspires.ftc.teamcode.CameraHandler;
import org.firstinspires.ftc.teamcode.Cycle;
import org.firstinspires.ftc.teamcode.DuckHandler;
import org.firstinspires.ftc.teamcode.IntakeServoHandler;
import org.firstinspires.ftc.teamcode.LiftHandler;
import org.firstinspires.ftc.teamcode.LightHandler;
import org.firstinspires.ftc.teamcode.SweeperHandler;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * A template containing shared code used in all autonomous programs.
 */
@RequiresApi(api = Build.VERSION_CODES.N)
public abstract class AutonomousTemplate extends LinearOpMode {
    private float xCenter;
    protected LiftHandler.Position target = LiftHandler.Position.HIGH;

    protected int multiplier() {
        return 1; // 1 is for red; -1 is for blue. Multiply all headings and y-coordinates (NOT x-coordinates) by this.
    }

    // Declare all hardware-related fields
    protected CameraHandler camera;
    protected SampleMecanumDrive drive;
    protected DuckHandler duck;
    protected ArmHandler arm;
    protected LiftHandler lift;
    protected BucketHandler bucket;
    protected SweeperHandler sweeper;
    protected IntakeServoHandler intakeServo;
    protected LightHandler light;
    protected DistanceSensor distanceSensor;
    private DistanceSensor xCoordinateSensor;
    protected Cycle currentCycle;

    protected static final ExecutorService executor = Executors.newSingleThreadExecutor();

    protected abstract Pose2d startPose();

    protected String cameraName() {
        return "intakeCamera";
    }

    @Override
    public void runOpMode() {
        // Initialize hardware
        camera = new CameraHandler(hardwareMap, cameraName());
        drive = new SampleMecanumDrive(hardwareMap);
        duck = new DuckHandler(hardwareMap, null);
        arm = new ArmHandler(hardwareMap, null, telemetry);
        lift = new LiftHandler(hardwareMap, null, telemetry);
        bucket = new BucketHandler(hardwareMap, null);
        sweeper = new SweeperHandler(hardwareMap, null);
        intakeServo = new IntakeServoHandler(hardwareMap);
        light = new LightHandler(hardwareMap);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        xCoordinateSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        light.setColor(LightHandler.Color.YELLOW);
        telemetry.addData("Status", "Initialized");

        // Assume intakeServo is close to up position
        intakeServo.hook();

        drive.setPoseEstimate(startPose());

        // light and drive.update()
        backgroundLoop();

        initializeTrajectories();

        setup();

        // Raise arm + lift bucket halfway
        currentCycle = new Cycle(sweeper, bucket, lift, target, distanceSensor);
        currentCycle.start();

        resetStartTime();
        main();
    }

    public static double rad(double deg) {
        return Math.toRadians(deg);
    }

    public static Vector2d pos(double x, double y) {
        return new Vector2d(x, y);
    }

    public static Vector2d pos(double pos) {
        return pos(pos, pos);
    }

    public static Pose2d pose(double x, double y, double deg) {
        return new Pose2d(x, y, rad(deg));
    }


    public double radM(double deg) {
        return Math.toRadians(deg) * multiplier();
    }

    public Vector2d posM(double x, double y) {
        return new Vector2d(x, y * multiplier());
    }

    public Vector2d posM(double pos) {
        return pos(pos, pos * multiplier());
    }

    public Pose2d poseM(double x, double y, double deg) {
        return new Pose2d(x, y * multiplier(), rad(deg * multiplier()));
    }

    public static double calculatePoint(double x1, double y1, double x2, double y2, boolean x, double xory) {
        if (x) {
            double slope = ((y2 - y1) / (x2 - x1));
            return slope * (xory - x1) + y1;
        } else {
            double slope = ((x2 - x1) / (y2 - y1));
            return slope * (xory - y1) + x1;
        }
    }

    public static LiftHandler.Position determineTarget(CameraHandler camera, float xCenter) {
        // Target will be high if there are no objects detected
        if (camera.mostConfident != null) {
            if (xCenter < 300) {
                //Set the target to low
                return LiftHandler.Position.LOW;
            }
            else if (xCenter < 600) {
                //Set the target to middle
                return LiftHandler.Position.MIDDLE;
            }
        }
        return LiftHandler.Position.HIGH;
    }

    public void cancelAndStop() {
        drive.cancelFollowing();
        drive.setDrivePower(new Pose2d());
    }

    public void backgroundLoop() {
        executor.submit(() -> {
            // Set up to blink robopandas in morse code
            light
                    .pause()
//                    //R
//                    .dot().dash().dot().pause()
//                    //O
//                    .dash().dash().dash().pause()
//                    //B
//                    .dash().dot().dot().dot().pause()
//                    //O
//                    .dash().dash().dash().pause()
                    //P
                    .dot().dash().dash().dot().pause()
                    //A
                    .dot().dash().pause()
                    //N
                    .dash().dot().pause()
                    //D
                    .dash().dot().dot().pause()
                    //A
                    .dot().dash().pause()
                    //S
                    .dot().dot().dot();
            waitForStart();
            light.resetTimer();
            while (opModeIsActive() && !isStopRequested()) {
                light.tick();
                telemetry.addData("bucket distance(cm)", distanceSensor.getDistance(DistanceUnit.CM)); //KEEP THIS HERE THE DISTANCE SENSOR READS ABOUT 3 WHEN IT FIRST TURNS ON
                telemetry.addData("xCoord distance(in)", xCoordinateSensor.getDistance(DistanceUnit.INCH));
                telemetry.update();
//                drive.update();
            }
            cancelAndStop();
            light.setColor(LightHandler.Color.OFF);
        });
    }

    /**
     * Trajectories need to be declared as fields and initialized here.
     */
    public abstract void initializeTrajectories();

    /**
     * In short, acts like the init() and start() methods combined.
     * This typically handles the camera and other beginning-of-match steps..
     * If your program does not use a camera, override this to be empty.
     */
    public void setup() {
        while (!opModeIsActive() && !isStopRequested()) {
            camera.tick();
            // Get x-coordinate of center of box
            if (camera.mostConfident != null) {
                xCenter = (camera.mostConfident.getLeft() + camera.mostConfident.getRight()) / 2;
                telemetry.addData("xCenter", xCenter);
                System.out.println(camera.mostConfident.getConfidence());
            }
            telemetry.addData("Ready!", ";ohifae;oihfew");
            light.setColor(LightHandler.Color.GREEN);
            telemetry.update();
        }
        // Run once when started
        target = determineTarget(camera, xCenter);

        lift.reset();

        // Raise arm
        arm.onStartAuto();
    }

    /**
     * Any code AFTER we determine the camera target goes here.
     */
    public abstract void main();
}