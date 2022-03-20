package org.firstinspires.ftc.teamcode.opmodes.autonomous

import android.os.Build
import androidx.annotation.RequiresApi
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DistanceSensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.*
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import kotlin.properties.Delegates

/**
 * A template containing shared code used in all autonomous programs.
 */
@RequiresApi(api = Build.VERSION_CODES.N)
abstract class AutonomousTemplate : LinearOpMode() {
    // Internal fields related to setup()
    private var xCenter by Delegates.notNull<Float>()
    protected var target = LiftHandler.Position.HIGH

    // Declare all hardware-related fields
    protected lateinit var camera: CameraHandler
    protected lateinit var drive: SampleMecanumDrive
    protected lateinit var duck: DuckHandler
    protected lateinit var arm: ArmHandler
    protected lateinit var lift: LiftHandler
    protected lateinit var bucket: BucketHandler
    protected lateinit var sweeper: SweeperHandler
    protected lateinit var intakeServo: IntakeServoHandler
    protected lateinit var light: LightHandler
    protected lateinit var distanceSensor: DistanceSensor
//    private lateinit var xCoordinateSensor: DistanceSensor // use if drive.xCoordinateSensor does not work
    protected var currentCycle: Cycle? = null

    override fun runOpMode() {
        // Initialize hardware
        camera = CameraHandler(hardwareMap, cameraName)
        drive = SampleMecanumDrive(hardwareMap)
        duck = DuckHandler(hardwareMap, null)
        arm = ArmHandler(hardwareMap, null, telemetry)
        lift = LiftHandler(hardwareMap, null, telemetry)
        bucket = BucketHandler(hardwareMap, null)
        sweeper = SweeperHandler(hardwareMap, null)
        intakeServo = IntakeServoHandler(hardwareMap)
        light = LightHandler(hardwareMap)
        distanceSensor = hardwareMap.get(DistanceSensor::class.java, "distanceSensor")

        light.setColor(LightHandler.Color.YELLOW)
        telemetry.addData("Status", "Initialized")

        // Assume intakeServo is close to up position
        intakeServo.hook()
        drive.poseEstimate = startPose

        // light and drive.update()
        backgroundLoop()

        initializeTrajectories()
        setup()

        // Raise arm + lift bucket halfway
        currentCycle = Cycle(sweeper, bucket, lift, target, distanceSensor)
        currentCycle!!.start()

        resetStartTime()
        main()
    }

    // Open/abstract properties and methods
    protected open val multiplier = 1 // 1 is for red; -1 is for blue. Multiply all headings and y-coordinates (NOT x-coordinates) by this.
    protected abstract val startPose: Pose2d
    protected open val cameraName: String = "intakeCamera"

    /**
     * Trajectories need to be declared as fields and initialized here.
     */
    abstract fun initializeTrajectories()

    /**
     * In short, acts like the init() and start() methods combined.
     * This typically handles the camera and other beginning-of-match steps..
     * If your program does not use a camera, override this to be empty.
     */
    open fun setup() {
        while (!opModeIsActive() && !isStopRequested) {
            camera.tick()
            // Get x-coordinate of center of box
            if (camera.mostConfident != null) {
                xCenter = (camera.mostConfident.left + camera.mostConfident.right) / 2
                telemetry.addData("xCenter", xCenter)
                println(camera.mostConfident.confidence)
            }
            telemetry.addData("Ready!", ";ohifae;oihfew")
            light.setColor(LightHandler.Color.GREEN)
            telemetry.update()
        }
        // Run once when started
        target = determineTarget(camera, xCenter)
        lift.reset()

        // Raise arm
        arm.onStartAuto()
    }

    /**
     * Any code AFTER we determine the camera target goes here.
     */
    abstract fun main()

    // Utilities
    fun cancelAndStop() {
        drive.cancelFollowing()
        drive.setDrivePower(Pose2d())
    }

    fun backgroundLoop() {
        executor.submit {
            light
                .pause()
//                //R
//                .dot().dash().dot().pause()
//                //O
//                .dash().dash().dash().pause()
//                //B
//                .dash().dot().dot().dot().pause()
//                //O
//                .dash().dash().dash().pause()
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
                .dot().dot().dot()
            waitForStart()
            light.resetTimer()
            while (opModeIsActive() && !isStopRequested) {
                light.tick()
                telemetry.addData("bucket distance(cm)", distanceSensor.getDistance(DistanceUnit.CM)) //KEEP THIS HERE THE DISTANCE SENSOR READS ABOUT 3 WHEN IT FIRST TURNS ON
                telemetry.addData("xCoord distance(in)", drive.xCoordinateSensor.getDistance(DistanceUnit.INCH))
                telemetry.update()
//                drive.update();
            }
            cancelAndStop()
            light.setColor(LightHandler.Color.OFF)
        }
    }

    fun radM(deg: Double): Double = rad(deg) * multiplier
    fun posM(x: Double, y: Double): Vector2d = Vector2d(x, y * multiplier)
    fun posM(pos: Double): Vector2d = posM(pos, pos)
    fun poseM(x: Double, y: Double, deg: Double): Pose2d = Pose2d(posM(x, y), radM(deg))
    
    companion object {
        protected val executor: ExecutorService = Executors.newSingleThreadExecutor()

        fun rad(deg: Double): Double = Math.toRadians(deg)
        fun pos(x: Double, y: Double): Vector2d = Vector2d(x, y)
        fun pos(pos: Double): Vector2d = pos(pos, pos)
        fun pose(x: Double, y: Double, deg: Double): Pose2d = Pose2d(x, y, rad(deg))

        fun calculatePoint(x1: Double, y1: Double, x2: Double, y2: Double, x: Boolean, xory: Double): Double {
            return if (x) {
                val slope = (y2 - y1) / (x2 - x1)
                slope * (xory - x1) + y1
            } else {
                val slope = (x2 - x1) / (y2 - y1)
                slope * (xory - y1) + x1
            }
        }

        // TODO refactor this to be target's getter
        fun determineTarget(camera: CameraHandler, xCenter: Float): LiftHandler.Position {
            // Target will be high if there are no objects detected
            if (camera.mostConfident != null) {
                if (xCenter < 300) {
                    // Set the target to low
                    return LiftHandler.Position.LOW
                } else if (xCenter < 600) {
                    // Set the target to middle
                    return LiftHandler.Position.MIDDLE
                }
            }
            return LiftHandler.Position.HIGH
        }
    }
}