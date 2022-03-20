package org.firstinspires.ftc.teamcode.opmodes.autonomous.red

import android.os.Build
import androidx.annotation.RequiresApi
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.DistanceSensor
import org.firstinspires.ftc.teamcode.Cycle
import org.firstinspires.ftc.teamcode.LiftHandler
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonomousTemplate
import kotlin.math.roundToInt

/**
 * Main for warehouse side
 * Warehouse side -> Preload -> 3 intake cycles -> park in warehouse
 */
@RequiresApi(api = Build.VERSION_CODES.N)
@Autonomous(name="RedWarehouse", group="Red Warehouse")
open class RedWarehouse : AutonomousTemplate() {
    protected lateinit var toHubInitial: Trajectory
    private val enterWarehouse = arrayOfNulls<Trajectory>(20)
    private lateinit var pickup: Trajectory
    private lateinit var enterWarehouseFinal: Trajectory
//    private Trajectory enterWarehouseAlign;
//    private Trajectory enterWarehouseFast;
    protected val toHub = arrayOfNulls<Trajectory>(50)

//    private final double XTEMPADDER = 16;
    private var xTemp = 50.0 /*- XTEMPADDER*/
//    private double yHubCoord = -39;

    protected open val maxCycles = 3
    protected open val shouldWaitForSeaLions = false

    override val cameraName = "liftCamera"
    override val startPose = poseM(12.0, -63.375, 270.0)

    override fun initializeTrajectories() {
        drive.velConstraint = SampleMecanumDrive.getVelocityConstraint(35.0, rad(240.0), 13.7)
        buildToHubInitial()
        for (i in enterWarehouse.indices) {
            enterWarehouse[i] = drive.trajectoryBuilder(poseM(-11.0, -37.0, 290.0), SampleMecanumDrive.getVelocityConstraint(45.0, rad(240.0), 13.7), SampleMecanumDrive.getAccelerationConstraint(45.0))
                .splineToSplineHeading(poseM(-2.0, -60.0, 0.0), radM(290.0))
                .splineToConstantHeading(posM(12.0, -71.0), radM(0.0))
                .splineToConstantHeading(posM((50 + i).toDouble(), -73.0), radM(0.0))
                .addTemporalMarker(1.0, -1.0, currentCycle!!::start)
                .addTemporalMarker(1.0, -0.7, this::cancelAndStop)
                .build()
        }
        pickup = drive.trajectoryBuilder(poseM(0.0, 0.0, 0.0), SampleMecanumDrive.getVelocityConstraint(10.0, rad(240.0), 13.7), SampleMecanumDrive.getAccelerationConstraint(75.0))
            .lineToConstantHeading(posM(20.0, -2.0))
            .build()
        enterWarehouseFinal = drive.trajectoryBuilder(poseM(-11.0, -37.0, 290.0), SampleMecanumDrive.getAccelerationConstraint(45.0))
            .splineToSplineHeading(poseM(-2.0, -60.0, 0.0), radM(290.0))
            .splineToConstantHeading(posM(12.0, -71.0), radM(0.0))
            .lineToConstantHeading(posM(56.0, -71.0))
            .addTemporalMarker(1.0, -0.7, this::cancelAndStop)
            .build()

        // Fills toHub[] with 50 trajectories, one for every x coordinate
        buildHubTrajectories()
    }

    override fun main() {
//        if (target == LiftHandler.Position.LOW) yHubCoord = -39;
//        else if (target == LiftHandler.Position.MIDDLE) yHubCoord = -38;
//        else if (target == LiftHandler.Position.HIGH) yHubCoord = -37;
        // Drop intake
        intakeServo.release()
        // Go to alliance hub (this also handles currentCycle.finish();
        drive.followTrajectory(toHubInitial)
        run park@{
            for (cycles in 0 until maxCycles) {
                currentCycle = Cycle(sweeper, bucket, lift, LiftHandler.Position.HIGH, hardwareMap[DistanceSensor::class.java, "distanceSensor"])
                // Align
                val indexWarehouse = (xTemp.roundToInt() - 50)
                drive.followTrajectoryAsync(if (indexWarehouse >= 0 && indexWarehouse < enterWarehouse.size) enterWarehouse[indexWarehouse] else enterWarehouse[0], false)
                while (!currentCycle!!.shouldCancel && drive.isBusy && !isStopRequested);
                cancelAndStop()
                val startTime = runtime
                if (!drive.isBusy) {
                    drive.followTrajectoryAsync(pickup)
                    while (currentCycle!!.softIsBusy && !currentCycle!!.isLowering && !currentCycle!!.shouldCancel && !isStopRequested && runtime - startTime < 3);
                    cancelAndStop()
                }

                if (runtime - startTime > 3) return@park
                if (shouldWaitForSeaLions) sleep(3000) // This only runs when running a SeaLion auto
                // To hub
                // Since we cancel our following, we need to get our start position for this trajectory on the fly
                xTemp = drive.findActualX(false)
                drive.poseEstimate = poseM(xTemp, -65.375, 0.0)
                drive.update()
                // Follow closest toHub trajectory
                val indexHub = (xTemp.roundToInt() - 21)
                println("index: $indexHub")
                // This also handles currentCycle.finish();
                drive.followTrajectory(if (indexHub >= 0 && indexHub < toHub.size) toHub[indexHub] else toHub[1], false)

                currentCycle = null
            }
            arm.onStopAuto()
            drive.followTrajectory(enterWarehouseFinal, false)
        }
        arm.onStopAuto()
//        // Park
//        drive.followTrajectory(buildParkTrajectory(), false);
    }

    protected open fun buildToHubInitial() {
        toHubInitial = drive.trajectoryBuilder(startPose)
            .lineToSplineHeading(poseM(-15.0, -37.0, 290.0))
            .addTemporalMarker(1.0, -0.7) {
                // Drop and retract
                currentCycle!!.finish()
            }
            .addTemporalMarker(1.0, -0.2) {
                // Cancel early to make it faster
                cancelAndStop()
            }
            .build()
    }

    protected open fun buildHubTrajectories() {
        for (i in toHub.indices) {
            toHub[i] = drive.trajectoryBuilder(poseM((21 + i).toDouble(), -65.375, 0.0), true)
                .splineToConstantHeading(posM(20.0, -65.375), radM(180.0))
                .splineToConstantHeading(posM(10.0, -62.0), radM(180.0))
                .splineTo(posM(-7.0, -35.0), radM(110.0))
                .addTemporalMarker(1.0, -0.8) {
                    // Drop and retract
                    currentCycle!!.finish()
                }
                .addTemporalMarker(1.0, -0.3) {
                    // Cancel early to make it faster
                    cancelAndStop()
                }
                .build()
        }
    }
}