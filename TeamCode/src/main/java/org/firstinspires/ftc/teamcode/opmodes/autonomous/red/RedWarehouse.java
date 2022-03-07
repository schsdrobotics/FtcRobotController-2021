/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opmodes.autonomous.red;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.Cycle;
import org.firstinspires.ftc.teamcode.LiftHandler;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonomousTemplate;

/**
 * Main for warehouse side
 * Warehouse side -> Preload -> 3 intake cycles -> park in warehouse
 */
@RequiresApi(api = Build.VERSION_CODES.N)
@Autonomous(name="RedWarehouse", group="Red Warehouse")
public class RedWarehouse extends AutonomousTemplate {
    private Trajectory toHubInitial;
    private Trajectory enterWarehouseAlign;
    private Trajectory enterWarehouseFast;
    private Trajectory[] toHub = new Trajectory[50];

//    private final double XTEMPADDER = 16;
    private double xTemp = 50 /*- XTEMPADDER*/;
//    private double yHubCoord = -39;

    protected int maxCycles() {
        return 3;
    }

    protected boolean shouldWaitForSeaLions() {
        return false;
    }

    @Override
    protected String cameraName() {
        return "liftCamera";
    }

    @Override
    protected Pose2d startPose() {
        return poseM(12, -63.375, 270);
    }

    @Override
    public void initializeTrajectories() {
        drive.velConstraint = SampleMecanumDrive.getVelocityConstraint(35, rad(240), 13.7);
//        drive.accelConstraint = SampleMecanumDrive.getAccelerationConstraint(35); // this is the default value so doesn't need to be written in

        toHubInitial = drive.trajectoryBuilder(startPose())
                .lineToLinearHeading(poseM(-15, -36, 280))
                .addTemporalMarker(1, -0.6, () -> {
                    //Drop and retract
                    currentCycle.finish();
                })
                .addTemporalMarker(1, -0.2, () -> {
                    //Cancel early to make it faster
                    cancelAndStop();
                })
            .build();

        enterWarehouseAlign = drive.trajectoryBuilder(poseM(-7, -37, 280), SampleMecanumDrive.getAccelerationConstraint(45))
                .lineToSplineHeading(pose(calculatePoint(-7, -37, 12, -69, false, -60), -60, 0))
                .lineToConstantHeading(pos(12, -69))
                .addTemporalMarker(1, -0.3, () -> {
                    //Cancel early to make it faster
                    cancelAndStop();
                })
                .build();

        enterWarehouseFast = drive.trajectoryBuilder(poseM(12, -65.375, 0), SampleMecanumDrive.getVelocityConstraint(45, rad(270), 13.7), SampleMecanumDrive.getAccelerationConstraint(75))
                .forward(30)
                .addTemporalMarker(1, -0.3, () -> {
                    //Cancel early to make it faster
                    cancelAndStop();
                })
                .build();

        //Fills toHub[] with 10 trajectories, one for every 2 x coordinates
        for (int i = 0; i < toHub.length; i++) {
            toHub[i] = drive.trajectoryBuilder(poseM(21 + i, -65.375, 0), true)
                    .splineToConstantHeading(pos(20, -65.375), rad(180))
                    .splineToConstantHeading(pos(10, -62), rad(180))
                    .splineTo(pos(-7, -35), rad(100))
                    .addTemporalMarker(1, -0.7, () -> {
                        //Drop and retract
                        currentCycle.finish();
                    })
                    .addTemporalMarker(1, -0.3, () -> {
                        //Cancel early to make it faster
                        cancelAndStop();
                    })
                    .build();
        }
    }

    @Override
    public void main() {
//        if (target == LiftHandler.Position.LOW) yHubCoord = -39;
//        else if (target == LiftHandler.Position.MIDDLE) yHubCoord = -38;
//        else if (target == LiftHandler.Position.HIGH) yHubCoord = -37;
        // Drop intake
        intakeServo.release();
        // Go to alliance hub (this also handles currentcycle.finish();
        drive.followTrajectory(toHubInitial);
//        enterWarehouse = buildEnterWarehouseTrajectory();
//        currentCycle.await();

        park: {
            for (int cycles = 0; cycles < maxCycles(); cycles++) {
                currentCycle = new Cycle(sweeper, bucket, lift, LiftHandler.Position.HIGH, hardwareMap.get(DistanceSensor.class, "distanceSensor"));
                // Align
                drive.followTrajectory(enterWarehouseAlign, false);
                drive.setMotorPowers(0.8, 0.8, 0.8, 0.8);
                sleep(370);
                drive.setMotorPowers(0.2, 0.2, 0.2, 0.2);
                sleep(50); //Sleep for 50ms to avoid voltage too low problems
                currentCycle.start();
                while (currentCycle.softIsBusy() && !currentCycle.isLowering() && !currentCycle.shouldCancel() && !isStopRequested()); //Await with a !isStopRequested()
//                if (drive.isBusy()) {
//                    cancelAndStop();
//                    drive.followTrajectoryAsync(toHub[cycles]);
//                }
//                else {
////                drive.followTrajectory(bonk, false);
////                drive.setPoseEstimate(poseM(bonk.end().getX(), -65.375, 0));
////                drive.followTrajectory(toWarehouse2, false); // THIS IS AN OLD TOWAREHOUSE2 AND IS NOT THE SAME AS IN THE NEW CODE. THIS WAS MERGED WITH TOWAREHOUSE1.
//                    drive.followTrajectoryAsync(pickupStraight, false);
//                    // wait for the cycle to finish before running the check for failure once
//                    if (currentCycle.await()) { // If this is true, we did NOT pick anything up
//                        // Assume that Cycle is working properly (i.e. the state will not be WAITING), and that there is only one possible error message
//                        if (!currentCycle.errorMessage.isEmpty())
//                            System.out.println("Failed to pick up an item; parking now");
//                        else System.out.println("Cycle is not working properly.");
//                        cancelAndStop();
//                        break park;
//                    } else {
//                        cancelAndStop();
//                        // To hub
//                        // Since we cancel our following, we need to get our start position for this trajectory on the fly
//                        xTemp = drive.findActualX(false);
//                        drive.setPoseEstimate(poseM(xTemp, -65.375, 0));
//                        drive.update();
//                        drive.followTrajectoryAsync(buildToHubTrajectory(), false);
//                    }
//                }

                drive.setDrivePower(new Pose2d());
                // To hub
                // Since we cancel our following, we need to get our start position for this trajectory on the fly
                xTemp = drive.findActualX(false);
                drive.setPoseEstimate(poseM(xTemp, -65.375, 0));
                drive.update();
                //Follow closest toHub trajectory
                int index = (int) (Math.round(xTemp) - 21);
                System.out.println("index: " + index);
                // This also handles currentCycle.finish();
                drive.followTrajectory((index >= 0 && index < toHub.length) ? toHub[index] : toHub[1], false);
                if (cycles == 0 && shouldWaitForSeaLions()) sleep(7000); // This only runs when running a SeaLion auto
//                drive.followTrajectoryAsync(toHub[cycles]);

//                if (getRuntime() < 7) { //If we run out of time
//                    cancelAndStop();
//                    break park;
//                }

//                enterWarehouse = buildEnterWarehouseTrajectory();
//                currentCycle.await();
                currentCycle = null;
            }

            arm.onStopAuto();
            drive.followTrajectory(enterWarehouseAlign, false);
            drive.setMotorPowers(0.8, 0.8, 0.8, 0.8);
            sleep(500);
            drive.setDrivePower(new Pose2d());

//            drive.followTrajectory(bonk, false);
//            drive.followTrajectory(toWarehouse2, false);
//            drive.followTrajectory(toWarehouse3, false);
        }
//
//        // Park
//        drive.followTrajectory(buildParkTrajectory(), false);
    }

    private Trajectory buildParkTrajectory() {
        drive.update();
        return drive.trajectoryBuilder(drive.getPoseEstimate(), false)
            .lineToSplineHeading(poseM(54, -65.375, 0))
            .splineToConstantHeading(posM(46, -38), radM(0))
            .lineToSplineHeading(poseM(60, -38, 270))
            .build();
    }

//    private Trajectory buildEnterWarehouseTrajectory() {
//        drive.update();
//        return drive.trajectoryBuilder(poseM(-7, -37, 280), false)
//                .splineToSplineHeading(pose(-2, -60, 0), rad(290))
//            .splineToConstantHeading(posM(12, -70), radM(0))
//            .lineToConstantHeading(posM(xTemp + XTEMPADDER, -70))
//            .build();
//    }
}
