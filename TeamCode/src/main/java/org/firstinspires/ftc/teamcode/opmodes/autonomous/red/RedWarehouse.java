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
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.Cycle;
import org.firstinspires.ftc.teamcode.LiftHandler;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonomousTemplate;


@RequiresApi(api = Build.VERSION_CODES.N)
@Autonomous(name="RedWarehouse", group="Red")
public class RedWarehouse extends AutonomousTemplate {
//    private Trajectory toHubInitial;
    private Trajectory toWarehouse1;
    private Trajectory bonk;
    private Trajectory toWarehouse2;
    private Trajectory toWarehouse3;
    private Trajectory park;

    private double xTemp = 48;
    private double xTempTurn = 48;
    private double yHubCoord;

    private final int MAX_CYCLES = 2;

    @Override
    protected Pose2d startPose() {
        return poseM(12, -63.375, 90);
    }

    @Override
    public void initializeTrajectories() {
        drive.velConstraint = SampleMecanumDrive.getVelocityConstraint(35, rad(240), 13.7);
        drive.accelConstraint = SampleMecanumDrive.getAccelerationConstraint(35);

        bonk = drive.trajectoryBuilder(pose(0, 0, 0), SampleMecanumDrive.getVelocityConstraint(23, rad(180), 13.7))
            .strafeRight(8)
            .build();

        toWarehouse2 = drive.trajectoryBuilder(pose(0, 0, 0), SampleMecanumDrive.getVelocityConstraint(10, rad(180), 13.7))
                .lineToConstantHeading(posM(12, -5))
                .build();

        toWarehouse3 = drive.trajectoryBuilder(pose(0, 0, 0), SampleMecanumDrive.getVelocityConstraint(13, rad(180), 13.7))
                .lineToConstantHeading(posM(10, 7))
                .build();
    }

    @Override
    public void main() {
        if (target == LiftHandler.Position.LOW) yHubCoord = -39;
        else if (target == LiftHandler.Position.MIDDLE) yHubCoord = -38;
        else if (target == LiftHandler.Position.HIGH) yHubCoord = -37;
        // Drop intake
        intakeServo.release();
        // Go to alliance hub
        drive.followTrajectoryAsync(drive.trajectoryBuilder(startPose())
            .lineToLinearHeading(poseM(-7, yHubCoord, 280))
            .build(), false);
        double startTime = getRuntime();
        while (getRuntime() < startTime + 2.25); //Wait for 2s
        // Drop and retract
        currentCycle.finish();
        toWarehouse1 = buildToWarehouse1Trajectory();
        currentCycle.await();

        park: {
            for (int cycles = 0; cycles < MAX_CYCLES; cycles++) {
                // To warehouse
                drive.followTrajectory(toWarehouse1, false);
//                drive.followTrajectory(bonk, false);
//                drive.setPoseEstimate(poseM(bonk.end().getX(), -65.375, 0));
//                drive.followTrajectory(toWarehouse2, false); // THIS IS AN OLD TOWAREHOUSE2 AND IS NOT THE SAME AS IN THE NEW CODE. THIS WAS MERGED WITH TOWAREHOUSE1.
                currentCycle = new Cycle(sweeper, bucket, lift, LiftHandler.Position.HIGH, hardwareMap.get(DistanceSensor.class, "distanceSensor"));
                if (cycles % 2 == 0) drive.followTrajectoryAsync(toWarehouse2, false);
                else drive.followTrajectoryAsync(toWarehouse3, false);
                currentCycle.start();
                // wait for the cycle to finish before running the check for failure once
                if (currentCycle.await()) { // If this is true, we did NOT pick anything up
                    // Assume that Cycle is working properly (i.e. the state will not be WAITING), and that there is only one possible error message
                    if (!currentCycle.errorMessage.isEmpty())
                        System.out.println("Failed to pick up an item; parking now");
                    else System.out.println("Cycle is not working properly.");
                    break park;
                } else {
                    drive.cancelFollowing();
                    drive.setDrivePower(new Pose2d());
                }

                // To hub
                // Since we cancel our following, we need to get our start position for this trajectory on the fly
                if (cycles % 2 == 1) {
//                    drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate(), false).forward(-6).build());
                    drive.followTrajectory(bonk);
                }
                xTemp = drive.findActualX(false);
                drive.setPoseEstimate(poseM(xTemp, -65.375, 0));
                drive.update();
                drive.followTrajectoryAsync(buildHubTrajectory(), false);
                startTime = getRuntime();
                while (getRuntime() < startTime + 3.25); //Wait for 3.75s
                currentCycle.finish();
                toWarehouse1 = buildToWarehouse1Trajectory();
                currentCycle.await();
                currentCycle = null;
            }

            arm.onStopAuto();
            drive.followTrajectory(toWarehouse1, false);
//            drive.followTrajectory(bonk, false);
//            drive.followTrajectory(toWarehouse2, false);
//            drive.followTrajectory(toWarehouse3, false);
        }
//
//        // Park
//        drive.followTrajectory(buildParkTrajectory(), false);
    }

    private Trajectory buildHubTrajectory() {
        drive.update();
        return drive.trajectoryBuilder(drive.getPoseEstimate(), false)
            .lineToSplineHeading(poseM(7, -65.375, 0))
            .splineToSplineHeading(poseM(-7, -37, 280), radM(100))
            .build();
    }

    private Trajectory buildParkTrajectory() {
        drive.update();
        return drive.trajectoryBuilder(drive.getPoseEstimate(), false)
            .lineToSplineHeading(poseM(54, -65.375, 0))
            .splineToConstantHeading(posM(46, -38), radM(0))
            .lineToSplineHeading(poseM(60, -38, 270))
            .build();
    }

    private Trajectory buildToWarehouse1Trajectory() {
        drive.update();
        return drive.trajectoryBuilder(poseM(-7, -37, 280), false)
            .splineToSplineHeading(poseM(-2, -54, 0), radM(290))
            .splineToConstantHeading(posM(12, -71), radM(0))
            .lineToConstantHeading(posM(xTemp, -71))
            .build();
    }
}
