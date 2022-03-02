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
    private Trajectory toHubInitial;
    private Trajectory toWarehouse1;
    private Trajectory bonk;
    private Trajectory toWarehouse2;
    private Trajectory toHub1;
    private Trajectory park;

    private double xTemp = 48;

    private final int MAX_CYCLES = 2;
    private final TrajectoryVelocityConstraint maxVel = SampleMecanumDrive.getVelocityConstraint(45, rad(240), 13.7);
    private final TrajectoryAccelerationConstraint maxAccel = SampleMecanumDrive.getAccelerationConstraint(39);

    @Override
    protected Pose2d startPose() {
        return poseM(12, -63.375, 90);
    }

    @Override
    public void initializeTrajectories() {
        drive.velConstraint = maxVel;
        drive.accelConstraint = maxAccel;
        toHubInitial = drive.trajectoryBuilder(startPose())
                .lineToLinearHeading(poseM(-5, -38, 280))
                .build();

//        bonk = drive.trajectoryBuilder(drive.getPoseEstimate(), SampleMecanumDrive.getVelocityConstraint(20, rad(180), 13.7))
//                .strafeRight(5 * multiplier())
//                .build();

        toWarehouse2 = drive.trajectoryBuilder(pose(0, 0, 0), SampleMecanumDrive.getVelocityConstraint(6, rad(180), 13.7))
                .lineToConstantHeading(pos(12, -3))
                .build();
    }

    @Override
    public void main() {
        // Drop intake
        intakeServo.release();
        // Go to alliance hub
        drive.followTrajectory(toHubInitial, false);
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
//                drive.followTrajectory(toWarehouse2, false);
                currentCycle = new Cycle(sweeper, bucket, lift, LiftHandler.Position.HIGH, hardwareMap.get(DistanceSensor.class, "distanceSensor"));
                drive.followTrajectoryAsync(toWarehouse2, false);
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
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate(), false).forward(-6).build());
//                drive.followTrajectory(bonk);
                xTemp = drive.findActualX(telemetry) + 6;
                drive.setPoseEstimate(poseM(drive.findActualX(telemetry), -65.375, 0));
                drive.update();
                drive.followTrajectory(buildHubTrajectory(), false);
                currentCycle.finish();
                toWarehouse1 = buildToWarehouse1Trajectory();
                currentCycle.await();
                currentCycle = null;
            }

//            drive.followTrajectory(toWarehouse1, false);
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
            .splineToSplineHeading(poseM(-5, -38, 280), radM(100))
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
        return drive.trajectoryBuilder(toHubInitial.end(), false)
            .splineToSplineHeading(poseM(-2, -54, 0), radM(290))
            .splineToConstantHeading(posM(12, -71), radM(0))
            .lineToConstantHeading(pos(xTemp, -71))
            .build();
    }
}
