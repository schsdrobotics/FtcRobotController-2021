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


@RequiresApi(api = Build.VERSION_CODES.N)
@Autonomous(name="RedWarehouse", group="Red")
public class RedWarehouse extends AutonomousTemplate {
    DistanceSensor xCoordinateSensor;

    Trajectory toHubInitial;
    Trajectory toWarehouse1;
    Trajectory bonk;
    Trajectory toWarehouse2;
    Trajectory toWarehouse3;
    Trajectory park;

    private final int MAX_CYCLES = 2;

    @Override
    protected Pose2d startPose() {
        return pose(12, -61.375, 90);
    }

    @Override
    public void initializeTrajectories() {
        toHubInitial = drive.trajectoryBuilder(startPose())
                .lineToLinearHeading(pose(-5, -38, 280))
                .build();

        toWarehouse1 = drive.trajectoryBuilder(toHubInitial.end(), false)
                .splineTo(pos(12, -61), 0)
                .build();

        bonk = drive.trajectoryBuilder(toWarehouse1.end())
                .strafeRight(8)
                .build();

        toWarehouse2 = drive.trajectoryBuilder(pose(bonk.end().getX(), -63.375, 0))
                .forward(28)
                .build();

        toWarehouse3 = drive.trajectoryBuilder(toWarehouse2.end(), SampleMecanumDrive.getVelocityConstraint(10, rad(180), 13.7))
                .forward(20)
                .build();

        xCoordinateSensor = hardwareMap.get(DistanceSensor.class, "xCoordinateSensor"); // putting this here to save time
    }

    @Override
    public void main() {
        // Drop intake
        intakeServo.release();
        // Go to alliance hub
        drive.followTrajectory(toHubInitial, false);
        // Drop and retract
        currentCycle.finish();
        currentCycle.await();

        park: {
            for (int cycles = 0; cycles < MAX_CYCLES - 1; cycles++) {
                // To warehouse
                drive.followTrajectory(toWarehouse1, false);
                drive.followTrajectory(bonk, false);
                drive.followTrajectory(toWarehouse2, false);
                currentCycle = new Cycle(sweeper, bucket, lift, LiftHandler.Position.HIGH, hardwareMap.get(DistanceSensor.class, "distanceSensor"));
                drive.followTrajectoryAsync(toWarehouse3, false);
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
                System.out.println("to hub");
                drive.followTrajectory(buildHubTrajectory(), false);
                System.out.println("finished hub");
                currentCycle.finish();
                currentCycle.await();
                currentCycle = null;
            }

            drive.followTrajectory(toWarehouse1, false);
            drive.followTrajectory(bonk, false);
            drive.followTrajectory(toWarehouse2, false);
            drive.followTrajectory(toWarehouse3, false);
        }

        // Park
        drive.followTrajectory(buildParkTrajectory(), false);
    }

    private Trajectory buildHubTrajectory() {
        drive.update();
        return drive.trajectoryBuilder(drive.getPoseEstimate(), false)
            .addDisplacementMarker(() -> System.out.println("line"))
            .lineToSplineHeading(pose(12, -63.375, 0))
            .addDisplacementMarker(() -> System.out.println("spline"))
            .splineToSplineHeading(pose(-5, -38, 280), rad(100))
            .build();
    }

    private Trajectory buildParkTrajectory() {
        drive.update();
        return drive.trajectoryBuilder(drive.getPoseEstimate(), false)
            .lineToSplineHeading(pose(54, -63.375, 0))
            .splineToConstantHeading(pos(46, -38), rad(0))
            .lineToSplineHeading(pose(60, -38, 270))
            .build();
    }
}
