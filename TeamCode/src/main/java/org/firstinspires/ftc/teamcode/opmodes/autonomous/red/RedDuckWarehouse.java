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

import org.firstinspires.ftc.teamcode.LiftHandler;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonomousTemplate;

/**
 * Main for duck side
 * Duck side -> Preload -> duck -> park in warehouse
 */
@RequiresApi(api = Build.VERSION_CODES.N)
@Autonomous(name="RedDuckWarehouse", group="Red")
public class RedDuckWarehouse extends AutonomousTemplate {
    @Override
    protected Pose2d startPose() {
        return pose(-35, -61.375, 90);
    }

    Trajectory toHubInitial = drive.trajectoryBuilder(startPose())
            .splineToConstantHeading(pos(-57,-38), rad(90))
            .lineToConstantHeading(pos(-57, -28))
            .splineToSplineHeading(pose(-30, -24, 180), 0)
            .build();

    Trajectory toDuckSpinner = drive.trajectoryBuilder(toHubInitial.end())
            .splineToSplineHeading(pose(-50, -22, 270), rad(180))
            .splineToConstantHeading(pos(-63.375, -35), rad(270))
            .forward(15)
            .build();

    Trajectory align = drive.trajectoryBuilder(toDuckSpinner.end())
            .lineToSplineHeading(pose(-55, -48, 0))
            .splineToConstantHeading(pos(-15, -64), rad(270))
            .strafeRight(7)
            .build();

    Trajectory toWarehouse = drive.trajectoryBuilder(pose(align.end().getX(), -65.25, 0))
            .forward(60)
            .build();

    Trajectory park = drive.trajectoryBuilder(toWarehouse.end())
            .strafeLeft(15)
            .build();

    @Override
    public void main() {
        // Go to alliance hub
        drive.followTrajectory(toHubInitial);
        // Drop and retract
        currentCycle.finish();
        currentCycle.await();
        // Go to duck spinner
        drive.followTrajectory(toDuckSpinner);
        // Lower lift
        lift.pursueTarget(LiftHandler.Position.LOW);
        // Run duck spinner for 2.5 seconds
        double startTime = getRuntime();
        while (getRuntime() - startTime < 1.5) {
            duck.tick();
            duck.start(); // red does not need reversing
        }
        // Stop duck motor
        duck.stop();
        duck.tick();
        //Wait until there's 8 seconds left
        while (getRuntime() < 22);
        // Align
        drive.followTrajectory(align);
        // Go to warehouse
        drive.followTrajectory(toWarehouse);
        // Park
        drive.followTrajectory(park);
    }
}
