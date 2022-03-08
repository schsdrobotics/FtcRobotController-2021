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

package org.firstinspires.ftc.teamcode.opmodes.autonomous.blue;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonomousTemplate;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.red.RedDuckStorage;

/**
 * Primary backup for duck side
 * Duck side -> Preload -> duck -> park in storage
 */
@RequiresApi(api = Build.VERSION_CODES.N)
@Autonomous(name="BlueDuckStorage", group="Blue Duck")
public class BlueDuckStorage extends AutonomousTemplate {
    private Trajectory toHubInitial;
    private Trajectory align;
    private Trajectory toDuckSpinner;
    private Trajectory park;

    @Override
    protected int multiplier() {
        return -1;
    }

    @Override
    protected Pose2d startPose() {
        return poseM(-37, -63.375, 90);
    }

    @Override
    public void initializeTrajectories() {
        toHubInitial = drive.trajectoryBuilder(startPose())
                .splineToConstantHeading(posM(-59,-40), radM(90))
                .lineToConstantHeading(posM(-59, -25))
                .splineToSplineHeading(poseM(-32, -21, 180), 0)
                .build();
        align = drive.trajectoryBuilder(toHubInitial.end())
                .splineToSplineHeading(poseM(-52, -19, 270), radM(180))
                .splineToConstantHeading(posM(-75, -27), radM(270))
                .build();
        toDuckSpinner = drive.trajectoryBuilder(pose(align.end().getX(), -65.375, 90))
                .forward(10)
                .splineToLinearHeading(poseM(-65.375 + 9.5, -63.375 + 6, 180), radM(180))
                .build();
        park = drive.trajectoryBuilder(toDuckSpinner.end())
                .lineToLinearHeading(poseM(-59,-35, 90))
                .build();
    }

    @Override
    public void main() {
        // Go to alliance hub
        drive.followTrajectory(toHubInitial, false);
        // Drop and retract
        currentCycle.finish();
        currentCycle.await();
        // Go to duck spinner
        drive.followTrajectory(align, false);
        // Reset pose estimate because we bonk
        drive.setPoseEstimate(pose(-65.375, align.end().getY(), 90));
        drive.followTrajectory(toDuckSpinner, false);
        // Run duck spinner for 1.5 seconds
        duck.reverse();
        double startTime = getRuntime();
        while (getRuntime() - startTime < 1.5) {
            duck.tick();
            duck.start();
        }
        // Stop duck motor
        duck.stop();
        duck.tick();
        // Park
        drive.followTrajectory(park, false);
        arm.onStopAuto();
    }
}