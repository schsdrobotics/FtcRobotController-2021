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
 * Backup for warehouse side
 * Warehouse side -> park in warehouse
 */
@RequiresApi(api = Build.VERSION_CODES.N)
@Autonomous(name="RedWarehousePark", group="Red")
public class RedWarehousePark extends AutonomousTemplate {
    Trajectory out;
    Trajectory align;
    Trajectory park;

    @Override
    protected Pose2d startPose() {
        return pose(12, -61.375, 90);
    }

    @Override
    public void initializeTrajectories() {
        out = drive.trajectoryBuilder(startPose())
                .splineToLinearHeading(pose(6, -55, 0), rad(270))
                .build();
        align = drive.trajectoryBuilder(out.end())
                .strafeRight(14)
                .build();
        park = drive.trajectoryBuilder(pose(align.end().getX(), -63.375, 0))
                .forward(27)
                .splineToConstantHeading(pos(44, -38), rad(0))
                .lineToSplineHeading(pose(60, -38, 270))
                .build();
    }

    @Override
    public void setup() { target = LiftHandler.Position.LOW; }

    @Override
    public void main() {
        //Go out
        drive.followTrajectory(out, false);
        //Align
        drive.followTrajectory(align, false);
        //Set pose estimate since we just bonked against the wall
        drive.setPoseEstimate(pose(drive.getPoseEstimate().getX(), -63.375, 0));
        //Park
        drive.followTrajectory(park, true);
    }
}
