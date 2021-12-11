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

package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DrivingHandler;
import org.firstinspires.ftc.teamcode.SweeperHandler;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;
import java.util.List;

/**
 * The OpMode that runs when the robot is automatically controlled.
 */
@RequiresApi(api = Build.VERSION_CODES.N)
@Autonomous(name="RedWarehouse")
public class RedWarehouse extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        runtime.reset();

        // Autonomous code goes here
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = pose(12, -62, 90);
        //TODO: pick whether you want the trajectories in a list or not - Stanley
        List<Trajectory> trajectories = new ArrayList<>();
        trajectories.add(drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(pose(-10, -40, 100))
                .build());
        trajectories.add(drive.trajectoryBuilder(trajectories.get(0).end())
                .lineToLinearHeading(pose(12, -62, 180))
                .build());
        trajectories.add(drive.trajectoryBuilder(trajectories.get(1).end())
                .forward(-30)
                .build());
        trajectories.add(drive.trajectoryBuilder(trajectories.get(2).end())
                .forward(30)
                .splineTo(pos(-10, -40), rad(100))
                .build());
        trajectories.add(drive.trajectoryBuilder(trajectories.get(3).end())
                .lineToLinearHeading(pose(12, -62, 180))
                .build());
        trajectories.add(drive.trajectoryBuilder(trajectories.get(4).end())
                .forward(-30)
                .build());

        //Other option
        /*
        Trajectory trajectory1 = drive.trajectoryBuilder(pose(-35, -62, 90))
                .lineTo(pos(-12, -45))
                .build();
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .lineToLinearHeading(pose(-60, -60, 180))
                .build();
        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .lineTo(pos(-60, -36))
                .build();
         */

        waitForStart();

        //Go to alliance hub
        drive.followTrajectory(trajectories.get(0));
        //Go back to starting position
        drive.followTrajectory(trajectories.get(1));
        //Go into warehouse
        drive.followTrajectory(trajectories.get(2));
        //Go to alliance hub
        drive.followTrajectory(trajectories.get(3));
        //Go back to starting position
        drive.followTrajectory(trajectories.get(4));
        //Go into warehouse
        drive.followTrajectory(trajectories.get(5));
        sleep(69420);
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

}