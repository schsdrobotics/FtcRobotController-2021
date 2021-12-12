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
import org.firstinspires.ftc.teamcode.ServoHandler;
import org.firstinspires.ftc.teamcode.SweeperHandler;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import org.firstinspires.ftc.teamcode.LiftHandler;

/**
 * The OpMode that runs when the robot is automatically controlled.
 */
@RequiresApi(api = Build.VERSION_CODES.N)
@Autonomous(name="RedWarehouse")
public class RedWarehouse extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private LiftHandler liftHandler;
    private SweeperHandler sweeperHandler;
    private ServoHandler servoHandler;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        runtime.reset();

        // Autonomous code goes here
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        liftHandler = new LiftHandler(hardwareMap, gamepad1);
        sweeperHandler = new SweeperHandler(hardwareMap, gamepad1);
        servoHandler = new ServoHandler(hardwareMap, gamepad1);

        TrajectorySequence path = drive.trajectorySequenceBuilder(pose(12, -62, 90))
                //Raise lift
                .addDisplacementMarker(() -> {
                    /* TODO: Fix this
                    liftHandler.setPosition("HIGH");
                     */
                })
                //Go to alliance hub
                .lineToLinearHeading(pose(-5, -42, 100))
                //TODO: Tune this later
                .waitSeconds(2)
                //Drop object
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    servoHandler.forwards();
                })
                //Lower lift
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    servoHandler.backwards();
                    /* TODO: Fix this
                    liftHandler.setPosition("LOW");
                     */
                })
                //Go into warehouse
                .setReversed(true)
                .splineTo(pos(12, -62), rad(0))
                .forward(-30)
                //Intake on
                .UNSTABLE_addDisplacementMarkerOffset(-3, () -> {
                    sweeperHandler.forwards(1);
                })
                //Intake off
                .UNSTABLE_addDisplacementMarkerOffset(3, () -> {
                    sweeperHandler.forwards(0);
                })
                //Go out of warehouse
                .forward(30)
                //Raise lift
                .addDisplacementMarker(() -> {
                    /*TODO: Fix this
                    liftHandler.setPosition("HIGH");
                     */
                })
                .splineTo(pos(-5, -42), rad(100))
                //TODO: Tune this later
                .waitSeconds(2)
                //Drop object
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    servoHandler.forwards();
                })
                //Lower lift
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    servoHandler.backwards();
                    /* TODO: Fix this
                    liftHandler.setPosition("LOW");
                     */
                })
                //Go into warehouse
                .setReversed(true)
                .splineTo(pos(12, -62), rad(0))
                .forward(-30)
                .build();

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

        drive.followTrajectorySequence(path);
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