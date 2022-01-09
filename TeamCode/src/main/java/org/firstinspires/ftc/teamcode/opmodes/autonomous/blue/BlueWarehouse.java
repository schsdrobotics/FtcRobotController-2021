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
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BucketHandler;
import org.firstinspires.ftc.teamcode.IntakeServoHandler;
import org.firstinspires.ftc.teamcode.LiftHandler;
import org.firstinspires.ftc.teamcode.SweeperHandler;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * The OpMode that runs when the robot is automatically controlled.
 */
@RequiresApi(api = Build.VERSION_CODES.N)
@Autonomous(name="BlueWarehouse", group="Blue")
public class BlueWarehouse extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        runtime.reset();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        LiftHandler lift = new LiftHandler(hardwareMap, null, telemetry);
        BucketHandler bucket = new BucketHandler(hardwareMap, null);
        SweeperHandler sweeper = new SweeperHandler(hardwareMap, null);
        IntakeServoHandler intakeServo = new IntakeServoHandler(hardwareMap, null);

        //Assume lift is down
        lift.finishInit();
        //Assume intakeServo is close to up position
        intakeServo.goToPos(intakeServo.HOOKED);

        //This is a faster autonomous than the one below
        TrajectorySequence seq1 = drive.trajectorySequenceBuilder(pose(12, 62, 270))
                .addDisplacementMarker(() -> {
                    //Raise lift
                    lift.pursueTargetAuto(lift.HIGH);
                    //Drop intake
                    intakeServo.goToPos(intakeServo.RELEASED);
                })
                //Go to alliance hub
                .lineToLinearHeading(pose(-5, 42, -100))
                .waitSeconds(2)
                //Drop object
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    bucket.forwards();
                })
                .addTemporalMarker(() -> {
                    //Retract bucket
                    bucket.backwards();
                    //Lower lift
                    lift.pursueTargetAuto(lift.LOW);
                })
                //Go into warehouse
                .setReversed(true)
                .splineTo(pos(12, 62), rad(0))
                .forward(-30)
                //Intake on
                .UNSTABLE_addDisplacementMarkerOffset(-3, () -> {
                    sweeper.forwards(1); // FIXME forwards or backwards?
                })
                //Intake off
                .UNSTABLE_addDisplacementMarkerOffset(3, () -> {
                    sweeper.forwards(0);
                })
                //Raise lift
                .UNSTABLE_addDisplacementMarkerOffset(8, () -> {
                    lift.pursueTargetAuto(lift.HIGH);
                })
                //Go to alliance hub
                .forward(30)
                .splineTo(pos(-5, 42), rad(-100))
                .waitSeconds(0.5)
                //Drop object
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    bucket.forwards();
                })
                //Lower lift
                .addTemporalMarker(() -> {
                    bucket.backwards();
                    lift.pursueTargetAuto(lift.LOW);
                })
                //Go into warehouse
                .setReversed(true)
                .splineTo(pos(12, 62), rad(0))
                .forward(-26)
                //Go deeper into warehouse
                .strafeLeft(24)
                .lineToLinearHeading(pose(60, 38, 270))
                .build();

//        TrajectorySequence seq = drive.trajectorySequenceBuilder(pose(12, 62, 270))
//                .lineToLinearHeading(pose(-5, 42, -100))
//                .addTemporalMarker(() -> {
//                    lift.setTarget(LiftHandler.Position.HIGH);
//                })
//                .waitSeconds(2)
//                .addTemporalMarker(bucket::forwards)
//                .waitSeconds(1.5)
//                .setReversed(true)
//                .splineTo(pos(12, 62), rad(0))
//                .addTemporalMarker(() -> {
//                    bucket.backwards();
//                    lift.setTarget(LiftHandler.Position.LOW);
//                })
//                .waitSeconds(2)
//                .addTemporalMarker(() -> {
//                    sweeper.forwards(1);
//                })
//                .forward(-30)
//                .addTemporalMarker(() -> {
//                    sweeper.stop();
//                    lift.setTarget(LiftHandler.Position.HIGH);
//                })
//                .forward(30)
//                .splineTo(pos(-5, 42), rad(-100))
//                .addTemporalMarker(bucket::forwards)
//                .waitSeconds(2)
//                .setReversed(true)
//                .splineTo(pos(12, 62), rad(0)) // now in storage
//                .forward(-30)
//                .build();

        waitForStart();
        drive.followTrajectorySequence(seq1);
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