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

import static org.firstinspires.ftc.teamcode.opmodes.autonomous.blue.BlueDuckStorage.pos;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.blue.BlueDuckStorage.pose;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.blue.BlueDuckStorage.rad;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BucketHandler;
import org.firstinspires.ftc.teamcode.CameraHandler;
import org.firstinspires.ftc.teamcode.DuckHandler;
import org.firstinspires.ftc.teamcode.IntakeServoHandler;
import org.firstinspires.ftc.teamcode.LiftHandler;
import org.firstinspires.ftc.teamcode.SweeperHandler;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * The OpMode that runs when the robot is automatically controlled.
 */
@RequiresApi(api = Build.VERSION_CODES.N)
@Autonomous(name="Remote", group="Remote")
public class Remote extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private float xCenter;
    private int target;

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    DuckHandler duck = new DuckHandler(hardwareMap, null);
    LiftHandler lift = new LiftHandler(hardwareMap, null, telemetry);
    BucketHandler bucket = new BucketHandler(hardwareMap, null);
    SweeperHandler sweeper = new SweeperHandler(hardwareMap, null);
    CameraHandler camera = new CameraHandler(hardwareMap);
    IntakeServoHandler intakeServo = new IntakeServoHandler(hardwareMap);

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        runtime.reset();

        //Assume intakeServo is close to up position
        intakeServo.goToPos(intakeServo.HOOKED);

        while (!opModeIsActive()) {
            camera.tick();
            //Get x-coordinate of center of box
            if (camera.mostConfident != null) {
                xCenter = (camera.mostConfident.getLeft() + camera.mostConfident.getRight())/2;
            }
        }
        waitForStart();

        determineTarget();

        TrajectorySequence seq1 = drive.trajectorySequenceBuilder(pose(-35, -62, 270))
                .addTemporalMarker(() -> {
                    //Drop intake
                    intakeServo.goToPos(intakeServo.RELEASED);
                })
                .lineToLinearHeading(pose(-58, -60, 270))
                .addTemporalMarker(() -> {
                    //Raise lift
                    lift.pursueTargetAuto(target);
                })
                .build();

        TrajectorySequence seq2 = drive.trajectorySequenceBuilder(seq1.end())
                .addTemporalMarker(() -> {
                    //Stop duck motor
                    duck.stop();
                    duck.tick();
                })
                //Go to alliance hub
                .lineToLinearHeading(pose(-12, -45, 270))
                .addTemporalMarker(() -> {
                    //Deposit item
                    bucket.forwards();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    //Retract bucket and lift
                    bucket.backwards();
                    lift.pursueTargetAuto(LiftHandler.LOW);
                })
                //Go into warehouse
                .splineTo(pos(12, -62), rad(0))
                .forward(30)

                //Intake on
                .UNSTABLE_addDisplacementMarkerOffset(-5, () -> {
                    sweeper.forwards(1); // FIXME forwards or backwards?
                })
                //Intake off
                .UNSTABLE_addDisplacementMarkerOffset(5, () -> {
                    sweeper.forwards(0);
                })
                //Raise lift
                .UNSTABLE_addDisplacementMarkerOffset(8, () -> {
                    lift.pursueTargetAuto(LiftHandler.HIGH);
                })
                //Go to alliance hub
                .forward(-30)
                .splineTo(pos(-5, -42), rad(110))
                .addTemporalMarker(() -> {
                    //Deposit item
                    bucket.forwards();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    //Retract bucket and lift
                    bucket.backwards();
                    lift.pursueTargetAuto(LiftHandler.LOW);
                })
                //Go into warehouse
                .splineTo(pos(12, -62), rad(0))
                .forward(30)

                //Intake on
                .UNSTABLE_addDisplacementMarkerOffset(-5, () -> {
                    sweeper.forwards(1); // FIXME forwards or backwards?
                })
                //Intake off
                .UNSTABLE_addDisplacementMarkerOffset(5, () -> {
                    sweeper.forwards(0);
                })
                //Raise lift
                .UNSTABLE_addDisplacementMarkerOffset(8, () -> {
                    lift.pursueTargetAuto(LiftHandler.HIGH);
                })
                //Go to alliance hub
                .forward(-30)
                .splineTo(pos(-5, -42), rad(110))
                .addTemporalMarker(() -> {
                    //Deposit item
                    bucket.forwards();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    //Retract bucket and lift
                    bucket.backwards();
                    lift.pursueTargetAuto(LiftHandler.LOW);
                })
                //Go into warehouse
                .splineTo(pos(12, -62), rad(0))
                .forward(30)
                .build();

        drive.followTrajectorySequence(seq1);
        //Run duck spinner for 2.5 seconds
        double startTime = getRuntime();
        while (getRuntime() - startTime < 2.5) {
            duck.tick();
            duck.start(); // FIXME once we have a robot, see if we need to call reverse for red or blue
        }
        drive.followTrajectorySequence(seq2);
    }

    private void determineTarget() {
        //Target will be high if there are no objects detected
        target = LiftHandler.HIGH;
        if (camera.mostConfident != null) {
            //FIXME tune these values
            if (xCenter < 477) {
                //Set the target to low
                target = LiftHandler.LOW;
            }
            else if (xCenter > 803) {
                //Set the target to high
                target = LiftHandler.HIGH;
            }
            else {
                //Set the target to middle
                target = LiftHandler.MIDDLE;
            }
        }
    }
}