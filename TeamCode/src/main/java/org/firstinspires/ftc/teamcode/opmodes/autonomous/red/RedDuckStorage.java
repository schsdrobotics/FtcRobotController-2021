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

import static org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonomousStuff.calculatePoint;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonomousStuff.determineTarget;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonomousStuff.pos;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonomousStuff.pose;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ArmHandler;
import org.firstinspires.ftc.teamcode.BucketHandler;
import org.firstinspires.ftc.teamcode.CameraHandler;
import org.firstinspires.ftc.teamcode.DuckHandler;
import org.firstinspires.ftc.teamcode.IntakeServoHandler;
import org.firstinspires.ftc.teamcode.LiftHandler;
import org.firstinspires.ftc.teamcode.LightHandler;
import org.firstinspires.ftc.teamcode.SweeperHandler;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * Backup for duck side
 * Duck side -> Preload -> duck -> park in storage
 */
@RequiresApi(api = Build.VERSION_CODES.N)
@Autonomous(name="RedDuckStorage", group="Red")
public class RedDuckStorage extends LinearOpMode {
    // Declare OpMode members.
    private float xCenter;
    private LiftHandler.Position target = LiftHandler.Position.HIGH;
    private CameraHandler camera;
    private SampleMecanumDrive drive;
    private DuckHandler duck;
    private ArmHandler arm;
    private LiftHandler lift;
    private BucketHandler bucket;
    private SweeperHandler sweeper;
    private IntakeServoHandler intakeServo;
    private LightHandler light;

    // This enum defines our "state"
    // This essentially just defines the possible steps our program will take
    private enum State {
        TO_HUB_INITIAL,   // Set-up + go to alliance hub
        DROP_AND_RETRACT,   // drop item + retract bucket
        TO_DUCK_SPINNER,         // Go to duck spinner
        DELIVER_DUCKS, // Deliver ducks + lower lift
        ALIGN, // Align the robot with the wall
        PARK, // Park in storage area
        IDLE            // Our bot will enter the IDLE state when done
    }

    // We define the current state we're on
    // Default to IDLE
    private State currentState = State.IDLE;

    private final Pose2d startPose = pose(-35, -62, 90);

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        duck = new DuckHandler(hardwareMap, null);
        arm = new ArmHandler(hardwareMap, null);
        lift = new LiftHandler(hardwareMap, null, telemetry);
        bucket = new BucketHandler(hardwareMap, null);
        sweeper = new SweeperHandler(hardwareMap, null);
        intakeServo = new IntakeServoHandler(hardwareMap);
        camera = new CameraHandler(hardwareMap);
        light = new LightHandler(hardwareMap);
        light.setColor(LightHandler.Color.YELLOW);
        telemetry.addData("Status", "Initialized");

        // Assume intakeServo is close to up position
        intakeServo.hook();

        drive.setPoseEstimate(startPose);

        Trajectory toHubInitial = drive.trajectoryBuilder(startPose)
                .lineTo(pos(calculatePoint(-35, -62, -7, -40, false, -58), -58))
                .lineToSplineHeading(pose(-7, -40, 270))
                .build();

        Trajectory toDuckSpinner = drive.trajectoryBuilder(toHubInitial.end())
                .lineToLinearHeading(pose(-61,-51, 245))
                .build();

        Trajectory align = drive.trajectoryBuilder(toDuckSpinner.end())
                .lineToLinearHeading(pose(-71,-51, 270))
                .build();

        Trajectory park = drive.trajectoryBuilder(pose(-64.25, align.end().getY(), 270))
                .forward(-15)
                .build();

        light.runAuto(this);

        while (!opModeIsActive() && !isStopRequested()) {
            camera.tick();
            // Get x-coordinate of center of box
            if (camera.mostConfident != null) {
                xCenter = (camera.mostConfident.getLeft() + camera.mostConfident.getRight()) / 2;
                telemetry.addData("xCenter", xCenter);
                System.out.println(camera.mostConfident.getConfidence());
            }
            telemetry.addData("Ready!", "ough nough; what will we dough");
            light.setColor(LightHandler.Color.GREEN);
            telemetry.update();
        }

        //Run once when started
        target = determineTarget(camera, xCenter);

        // Set the current state to TO_HUB_INITIAL, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        currentState = State.TO_HUB_INITIAL;

        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case TO_HUB_INITIAL:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy()) {
                        // Raise arm
                        arm.onStart();
                        // Drop intake
                        intakeServo.release();
                        // Make bucket stand straight up
                        bucket.halfway();
                        // Raise lift
                        lift.pursueTarget(target);
                        // Go to alliance hub
                        drive.followTrajectoryAsync(toHubInitial);

                        currentState = State.DROP_AND_RETRACT;
                    }
                    break;
                case DROP_AND_RETRACT:
                    if (!drive.isBusy()) {
                        // Drop item
                        bucket.forwards();
                        double startTime = getRuntime();
                        while (getRuntime() - startTime < 0.350); // Wait 350 ms
                        bucket.wiggleUntil(() -> {
                            return getRuntime() - startTime > 1; // wiggle for 1 sec at most
                        });
                        // Retract bucket
                        bucket.backwards();

                        currentState = State.TO_DUCK_SPINNER;
                    }
                    break;
                case TO_DUCK_SPINNER:
                    if (!drive.isBusy()) {
                        // Go to duck spinner
                        drive.followTrajectoryAsync(toDuckSpinner);

                        currentState = State.DELIVER_DUCKS;
                    }
                    break;
                case DELIVER_DUCKS:
                    if (!drive.isBusy()) {
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
                        currentState = State.ALIGN;
                    }
                    break;
                case ALIGN:
                    if (!drive.isBusy()) {
                        // Align
                        drive.followTrajectoryAsync(align);

                        currentState = State.PARK;
                    }
                    break;
                case PARK:
                    if (!drive.isBusy()) {
                        // Park
                        drive.followTrajectoryAsync(park);

                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }

            // Anything outside of the switch statement will run independent of the currentState
            drive.update();
            telemetry.addData("State", currentState);
        }
    }
}
