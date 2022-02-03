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

package org.firstinspires.ftc.teamcode.opmodes.autonomous.solo;

import static org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonomousStuff.*;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.BucketHandler;
import org.firstinspires.ftc.teamcode.CameraHandler;
import org.firstinspires.ftc.teamcode.DuckHandler;
import org.firstinspires.ftc.teamcode.IntakeServoHandler;
import org.firstinspires.ftc.teamcode.LiftHandler;
import org.firstinspires.ftc.teamcode.SweeperHandler;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * The OpMode that runs when the robot is automatically controlled.
 */
@RequiresApi(api = Build.VERSION_CODES.N)
@Autonomous(name="Remote2", group="Remote")
public class Remote2 extends OpMode {
    // Declare OpMode members.
    private float xCenter;
    private int target = LiftHandler.HIGH;
    private CameraHandler camera;
    private SampleMecanumDrive drive;
    private DuckHandler duck;
    private LiftHandler lift;
    private BucketHandler bucket;
    private SweeperHandler sweeper;
    private IntakeServoHandler intakeServo;

    // This enum defines our "state"
    // This essentially just defines the possible steps our program will take
    private enum State {
        TO_HUB_INITIAL,   // Set-up + go to alliance hub
        DROP_AND_RETRACT,   // drop item + retract bucket
        TO_DUCK_SPINNER,         // Go to duck spinner
        DELIVER_DUCKS,   // Deliver ducks + lower lift
        TO_WAREHOUSE_INITIAL, // Go to warehouse; TODO: starts intake cycle
        IDLE            // Our bot will enter the IDLE state when done
    }

    // We define the current state we're on
    // Default to IDLE
    private State currentState = State.IDLE;

    private final Pose2d startPose = pose(-35, -62, 90);
    private Trajectory traj1;
    private Trajectory traj2;
    private Trajectory traj3;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        duck = new DuckHandler(hardwareMap, null);
        lift = new LiftHandler(hardwareMap, null, telemetry);
        bucket = new BucketHandler(hardwareMap, null);
        sweeper = new SweeperHandler(hardwareMap, null);
        intakeServo = new IntakeServoHandler(hardwareMap);
        camera = new CameraHandler(hardwareMap);

        telemetry.addData("Status", "Initialized");

        // Assume intakeServo is close to up position
        intakeServo.hook();

        drive.setPoseEstimate(startPose);
    }

    /**
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        camera.tick();
        // Get x-coordinate of center of box
        if (camera.mostConfident != null) {
            xCenter = (camera.mostConfident.getLeft() + camera.mostConfident.getRight())/2;
            telemetry.addData("xCenter", xCenter);
            telemetry.update();
        }
    }

    /**
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        determineTarget();

        //This is in start because of the stupid stuck in init thing
        traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(pos(calculatePoint(-35, -62, -5, -39, false, -58), -58))
                .lineToSplineHeading(pose(-5, -39, 270))
                .build();

        traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(pose(-59,-51, 245))
                .build();

        traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToSplineHeading(pose(-20, -57, -5))
                .splineToConstantHeading(pos(50, -77), -5)
                .build();

        currentState = State.TO_HUB_INITIAL;
        // Drop intake
        intakeServo.release();
        // Make bucket stand straight up
        bucket.halfway();
        // Raise lift
        lift.pursueTarget(target);
        // Go to alliance hub
        drive.followTrajectoryAsync(traj1);
    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
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
                    currentState = State.DROP_AND_RETRACT;

                    // Drop item
                    bucket.forwards();
                    double startTime = getRuntime();
                    while (getRuntime() - startTime < 1.0) {} // Wait 1s
                    // Retract bucket
                    bucket.backwards();
                }
                break;
            case DROP_AND_RETRACT:
                if (!drive.isBusy()) {
                    currentState = State.TO_DUCK_SPINNER;

                    // Go to duck spinner
                    drive.followTrajectoryAsync(traj2);
                }
                break;
            case TO_DUCK_SPINNER:

                if (!drive.isBusy()) {
                    currentState = State.DELIVER_DUCKS;

                    // Lower lift
                    lift.pursueTarget(LiftHandler.INTAKING);
                    // Run duck spinner for 2.5 seconds
                    double startTime = getRuntime();
                    while (getRuntime() - startTime < 1.5) {
                        duck.tick();
                        duck.start(); // red does not need reversing
                    }
                    // Stop duck motor
                    duck.stop();
                    duck.tick();
                }
                break;
            case DELIVER_DUCKS:
                if (!drive.isBusy()) {
                    currentState = State.TO_WAREHOUSE_INITIAL;

                    // Go into warehouse
                    drive.followTrajectoryAsync(traj3);
                }
                break;
            case TO_WAREHOUSE_INITIAL:
                if (!drive.isBusy()) {
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
        // Distance sensor background loop
    }

    /**
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private void determineTarget() {
        // Target will be high if there are no objects detected
        if (camera.mostConfident != null) {
            // FIXME tune these values
            if (xCenter < 300) {
                //Set the target to low
                target = LiftHandler.LOW;
            }
            else if (xCenter < 600) {
                //Set the target to middle
                target = LiftHandler.MIDDLE;
            }
        }
    }
}
