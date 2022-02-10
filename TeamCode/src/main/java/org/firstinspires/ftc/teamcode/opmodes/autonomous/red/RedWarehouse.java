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

import static org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonomousStuff.determineTarget;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonomousStuff.pos;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonomousStuff.pose;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonomousStuff.rad;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonomousStuff.backgroundLoop;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.ArmHandler;
import org.firstinspires.ftc.teamcode.BucketHandler;
import org.firstinspires.ftc.teamcode.CameraHandler;
import org.firstinspires.ftc.teamcode.Cycle;
import org.firstinspires.ftc.teamcode.DuckHandler;
import org.firstinspires.ftc.teamcode.IntakeServoHandler;
import org.firstinspires.ftc.teamcode.LiftHandler;
import org.firstinspires.ftc.teamcode.LightHandler;
import org.firstinspires.ftc.teamcode.SweeperHandler;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.concurrent.Executors;

/**
 * Main for warehouse side
 * Warehouse side -> Preload -> intake cycles -> park in warehouse(?)
 */
@RequiresApi(api = Build.VERSION_CODES.N)
@Autonomous(name="RedWarehouse", group="Red")
public class RedWarehouse extends LinearOpMode {
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
        TO_WAREHOUSE, // Go to warehouse
        TO_HUB, // Go to hub
        PARK, // Strafes left a little
        IDLE            // Our bot will enter the IDLE state when done
    }

    // We define the current state we're on
    // Default to IDLE
    private State currentState = State.IDLE;

    private final Pose2d startPose = pose(12, -61.375, 90);

    private final int MAX_CYCLES = 3;

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

        Cycle currentCycle;
        int cycles = 0;

        // Assume intakeServo is close to up position
        intakeServo.hook();

        drive.setPoseEstimate(startPose);

        Trajectory toHubInitial = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(pose(-5, -38, 280))
                .build();

        Trajectory toWarehouse1 = drive.trajectoryBuilder(toHubInitial.end(), false)
                .splineTo(pos(12, -61), 0)
                .build();

        Trajectory toWarehouse2 = drive.trajectoryBuilder(toWarehouse1.end(), false)
                .forward(48)
                .build();

        Trajectory park = drive.trajectoryBuilder(toWarehouse2.end(), false)
                .forward(-6)
                .splineToConstantHeading(pos(42, -38), rad(0))
                .lineToSplineHeading(pose(60, -38, 270))
                .build();

        while (!opModeIsActive() && !isStopRequested()) {
            camera.tick();
            // Get x-coordinate of center of box
            if (camera.mostConfident != null) {
                xCenter = (camera.mostConfident.getLeft() + camera.mostConfident.getRight()) / 2;
                telemetry.addData("xCenter", xCenter);
                System.out.println(camera.mostConfident.getConfidence());
            }
            telemetry.addData("Ready!", ";ohifae;oihfew");
            light.setColor(LightHandler.Color.GREEN);
            telemetry.update();
        }
        // light and drive.update()
        backgroundLoop(this, drive, light);
        // Run once when started
        target = determineTarget(camera, xCenter);

        currentCycle = new Cycle(sweeper, bucket, lift, target, hardwareMap.get(DistanceSensor.class, "distanceSensor"));

        // Set the current state to TO_HUB_INITIAL, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        currentState = State.TO_HUB_INITIAL;
        State last;
        while (opModeIsActive() && !isStopRequested()) {
            last = currentState;
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
                        currentCycle.finish();
                        currentCycle.await();

                        currentState = State.TO_WAREHOUSE;
                    }
                    break;
                case TO_WAREHOUSE:
                    if (!drive.isBusy()) {
                        drive.followTrajectory(toWarehouse1);
                        cycles++; // Cycles are counted by how many times we reach the warehouse, and this is close enough
                        if (cycles <= MAX_CYCLES) {
                            currentCycle = new Cycle(sweeper, bucket, lift, LiftHandler.Position.HIGH, hardwareMap.get(DistanceSensor.class, "distanceSensor"));
                            drive.followTrajectoryAsync(toWarehouse2);
                            currentCycle.start();
                            // wait for the cycle to finish before running the check for failure once
                            if (currentCycle.await()) { // If this is true, we did NOT pick anything up
                                // Assume that Cycle is working properly (i.e. the state will not be WAITING), and that there is only one possible error message
                                if (!currentCycle.errorMessage.isEmpty()) System.out.println("Failed to pick up an item; parking now");
                                else System.out.println("Cycle is not working properly.");
                                currentState = State.PARK;
                            } else {
                                drive.cancelFollowing();
                                drive.setDrivePower(new Pose2d());
                                currentState = State.TO_HUB;
                            }
                        } else {
                            drive.followTrajectoryAsync(toWarehouse2);
                            currentState = State.PARK;
                        }
                    }
                    break;
                case TO_HUB:
                    if (!drive.isBusy()) {
                        // Since we cancel our following, we need to get our start psoition for this trajecotry on the fly
                        drive.followTrajectory(buildHubTrajectory());
                        currentCycle.finish();
                        currentCycle.await();
                        currentCycle = null;
                        currentState = State./*TO_WAREHOUSE*/IDLE;
                    }
                    break;
                case PARK:
                    if (!drive.isBusy()) {
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
            telemetry.addData("State", currentState);
        }
    }

    private Trajectory buildHubTrajectory() {
        return drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                .lineToLinearHeading(pose(12, -62, 0))
                .splineToSplineHeading(pose(-5, -38, 280), rad(100))
                .build();
    }
}
