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
 * Backup for warehouse side
 * Warehouse side -> park in warehouse
 */
@RequiresApi(api = Build.VERSION_CODES.N)
@Autonomous(name="RedWarehousePark", group="Red")
public class RedWarehousePark extends LinearOpMode {
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
    private enum State { // This could just be linear, but I made it this way in case it needs expanding.
        ALIGN, // Turns
        PARK, // Strafes left a little
        IDLE            // Our bot will enter the IDLE state when done
    }

    // We define the current state we're on
    // Default to IDLE
    private State currentState = State.IDLE;

    private final Pose2d startPose = pose(12, -61, 90);

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
//        duck = new DuckHandler(hardwareMap, null);
//        arm = new ArmHandler(hardwareMap, null);
//        lift = new LiftHandler(hardwareMap, null, telemetry);
//        bucket = new BucketHandler(hardwareMap, null);
//        sweeper = new SweeperHandler(hardwareMap, null);
        intakeServo = new IntakeServoHandler(hardwareMap);
        camera = new CameraHandler(hardwareMap);
        light = new LightHandler(hardwareMap);
        light.setColor(LightHandler.Color.YELLOW);
        telemetry.addData("Status", "Initialized");

        // Assume intakeServo is close to up position
        intakeServo.hook();

        drive.setPoseEstimate(startPose);

        Trajectory park = drive.trajectoryBuilder(startPose)
                .forward(30)
                .strafeLeft(24)
                .lineToLinearHeading(pose(60, -38, 270))
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
            telemetry.addData("Ready!", ";ohifae;oihfew");
            light.setColor(LightHandler.Color.GREEN);
            telemetry.update();
        }

        //Run once when started
        target = determineTarget(camera, xCenter);

        // Set the current state to TO_HUB_INITIAL, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        currentState = State.PARK;

        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case ALIGN: // TODO copy the trajectories from MeepMeep (after fixing them)
                    if (!drive.isBusy()) {
                        drive.turnAsync(rad(-90));

                        currentState = State.PARK;
                    }
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
            drive.update();
            telemetry.addData("State", currentState);
        }
    }
}
