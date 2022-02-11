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

import static org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonomousStuff.backgroundLoop;
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
import com.qualcomm.robotcore.hardware.Servo;

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

@RequiresApi(api = Build.VERSION_CODES.N)
@Autonomous(name="Test", group="Red")
public class Test extends LinearOpMode {
    // Declare OpMode members.
    private LiftHandler lift;
    private BucketHandler bucket;
    private SweeperHandler sweeper;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {
        lift = new LiftHandler(hardwareMap, null, telemetry);
        bucket = new BucketHandler(hardwareMap, null);
        sweeper = new SweeperHandler(hardwareMap, null);

        telemetry.setAutoClear(false);
        telemetry.addLine("back");
        telemetry.update();
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 2000) {
            bucket.backwards();
        }
        telemetry.addData("servoPos", bucket.servo.servo.getPosition());
        telemetry.addLine("half");
        telemetry.update();
        start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 2000) {
            bucket.halfway();
        }
        telemetry.addData("servoPos", bucket.servo.servo.getPosition());
        telemetry.addLine("forward");
        telemetry.update();
        start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 2000) {
            bucket.forwards();
        }
        telemetry.addData("servoPos", bucket.servo.servo.getPosition());
        telemetry.addLine("done");
        telemetry.update();
        sleep(10000);
//        Cycle currentCycle = new Cycle(sweeper, bucket, lift, LiftHandler.Position.HIGH, hardwareMap.get(DistanceSensor.class, "distanceSensor"));
//        System.out.println("start");
//        currentCycle.start();
//        System.out.println("await");
//        currentCycle.await();
//        sleep(5000);
//        System.out.println("finish");
//        currentCycle.finish();
//        System.out.println("await");
//        currentCycle.await();
    }
}
