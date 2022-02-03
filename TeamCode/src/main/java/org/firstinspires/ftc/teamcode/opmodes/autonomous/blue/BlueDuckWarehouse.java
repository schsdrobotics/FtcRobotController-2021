///* Copyright (c) 2017 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package org.firstinspires.ftc.teamcode.opmodes.autonomous.blue;
//
//import static org.firstinspires.ftc.teamcode.opmodes.autonomous.blue.BlueDuckStorage.pos;
//import static org.firstinspires.ftc.teamcode.opmodes.autonomous.blue.BlueDuckStorage.pose;
//import static org.firstinspires.ftc.teamcode.opmodes.autonomous.blue.BlueDuckStorage.rad;
//
//import android.os.Build;
//
//import androidx.annotation.RequiresApi;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.BucketHandler;
//import org.firstinspires.ftc.teamcode.DuckHandler;
//import org.firstinspires.ftc.teamcode.IntakeServoHandler;
//import org.firstinspires.ftc.teamcode.LiftHandler;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//
///**
// * The OpMode that runs when the robot is automatically controlled.
// */
//@RequiresApi(api = Build.VERSION_CODES.N)
//@Autonomous(name="BlueDuckWarehouse", group="Blue")
//public class BlueDuckWarehouse extends LinearOpMode {
//    // Declare OpMode members.
//    private ElapsedTime runtime = new ElapsedTime();
//
//    /**
//     * Code to run ONCE when the driver hits INIT
//     */
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry.addData("Status", "Initialized");
//        runtime.reset();
//
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        DuckHandler duck = new DuckHandler(hardwareMap, null);
//        LiftHandler lift = new LiftHandler(hardwareMap, null, telemetry);
//        BucketHandler bucket = new BucketHandler(hardwareMap, null);
//        IntakeServoHandler intakeServo = new IntakeServoHandler(hardwareMap);
//
//        //Assume intakeServo is close to up position
//        intakeServo.hook();
//
//        TrajectorySequence seq1 = drive.trajectorySequenceBuilder(pose(-35, 62, 270))
//                .addDisplacementMarker(() -> {
//                    //Raise lift
//                    lift.pursueTargetAuto(LiftHandler.HIGH);
//                    //Drop intake
//                    intakeServo.release();
//                })
//                //Go to alliance hub
//                .lineTo(pos(-12, 45))
//                .waitSeconds(2)
//                //Drop object
//                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
//                    bucket.forwards();
//                })
//                //Lower lift
//                .addTemporalMarker(() -> {
//                    bucket.backwards();
//                    lift.pursueTargetAuto(LiftHandler.LOW);
//                })
//                //Go to duck spinner
//                .lineToLinearHeading(pose(-60, 60, 90))
//                .build();
//
//        TrajectorySequence seq2 = drive.trajectorySequenceBuilder(seq1.end())
//                //Stop duck motor
//                .addTemporalMarker(() -> {
//                    duck.stop();
//                    duck.tick();
//                })
//                //Go into warehouse
//                .splineTo(pos(0, 62), rad(0))
//                .forward(40)
//                .build();
//
////        TrajectorySequence seq = drive.trajectorySequenceBuilder(pose(-35, 62, 270))
////                .lineTo(pos(-12, 45))
////                .addTemporalMarker(() -> {
////                    // drop initial cube
////                })
////                .waitSeconds(2)
////                .lineToLinearHeading(pose(-60, 60, 90))
////                .addTemporalMarker(duck::start)
////                .waitSeconds(2.5)
////                .addTemporalMarker(duck::stop)
////                .lineTo(pos(-60, 36)) // now in hub
////                .build();
//
//        waitForStart();
//        drive.followTrajectorySequence(seq1);
//        //Run duck motor for 2.5 seconds
//        double startTime = getRuntime();
//        while (getRuntime() - startTime < 2.5) {
//            duck.tick();
//            duck.start(); // FIXME once we have a robot, see if we need to call reverse for red or blue
//        }
//        drive.followTrajectorySequence(seq2);
//    }
//}