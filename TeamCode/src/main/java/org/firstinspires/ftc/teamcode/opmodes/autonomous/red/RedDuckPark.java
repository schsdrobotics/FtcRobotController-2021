package org.firstinspires.ftc.teamcode.opmodes.autonomous.red;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.LiftHandler;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonomousTemplate;

/**
 * Emergency backup for duck side; only parks in storage area.
 */
@RequiresApi(api = Build.VERSION_CODES.N)
@Autonomous(name="RedDuckPark", group="Red Duck")
public class RedDuckPark extends AutonomousTemplate {
    Trajectory forward1;
    Trajectory align;
    Trajectory forward2;

    @Override
    protected Pose2d startPose() {
        return poseM(-35, -63.375, 90);
    }

    @Override
    public void initializeTrajectories() {
        forward1 = drive.trajectoryBuilder(startPose())
            .forward(11)
            .build();
        align = drive.trajectoryBuilder(forward1.end(), SampleMecanumDrive.getVelocityConstraint(17, rad(180), 13.7))
                .strafeLeft(48 * multiplier())
                .build();
        forward2 = drive.trajectoryBuilder(align.end())
                .forward(18)
                .build();
    }

    @Override
    public void setup() {
        target = LiftHandler.Position.LOW;
        waitForStart();
    }

    @Override
    public void main() {
        drive.followTrajectory(forward1);
        drive.followTrajectory(align);
        drive.followTrajectory(forward2);
    }
}
