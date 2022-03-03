package org.firstinspires.ftc.teamcode.opmodes.autonomous.red;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.LiftHandler;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonomousTemplate;

/**
 * Backup #2 for duck side, in case of emergency.
 */
@RequiresApi(api = Build.VERSION_CODES.N)
@Autonomous(name="RedDuckPark", group="Red")
public class RedDuckPark extends AutonomousTemplate {
    Trajectory park;

    @Override
    protected Pose2d startPose() {
        return poseM(-35, -63.375, 90);
    }

    @Override
    public void initializeTrajectories() {
        park = drive.trajectoryBuilder(startPose())
            .lineTo(posM(-63.375, -33.375))
            .build();
    }

    @Override
    public void setup() {
        target = LiftHandler.Position.LOW;
        waitForStart();
    }

    @Override
    public void main() {
        drive.followTrajectory(park);
    }
}
