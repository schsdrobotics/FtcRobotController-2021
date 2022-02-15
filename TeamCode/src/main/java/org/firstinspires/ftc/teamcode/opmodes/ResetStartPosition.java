package org.firstinspires.ftc.teamcode.opmodes;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonomousTemplate;

/**
 * This OpMode should only be used when wanting to test teleop.
 * It should not be used in a competition, since autonomous runs first.
 * Make sure that the robot has an initial heading of 90 degrees!
 */
@TeleOp(name = "ResetStartPosition")
public class ResetStartPosition extends OpMode {
    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void init() {
        AutonomousTemplate.teleOpStartPose = AutonomousTemplate.pose(0, 0, 90);
    }

    @Override
    public void loop() {}
}
