package org.firstinspires.ftc.teamcode.opmodes.autonomous.blue;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.red.RedDuckPark;

/**
 * Emergency backup for duck side; only parks in storage area.
 */
@RequiresApi(api = Build.VERSION_CODES.N)
@Autonomous(name="BlueDuckPark", group="Blue Duck")
public class BlueDuckPark extends RedDuckPark {
    @Override
    protected int multiplier() {
        return -1;
    }
}