package org.firstinspires.ftc.teamcode.opmodes.autonomous.blue;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.red.RedSeaLion;

/**
 * A modified Warehouse optimized for the Sea Lion Techs.
 */
@RequiresApi(api = Build.VERSION_CODES.N)
@Autonomous(name="BlueSeaLion", group="Blue Warehouse")
public class BlueSeaLion extends RedSeaLion {
    @Override
    protected int multiplier() {
        return -1;
    }
}
