package org.firstinspires.ftc.teamcode.opmodes.autonomous.red;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * A modified Warehouse optimized for the Sea Lion Techs.
 */
@RequiresApi(api = Build.VERSION_CODES.N)
@Autonomous(name="RedSeaLion", group="Red Warehouse")
public class RedSeaLion extends RedWarehouse {
    @Override
    protected int MAX_CYCLES() {
        return 2;
    }

    @Override
    protected boolean shouldWaitForSeaLions() {
        return true;
    }
}
