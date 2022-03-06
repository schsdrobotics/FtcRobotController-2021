package org.firstinspires.ftc.teamcode.opmodes.autonomous.red;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Primary backup for warehouse side. Same as Warehouse without preload and with delay at the beginning.
 */
@RequiresApi(api = Build.VERSION_CODES.N)
@Autonomous(name="RedWarehousePreload", group="Red Warehouse")
public class RedWarehousePreload extends RedWarehouse {
    @Override
    protected int MAX_CYCLES() {
        return 0;
    }

    @Override
    public void main() {
        sleep(15000);
        super.main();
    }
}
