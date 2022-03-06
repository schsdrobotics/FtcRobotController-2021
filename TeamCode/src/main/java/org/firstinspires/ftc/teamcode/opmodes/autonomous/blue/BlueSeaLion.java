package org.firstinspires.ftc.teamcode.opmodes.autonomous.blue;

import android.os.Build;

import androidx.annotation.RequiresApi;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.red.RedSeaLion;

@RequiresApi(api = Build.VERSION_CODES.N)
public class BlueSeaLion extends RedSeaLion {
    @Override
    protected int multiplier() {
        return -1;
    }
}
