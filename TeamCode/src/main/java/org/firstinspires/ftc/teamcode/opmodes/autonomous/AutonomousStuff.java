package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CameraHandler;
import org.firstinspires.ftc.teamcode.LiftHandler;
import org.firstinspires.ftc.teamcode.LightHandler;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@RequiresApi(api = Build.VERSION_CODES.N)
public class AutonomousStuff {
    private static final ExecutorService executor = Executors.newSingleThreadExecutor();

    public static double rad(double deg) {
        return Math.toRadians(deg);
    }

    public static Vector2d pos(double x, double y) {
        return new Vector2d(x, y);
    }

    public static Vector2d pos(double pos) {
        return pos(pos, pos);
    }

    public static Pose2d pose(double x, double y, double deg) {
        return new Pose2d(x, y, rad(deg));
    }

    public static double calculatePoint(double x1, double y1, double x2, double y2, boolean x, double xory) {
        if (x) {
            double slope = ((y2-y1)/(x2-x1));
            return slope*(xory - x1) + y1;
        }
        else {
            double slope = ((x2 - x1)/(y2-y1));
            return slope * (xory - y1) + x1;
        }
    }

    public static LiftHandler.Position determineTarget(CameraHandler camera, float xCenter) {
        // Target will be high if there are no objects detected
        if (camera.mostConfident != null) {
            if (xCenter < 300) {
                //Set the target to low
                return LiftHandler.Position.LOW;
            }
            else if (xCenter < 600) {
                //Set the target to middle
                return LiftHandler.Position.MIDDLE;
            }
        }
        return LiftHandler.Position.HIGH;
    }

    public static void backgroundLoop(LinearOpMode opMode, SampleMecanumDrive drive, LightHandler light) {
        executor.submit(() -> {
            // Set up to blink robopandas in morse code
            light
                .pause()
                //R
                .dot().dash().dot().pause()
                //O
                .dash().dash().dash().pause()
                //B
                .dash().dot().dot().dot().pause()
                //O
                .dash().dash().dash().pause()
                //P
                .dot().dash().dash().dot().pause()
                //A
                .dot().dash().pause()
                //N
                .dash().dot().pause()
                //D
                .dash().dot().dot().pause()
                //A
                .dot().dash().pause()
                //S
                .dot().dot().dot();
            opMode.waitForStart();
            light.resetTimer();
            while (opMode.opModeIsActive() && !opMode.isStopRequested()) {
                drive.update();
                light.tick();
            }
            light.setColor(LightHandler.Color.OFF);
        });
    }
}
