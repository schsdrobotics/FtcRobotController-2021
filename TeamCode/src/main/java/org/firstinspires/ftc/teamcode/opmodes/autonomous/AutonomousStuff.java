package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class AutonomousStuff {
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
}
