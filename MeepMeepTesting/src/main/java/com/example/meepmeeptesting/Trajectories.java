package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

import org.jetbrains.annotations.NotNull;

import java.util.function.Function;

public enum Trajectories implements Function<DriveShim, TrajectorySequence> {
    RED_DUCK(drive -> drive.trajectorySequenceBuilder(pose(-35, -62, 90))
            .lineTo(pos(-12, -45))
            .lineToLinearHeading(pose(-60, -60, 180))
            .waitSeconds(2)
            .lineTo(pos(-60, -36))
            .build()),
    RED_WAREHOUSE(drive -> drive.trajectorySequenceBuilder(pose(12, -62, 90))
            .lineToLinearHeading(pose(-5, -42, 100))
            .setReversed(true)
            .splineTo(pos(12, -62), rad(0))
            .forward(-30)
            .forward(30)
            .splineTo(pos(-5, -42), rad(100))
            .setReversed(true)
            .splineTo(pos(12, -62), rad(0))
            .forward(-30)
            .build()),
    BLUE_DUCK(drive -> drive.trajectorySequenceBuilder(pose(-35, 62, 270))
            .lineTo(pos(-12, 45))
            .lineToLinearHeading(pose(-60, 60, 90))
            .waitSeconds(2)
            .lineTo(pos(-60, 36))
            .build()),
    BLUE_WAREHOUSE(drive -> drive.trajectorySequenceBuilder(pose(12, 62, 270))
            .lineToLinearHeading(pose(-5, 42, -100))
            .setReversed(true)
            .splineTo(pos(12, 62), rad(0))
            .forward(-30)
            .forward(30)
            .splineTo(pos(-5, 42), rad(-100))
            .setReversed(true)
            .splineTo(pos(12, 62), rad(0))
            .forward(-30)
            .build())
    ;

    private Function<DriveShim, TrajectorySequence> func;

    Trajectories(Function<DriveShim, TrajectorySequence> func) {
        this.func = func;
    }

    @NotNull
    @Override
    public TrajectorySequence apply(DriveShim driveShim) {
        return func.apply(driveShim);
    }

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

    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep mm = new MeepMeep(800)
                // Set field image
                .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
                // Set theme
                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(58.5, 58.5, rad(180), rad(180), 9.5)
                .followTrajectorySequence(RED_WAREHOUSE::apply)
                .start();
    }
}
