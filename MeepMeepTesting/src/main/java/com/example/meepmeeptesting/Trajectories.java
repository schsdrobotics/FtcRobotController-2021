package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

import org.jetbrains.annotations.NotNull;

import java.util.function.Function;

public enum Trajectories implements Function<DriveShim, TrajectorySequence> {
    RED_DUCK_STORAGE(drive -> drive.trajectorySequenceBuilder(pose(-35, -61.375, 90))
            .splineToConstantHeading(pos(-57,-38), rad(90))
            .lineToConstantHeading(pos(-57, -28))
            .splineToSplineHeading(pose(-30, -24, 180), 0)
            .waitSeconds(0)
            .splineToSplineHeading(pose(-50, -22, 270), rad(180))
            .splineToConstantHeading(pos(-63.375, -35), rad(270))
            .forward(15)
            .waitSeconds(1.5)
            .forward(-15)
            .build()),
    RED_DUCK_WAREHOUSE(drive -> drive.trajectorySequenceBuilder(pose(-35, -61.375, 90))
            .splineToConstantHeading(pos(-57,-38), rad(90))
            .lineToConstantHeading(pos(-57, -28))
            .splineToSplineHeading(pose(-30, -24, 180), 0)
            .waitSeconds(0)
            .splineToSplineHeading(pose(-50, -22, 270), rad(180))
            .splineToConstantHeading(pos(-63.375, -35), rad(270))
            .forward(15)
//            .waitSeconds(1.5)
            .lineToSplineHeading(pose(-55, -48, 0))
            .splineToConstantHeading(pos(-15, -64), rad(270))
            .strafeRight(7)
            .forward(60)
            .build()),
    RED_WAREHOUSE(drive -> drive.trajectorySequenceBuilder(pose(12, -61.375, 90))
            .lineToLinearHeading(pose(-5, -42, 280))
            .splineTo(pos(12, -64), rad(0))
            .forward(40)
            .setReversed(true)
            .lineTo(pos(12, -64))
            .splineToSplineHeading(pose(-5, -42, 280), rad(100))
            .setReversed(false)
            .splineTo(pos(12, -64), rad(0))
            .forward(40)
            .forward(-6)
            .splineToConstantHeading(pos(42, -38), rad(0))
            .lineToSplineHeading(pose(60, -38, 270))
            .build()),
    RED_WAREHOUSE_PARK(drive -> drive.trajectorySequenceBuilder(pose(12, -61.375, 90))
            .splineToLinearHeading(pose(6, -55, 0), rad(270))
            .strafeRight(14)
            .addTemporalMarker(() -> drive.setPoseEstimate(pose(6, -63.375, 0)))
            .forward(27)
            .splineToConstantHeading(pos(44, -38), rad(0))
            .lineToSplineHeading(pose(60, -38, 270))
            .build()),
    BLUE_DUCK_STORAGE(drive -> drive.trajectorySequenceBuilder(pose(-35, 62, 270))
            .lineTo(pos(-12, 45))
            .addTemporalMarker(() -> {
                // drop initial cube
            })
            .waitSeconds(2)
            .lineToLinearHeading(pose(-60, 60, 0))
            .addTemporalMarker(() -> {
                // duck motor
            })
            .waitSeconds(2)
            .lineTo(pos(-60, 36)) // now in hub
            .build()),
    BLUE_DUCK_WAREHOUSE(drive -> drive.trajectorySequenceBuilder(pose(-35, 62, 270))
            .lineTo(pos(-12, 45))
            .addTemporalMarker(() -> {
                // drop initial cube
            })
            .waitSeconds(2)
            .lineToLinearHeading(pose(-60, 60, 0))
            .addTemporalMarker(() -> {
                // duck motor
            })
            .waitSeconds(2)
            .splineTo(pos(0, 62), rad(0))
            .forward(40)
            .build()),
    BLUE_WAREHOUSE(drive -> drive.trajectorySequenceBuilder(pose(12, 62, 270))
            .lineToLinearHeading(pose(-5, 42, -100))
            .addTemporalMarker(() -> {
                // drop initial cube
            })
            //.waitSeconds(2)
            .setReversed(true)
            .splineTo(pos(12, 62), rad(0))
            .forward(-30)
            .addTemporalMarker(() -> {
                // grab
            })
            //.waitSeconds(2)
            .forward(30)
            .splineTo(pos(-5, 42), rad(-100))
            .addTemporalMarker(() -> {
                // drop
            })
            //.waitSeconds(2)
            .setReversed(true)
            .splineTo(pos(12, 62), rad(0)) // now in storage
            .forward(-26)
            .strafeLeft(24)
            .lineToLinearHeading(pose(60, 38, 270))
            .build()),
    BLUE_WAREHOUSE_PARK(drive -> drive.trajectorySequenceBuilder(pose(12, 62, 270))
            .lineToLinearHeading(pose(0, 56, 225))
            .setReversed(true)
            .splineTo(pos(12, 62), rad(0)) // now in storage
            .forward(-26)
            .strafeLeft(24)
            .lineToLinearHeading(pose(60, 38, 270))
            .build()),
    TEST(drive -> drive.trajectorySequenceBuilder(pose(12, 62, 270))
            .lineToLinearHeading(pose(12+1e-5, 62, 270))
            .build()),
    REMOTE(drive -> drive.trajectorySequenceBuilder(pose(-35, -62, 90))
            .lineTo(pos(calculatePoint(-35, -62, -7, -40, false, -58), -58))
            .lineToSplineHeading(pose(-7, -40, 270))
            .lineToLinearHeading(pose(-61,-51, 245))
            .lineToSplineHeading(pose(-55, -48, 0))
            .splineToConstantHeading(pos(-15, -64), rad(270))
//            .strafeRight(5)
            .forward(60)
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

    public static double stupid = 1e4;

    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep mm = new MeepMeep(750)
                // Set field image
                .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
                // Set theme
                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45*stupid, 45*stupid, rad(180)*stupid, rad(180)*stupid, 13.7)
                .setBotDimensions(13.25, 17.25)
                .followTrajectorySequence(TEST::apply)
                .start();
    }
}
