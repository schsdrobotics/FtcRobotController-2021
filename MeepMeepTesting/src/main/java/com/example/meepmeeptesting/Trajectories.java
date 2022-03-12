package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.function.Function;

public enum Trajectories implements Function<DriveShim, TrajectorySequence> {
    RED_DUCK_STORAGE(drive -> drive.trajectorySequenceBuilder(pose(-37, -63.375, 90))
            .splineToConstantHeading(pos(-59,-40), rad(90))
            .lineToConstantHeading(pos(-59, -25))
            .splineToSplineHeading(pose(-32, -21, 180), 0)
            .waitSeconds(0)
            .splineToSplineHeading(pose(-52, -19, 270), rad(180))
            .splineToConstantHeading(pos(-75, -37), rad(270))
            .forward(26)
            .waitSeconds(1.5)
            .forward(-20)
            .build()),
    RED_DUCK_WAREHOUSE(drive -> drive.trajectorySequenceBuilder(pose(-35, -61.375, 90))
            .splineToConstantHeading(pos(-57,-38), rad(90))
            .lineToConstantHeading(pos(-57, -23))
            .splineToSplineHeading(pose(-30, -19, 180), 0)
            .splineToSplineHeading(pose(-50, -17, 270), rad(180))
            .splineToConstantHeading(pos(-77, -35), rad(270))
            .forward(26)
            .lineToSplineHeading(pose(-55, -40, 0))
            .splineToConstantHeading(pos(-15, -64), rad(270))
            .strafeRight(10)
            .forward(65)
            .strafeLeft(10)
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
    RED_WAREHOUSE2(drive -> drive.trajectorySequenceBuilder(poseM(12, -63.375, 270))
//            .setReversed(true)
            //toHubInitial
            .lineToSplineHeading(poseM(-15, -37, 290))
            .addTemporalMarker(1, -0.6, () -> {
                //Drop and retract
//                currentCycle.finish();
            })
            .addTemporalMarker(1, -0.2, () -> {
                //Cancel early to make it faster
//                cancelAndStop();
            })
            .waitSeconds(0)
//            //align(kinda)
//            .splineToSplineHeading(pose(-2, -60, 0), rad(290))
//            .splineToConstantHeading(pos(12, -71), rad(0))
//            //toWarehouse
//            .forward(40)
//            .setReversed(true)
//            .setVelConstraint(getVelocityConstraint(1000, 1, 13.7))
//            .strafeLeft(8)
//            .resetVelConstraint()
//            //toHub
//            .lineTo(pos(12, -64))
//            .splineToConstantHeading(pos(-2, -54), rad(110))
//            .splineToSplineHeading(pose(-5, -42, 280), rad(100))

            //align(kinda)
            .splineToSplineHeading(poseM(-2, -60, 0), radM(290))
            .splineToConstantHeading(posM(12, -71), radM(0))
            .splineToConstantHeading(posM(54, -73), radM(0))
            .addTemporalMarker(1, -1.1, () -> {
//                currentCycle.start();
            })
            .addTemporalMarker(1, -0.7, () -> {
//                drive.cancelFollowing();
            })
            .addTemporalMarker(1, -0.3, () -> {
                //Cancel following
            })

            .waitSeconds(0)
            .splineToConstantHeading(posM(54, -65.375), radM(180))
            .splineToConstantHeading(posM(20, -65.375), radM(180))
            .splineToConstantHeading(posM(10, -62), radM(180))
            .splineTo(posM(-11, -35), radM(110))
            .addTemporalMarker(1, -0.7, () -> {
                //Drop and retract
//                currentCycle.finish();
            })
            .addTemporalMarker(1, -0.3, () -> {
                //Cancel early to make it faster
//                cancelAndStop();
            })
//            .lineToConstantHeading(pos(30, -71))
//            .splineToConstantHeading(pos(50, -67), rad(0))
//            .lineToConstantHeading(pos(60, -67))
//            .setVelConstraint(getVelocityConstraint(1e6, 1e6, 13.7))
//            .setAccelConstraint(getAccelerationConstraint(1e6))
//            .lineToLinearHeading(pose(48, -65.375, 0))
//            .resetConstraints()
//            .setReversed(true)
//            .splineToConstantHeading(pos(10, -65.375), rad(180))
//            .splineToConstantHeading(pos(0, -62), rad(180))
//            .splineTo(pos(-10, -34), rad(100))

//            .splineTo(pos(48, -67), rad(15))
            //park
//            .forward(-6)
//            .splineToConstantHeading(pos(42, -38), rad(0))
//            .lineToSplineHeading(pose(60, -38, 270))
            .build()),
    RED_WAREHOUSE_PARK(drive -> drive.trajectorySequenceBuilder(pose(12, -63.375, 0))
            .lineTo(pos(40, -63.375))
            .lineTo(pos(35, -30))
            .lineToSplineHeading(pose(60, -30, 270))
            .build()),
    BLUE_DUCK_STORAGE(drive -> drive.trajectorySequenceBuilder(poseM(-37, -63.375, 90))
            .splineToConstantHeading(posM(-59,-40), radM(90))
            .lineToConstantHeading(posM(-59, -25))
            .splineToSplineHeading(poseM(-32, -21, 180), 0)
            .waitSeconds(0)
            .splineToSplineHeading(poseM(-52, -19, 270), radM(180))
            .splineToConstantHeading(posM(-75, -27), radM(270))
            // RESET POSE ESTIMATE HERE
            .forward(10)
            .splineToLinearHeading(poseM(-65.375 + 9.5, -63.375 + 6, 180), radM(180))
            .waitSeconds(1.5)

            ///////
            .lineToLinearHeading(poseM(-59,-35, 90))
            .build()),
    BLUE_DUCK_WAREHOUSE(drive -> drive.trajectorySequenceBuilder(poseM(-37, -63.375, 90))
            .splineToConstantHeading(posM(-59,-40), radM(90))
            .lineToConstantHeading(posM(-59, -25))
            .splineToSplineHeading(poseM(-32, -21, 180), 0)
            .waitSeconds(0)
            .splineToSplineHeading(poseM(-52, -19, 270), radM(180))
            .splineToConstantHeading(posM(-75, -27), radM(270))
            // RESET POSE ESTIMATE HERE
            .forward(10)
            //here
            .splineToLinearHeading(poseM(-65.375 + 9.5, -63.375 + 6, 180), radM(180))
            .waitSeconds(1.5)

            ///////
            .forward(-10)
            .lineToConstantHeading(posM(-65.375 + 9.5 + 20,-75))
            // RESET POSE ESTIMATE HERE
            .forward(-74)
            .strafeLeft(10)
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
    TEST(drive -> drive.trajectorySequenceBuilder(poseM(51, -65.375, 0))
            .setReversed(true)
            .splineToConstantHeading(posM(20, -65.375), radM(180))
            .splineToConstantHeading(posM(10, -62), radM(180))
            .splineTo(posM(-7, -35), radM(110))
            .addTemporalMarker(1, -0.8, () -> {
                //Drop and retract
//                currentCycle.finish();
            })
            .addTemporalMarker(1, -0.3, () -> {
                //Cancel early to make it faster
//                cancelAndStop();
            })
            .build()),
    TEST2(drive -> drive.trajectorySequenceBuilder(pose(0, -60, 270))
            .splineToSplineHeading(pose(-60, 0, 180), rad(0))
            .splineToSplineHeading(pose(0, 60, 90), rad(270))
            .splineToSplineHeading(pose(60, 0, 0), rad(180))
            .splineToSplineHeading(pose(0, -60, 270), rad(90))
            .build()),
    PICKUP_STRAFE(drive -> drive.trajectorySequenceBuilder(pose(0, 0, 0))
            .splineToConstantHeading(pos(12, -5), 0)
            .forward(5)
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

    public static double radM(double deg) {
        return Math.toRadians(deg) * -1;
    }

    public static Vector2d posM(double x, double y) {
        return new Vector2d(x, y * -1);
    }

    public static Vector2d posM(double pos) {
        return pos(pos, pos * -1);
    }

    public static Pose2d poseM(double x, double y, double deg) {
        return new Pose2d(x, y * -1, rad(deg * -1));
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

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    public static double multiplier = 1;

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
                .setBotDimensions(13.25, 17.25)
                .setConstraints(45 * multiplier, 45 * multiplier, rad(180) * multiplier, rad(180) * multiplier, 13.7)
                .followTrajectorySequence(TEST::apply)
                .start();
    }
}
