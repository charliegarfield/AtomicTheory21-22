package com.atomictheory.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.ExtensionsKt;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {

    public static void main(String[] args) {
        MeepMeep mm = new MeepMeep(800);
        Constraints constraints = new Constraints(40, 30, Math.toRadians(180), Math.toRadians(180), 12.8);
        Constraints cephConstraints = new Constraints(60, 60, Math.toRadians(180), Math.toRadians(180), 11.5);

        RoadRunnerBotEntity blueCycleBot = new DefaultBotBuilder(mm)
                .setDimensions(11.95, 13.8)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(cephConstraints)
                .followTrajectorySequence(drive ->

                        drive.trajectorySequenceBuilder(new Pose2d(45, 64, Math.toRadians(0)))
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(12, 64), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-6, 40), Math.toRadians(-110))
                                .splineToConstantHeading(new Vector2d(12, 64), Math.toRadians(0))
                                .splineTo(new Vector2d(45, 64), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(12, 64), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-6, 40), Math.toRadians(-110))
                                .splineToConstantHeading(new Vector2d(12, 64), Math.toRadians(0))
                                .splineTo(new Vector2d(45, 64), 0)
                                .splineToConstantHeading(new Vector2d(12, 64), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-6, 40), Math.toRadians(-110))
                                .splineToConstantHeading(new Vector2d(12, 64), Math.toRadians(0))
                                .splineTo(new Vector2d(45, 64), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(12, 64), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-6, 40), Math.toRadians(-110))
                                .splineToConstantHeading(new Vector2d(12, 64), Math.toRadians(0))
                                .splineTo(new Vector2d(45, 64), 0)
                                .build());


        RoadRunnerBotEntity blueCarouselBot = new DefaultBotBuilder(mm)
                .setDimensions(11.95, 13.8)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(constraints)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-36, 64, Math.toRadians(90)))
                        .waitSeconds(2)
                        .setReversed(true)
                        .splineTo(new Vector2d(-18, 40), Math.toRadians(-70))
                        .waitSeconds(1)
                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(-62, 63, Math.toRadians(330)), Math.toRadians(110))
                        .waitSeconds(1)
                        .splineToLinearHeading(new Pose2d(-55, 58, Math.toRadians(90)), Math.toRadians(-90))
                        .forward(8)
                        .setReversed(true)
                        .splineTo(new Vector2d(-18, 40), Math.toRadians(-70))
                        .waitSeconds(1)
                        .setReversed(false)
                        .splineTo(new Vector2d(-60, 36), Math.toRadians(270))
                        .build());
        RoadRunnerBotEntity redCycleBot = new DefaultBotBuilder(mm)
                .setDimensions(11.95, 13.8)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(cephConstraints)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(12,-64, Math.toRadians(0)))

                                // Pre-load
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-6, -48), Math.toRadians(110))
                                .waitSeconds(0.5)

                                // 1st cycle
                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(12, -64), Math.toRadians(0))
                                .splineTo(new Vector2d(24,-64), Math.toRadians(0))
                                .splineTo(new Vector2d(44,-64), Math.toRadians(0))

                                .lineTo(new Vector2d(12, -64))
                                .splineToConstantHeading(new Vector2d(-6, -48), Math.toRadians(110))

                                // 2nd cycle
                                .splineToConstantHeading(new Vector2d(12, -64), Math.toRadians(0))
                                .splineTo(new Vector2d(24,-64), Math.toRadians(0))
                                .splineTo(new Vector2d(44,-64), Math.toRadians(0))

                                .lineTo(new Vector2d(12, -64))
                                .splineToConstantHeading(new Vector2d(-6, -48), Math.toRadians(110))

                                // 3rd cycle
                                .splineToConstantHeading(new Vector2d(12, -64), Math.toRadians(0))
                                .splineTo(new Vector2d(24,-64), Math.toRadians(0))
                                .splineTo(new Vector2d(44,-64), Math.toRadians(0))

                                .lineTo(new Vector2d(12, -64))
                                .splineToConstantHeading(new Vector2d(-6, -48), Math.toRadians(110))

                                // 4th cycle
                                .splineToConstantHeading(new Vector2d(12, -64), Math.toRadians(0))
                                .splineTo(new Vector2d(24,-64), Math.toRadians(0))
                                .splineTo(new Vector2d(36,-64), Math.toRadians(0))
                                .splineTo(new Vector2d(46, -58), Math.toRadians(50))

                                .setReversed(true)
                                .splineTo(new Vector2d(34, -64), Math.toRadians(180))
                                .lineTo(new Vector2d(12, -64))
                                .splineToConstantHeading(new Vector2d(-6, -48), Math.toRadians(110))

                                // 5th cycle
                                .splineToConstantHeading(new Vector2d(12, -64), Math.toRadians(0))
                                .splineTo(new Vector2d(24,-64), Math.toRadians(0))
                                .splineTo(new Vector2d(38,-64), Math.toRadians(0))
                                .splineTo(new Vector2d(46, -58), Math.toRadians(50))

                                .setReversed(true)
                                .splineTo(new Vector2d(34, -64), Math.toRadians(180))
                                .lineTo(new Vector2d(12, -64))
                                .splineToConstantHeading(new Vector2d(-6, -48), Math.toRadians(110))

                                // 6th cycle
                                .splineToConstantHeading(new Vector2d(12, -64), Math.toRadians(0))
                                .splineTo(new Vector2d(24,-64), Math.toRadians(0))
                                .splineTo(new Vector2d(44,-64), Math.toRadians(0))

                                .lineTo(new Vector2d(12, -64))
                                .splineToConstantHeading(new Vector2d(-6, -48), Math.toRadians(110))

                                // Park
                                .splineToConstantHeading(new Vector2d(12, -64), Math.toRadians(0))
                                .splineTo(new Vector2d(24,-64), Math.toRadians(0))
                                .splineTo(new Vector2d(44,-64), Math.toRadians(0))
                .build());

        RoadRunnerBotEntity redCarouselBot = new DefaultBotBuilder(mm)
                .setDimensions(11.95, 13.8)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(constraints)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-36, -64, Math.toRadians(-90)))
                        .waitSeconds(2)
                        .setReversed(true)
                        .splineTo(new Vector2d(-28, -24), Math.toRadians(0))
                        .waitSeconds(1)
                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(-62, -63, Math.toRadians(-330)), Math.toRadians(-110))
                        .waitSeconds(1)
                        .splineToLinearHeading(new Pose2d(-55, -58, Math.toRadians(-90)), Math.toRadians(90))
                        .forward(8)
                        .setReversed(true)
                        .splineTo(new Vector2d(-28, -24), Math.toRadians(0))
                        .waitSeconds(1)
                        .setReversed(false)
                        .splineTo(new Vector2d(-60, -36), Math.toRadians(-270))
                        .build());
        mm
                .setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_OFFICIAL)
                // Set theme
                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                .addEntity(blueCycleBot)
                .addEntity(blueCarouselBot)
                .addEntity(redCycleBot)
                .addEntity(redCarouselBot)
                .start();
    }
}