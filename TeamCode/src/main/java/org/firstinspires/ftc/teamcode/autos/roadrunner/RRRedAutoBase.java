package org.firstinspires.ftc.teamcode.autos.roadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.mechanism.Color;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public abstract class RRRedAutoBase extends RRAutoBase{

    @Override
    Color getColor() {
        return Color.RED;
    }

    @Override
    Pose2d startingPose() {
        return new Pose2d(-36, -64, Math.toRadians(-90));
    }

    @Override
    Vector2d hubPosition() {
        return new Vector2d(-19, -41);
    }

    @Override
    double hubAngle() {
        return Math.toRadians(70);
    }

    @Override
    TrajectorySequence goToHub() {
        return trajectorySequenceBuilder(startingPose())
                .setReversed(true)
                .splineTo(hubPosition(), hubAngle())
                .build();
    }

    @Override
    TrajectorySequence goToCarousel() {
        return trajectorySequenceBuilder(goToHub().end())
                .splineToLinearHeading(new Pose2d(-60, -62, Math.toRadians(-280)), Math.toRadians(280))
                .build();
    }

    @Override
    TrajectorySequence interruptableSpline() {
        return trajectorySequenceBuilder(goToCarousel().end())
                .splineToLinearHeading(new Pose2d(-55, -58 , Math.toRadians(-90)), Math.toRadians(90))
                .build();
    }

    @Override
    TrajectorySequence interruptableStrafe() {
        return trajectorySequenceBuilder(interruptableSpline().end())
                .strafeLeft(10)
                .build();
    }
}
