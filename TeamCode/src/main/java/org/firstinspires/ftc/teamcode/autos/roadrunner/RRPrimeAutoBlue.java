package org.firstinspires.ftc.teamcode.autos.roadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class RRPrimeAutoBlue extends RRBlueAutoBase{

    @Override
    TrajectorySequence goToWarehouse() {
        return drive.trajectorySequenceBuilder(new Pose2d(-24, 37, Math.toRadians(-45)))
                .setReversed(false)
                .forward(2)
                .splineTo(new Vector2d(30, 64), Math.toRadians(0))
                .splineTo(new Vector2d(44, 64), Math.toRadians(0))
                .build();
    }
}
