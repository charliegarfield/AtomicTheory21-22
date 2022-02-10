package org.firstinspires.ftc.teamcode.autos.roadrunner;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="RoadRunner Carousel Auto Red", group="Autonomous")
public class RRPrimeAutoRed extends RRRedAutoBase {
    @Override
    TrajectorySequence goToWarehouse() {
        return trajectorySequenceBuilder(goToHub().end())
                .setReversed(false)
                .forward(2)
                .splineTo(new Vector2d(27, -67), Math.toRadians(0))
                .splineTo(new Vector2d(44, -67), Math.toRadians(0))
                .build();
    }
}
