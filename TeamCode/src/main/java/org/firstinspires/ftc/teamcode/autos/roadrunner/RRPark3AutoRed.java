package org.firstinspires.ftc.teamcode.autos.roadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="RoadRunner Storage Hub Auto Red", group="Autonomous")
public class RRPark3AutoRed extends RRRedAutoBase {

    @Override
    TrajectorySequence goToWarehouse() {
        return trajectorySequenceBuilder(goToHub().end())
                .setReversed(false)
                .forward(2)
                .splineTo(new Vector2d(-60, -40.5), Math.toRadians(90))
                .build();
    }
}
