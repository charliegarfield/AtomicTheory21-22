package org.firstinspires.ftc.teamcode.autos.roadrunner;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="RoadRunner Park 2 Auto Blue", group="Autonomous")
public class RRPark2AutoBlue extends RRBlueAutoBase {
    @Override
    TrajectorySequence goToWarehouse() {
        return trajectorySequenceBuilder(goToHub().end())
                .setReversed(false)
                .forward(2)
                .splineTo(new Vector2d(14, 48), Math.toRadians(0))
                .splineTo(new Vector2d(70, 48), Math.toRadians(0))
                .build();
    }
}
