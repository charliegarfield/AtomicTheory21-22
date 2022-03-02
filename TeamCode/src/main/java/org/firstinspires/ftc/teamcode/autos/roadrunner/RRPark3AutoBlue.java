package org.firstinspires.ftc.teamcode.autos.roadrunner;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="RoadRunner Storage Hub Park Auto Blue", group="Autonomous")
public class RRPark3AutoBlue extends RRBlueAutoBase {

    @Override
    TrajectorySequence goToWarehouse() {
        return trajectorySequenceBuilder(goToHub().end())
                .setReversed(false)
                .forward(2)
                .splineTo(new Vector2d(-64, 40), Math.toRadians(270))
                .build();
    }
}
