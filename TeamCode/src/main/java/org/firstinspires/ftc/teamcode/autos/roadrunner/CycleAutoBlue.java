package org.firstinspires.ftc.teamcode.autos.roadrunner;



import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="RoadRunner Cycle Auto Blue", group="Autonomous")
public class CycleAutoBlue extends CycleAutoBase {

    @Override
    Pose2d getStartPose() {
        return new Pose2d(12, 64, Math.toRadians(90));
    }

    @Override
    Vector2d getHubVector() {
        return new Vector2d(-6, 40);
    }

    @Override
    Vector2d getWarehouseEntryVector() {
        return new Vector2d(16, 66);
    }

    @Override
    Vector2d getWarehouseExitVector() {
        return new Vector2d(45, 66);
    }

    @Override
    Vector2d getInsideWarehouseVector() {
        return new Vector2d(45, 66);
    }

    @Override
    double getHubAngle() {
        return Math.toRadians(-110);
    }
}