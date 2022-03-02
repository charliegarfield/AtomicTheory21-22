package org.firstinspires.ftc.teamcode.autos.roadrunner;

import static org.firstinspires.ftc.teamcode.Constants.HOPPER_BOTTOM;
import static org.firstinspires.ftc.teamcode.Constants.HOPPER_TOP;
import static org.firstinspires.ftc.teamcode.Constants.LEVEL_1;
import static org.firstinspires.ftc.teamcode.Constants.LEVEL_2;
import static org.firstinspires.ftc.teamcode.Constants.LEVEL_3;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autos.AutoUtil;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.mechanism.Color;
import org.firstinspires.ftc.teamcode.mechanism.Hopper;
import org.firstinspires.ftc.teamcode.mechanism.Intake;
import org.firstinspires.ftc.teamcode.mechanism.Lift;
import org.firstinspires.ftc.teamcode.mechanism.Webcam;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.LinkedList;

@Autonomous(name="RoadRunner Cycle Auto Red", group="Autonomous")
public class CycleAutoRed extends CycleAutoBase {

    @Override
    Pose2d getStartPose() {
        return new Pose2d(12, -64, Math.toRadians(-90));
    }

    @Override
    Vector2d getHubVector() {
        return new Vector2d(-6, -40);
    }

    @Override
    Vector2d getWarehouseEntryVector() {
        return new Vector2d(16, -66);
    }

    @Override
    Vector2d getWarehouseExitVector() {
        return new Vector2d(45, -66);
    }

    @Override
    Vector2d getInsideWarehouseVector() {
        return new Vector2d(45, -66);
    }

    @Override
    double getHubAngle() {
        return Math.toRadians(110);
    }
}
