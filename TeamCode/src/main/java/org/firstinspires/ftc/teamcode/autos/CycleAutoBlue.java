package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.mechanism.Color;
import org.firstinspires.ftc.teamcode.mechanism.Hopper;
import org.firstinspires.ftc.teamcode.mechanism.Intake;
import org.firstinspires.ftc.teamcode.mechanism.Lift;
import org.firstinspires.ftc.teamcode.mechanism.Webcam;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
@Autonomous(name="RoadRunner Cycle Auto Blue", group="Autonomous")
public class CycleAutoBlue extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Carousel carousel = new Carousel(Color.BLUE);
        Lift lift = new Lift();
        Hopper hopper = new Hopper();
        Intake intake = new Intake();
        Webcam webcam = new Webcam();

        carousel.init(hardwareMap);
        lift.init(hardwareMap);
        hopper.init(hardwareMap);
        intake.init(hardwareMap);
        webcam.init(hardwareMap);

        int level = 3;

        drive.setPoseEstimate(new Pose2d(12, 64, Math.toRadians(90)));

        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(new Pose2d(12, 64, Math.toRadians(90)))
                .setReversed(true)
                .splineTo(new Vector2d(0, 37), Math.toRadians(-135))
                .build();

        TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(trajectory1.end())
                .setReversed(false)
                .splineTo(new Vector2d(44, 48), Math.toRadians(0))
                .build();
        TrajectorySequence trajectory3 = drive.trajectorySequenceBuilder(trajectory2.end())
                .setReversed(true)
                .splineTo(new Vector2d(0, 37), Math.toRadians(-135))
                .build();
        TrajectorySequence trajectory4 = drive.trajectorySequenceBuilder(trajectory3.end())
                .setReversed(false)
                .splineTo(new Vector2d(44, 48), Math.toRadians(0))
                .build();

        waitForStart();

        drive.followTrajectorySequence(trajectory1);
        lift.goTo(1450,0.8);
        delay(1200);
        hopper.hopper.setPosition(0.33);
        delay(1200);
        hopper.hopper.setPosition(0);
        lift.goTo(0,0.8);
        drive.followTrajectorySequence(trajectory2);
//        drive.followTrajectorySequence(trajectory3);
//        lift.goTo(1450,0.8);
//        delay(750);
//        hopper.hopper.setPosition(0.33);
//        delay(700);
//        hopper.hopper.setPosition(0);
//        lift.goTo(0,0.8);
//        drive.followTrajectorySequence(trajectory4);
    }

    public void delay(int time) {
        double startTime = runtime.milliseconds();
        while (runtime.milliseconds() - startTime < time) {
        }
    }
}
