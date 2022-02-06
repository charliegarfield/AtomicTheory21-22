package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.Constants.HOPPER_BOTTOM;
import static org.firstinspires.ftc.teamcode.Constants.HOPPER_TOP;
import static org.firstinspires.ftc.teamcode.Constants.LEVEL_1;
import static org.firstinspires.ftc.teamcode.Constants.LEVEL_2;
import static org.firstinspires.ftc.teamcode.Constants.LEVEL_3;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
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
public class CycleAutoRed extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Carousel carousel = new Carousel(Color.RED);
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
        LinkedList<Integer> levels = new LinkedList<>();
        // Make the level the most common one from the past 100 loops
        while (!isStarted() && !isStopRequested()) {
            levels.add(webcam.getShippingHubLevel());
            if (levels.size() > 100) {
                levels.removeFirst();
            }
        }
        level = AutoUtil.mostCommon(levels);

        Pose2d startPose = new Pose2d(12, -64, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence goToHub = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(-6, -40), Math.toRadians(110))
                .build();
        TrajectorySequence enterWarehouse = drive.trajectorySequenceBuilder(goToHub.end())
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> lift.goTo(0, .8))
                .splineTo(new Vector2d(16, -66), Math.toRadians(0))
                .addTemporalMarker(() -> intake.intakeMotor.setPower(.8))
                .splineTo(new Vector2d(50, -66), Math.toRadians(0))
                .build();
        TrajectorySequence returnToHub = drive.trajectorySequenceBuilder(enterWarehouse.end())
                .setReversed(true)
                .splineTo(new Vector2d(16, -66), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    intake.intakeMotor.setPower(-.3);
                })
                .splineTo(new Vector2d(-6, -40), Math.toRadians(110))
                .UNSTABLE_addTemporalMarkerOffset(-1, () ->{
                    intake.intakeMotor.setPower(0);
                    lift.goTo(LEVEL_3,0.8);
                })
                .build();
        TrajectorySequence finishInWarehouse = drive.trajectorySequenceBuilder(goToHub.end())
                .setReversed(false)
                .splineTo(new Vector2d(16, -66), Math.toRadians(0))
                .splineTo(new Vector2d(40, -66), Math.toRadians(0))
                .build();

        waitForStart();

        if (level == 1) {
            lift.goTo(LEVEL_1, 0.8);
        } else if (level == 2) {
            lift.goTo(LEVEL_2, 0.8);
        } else if (level == 3) {
            lift.goTo(LEVEL_3, 0.8);
        } else {
            throw new IllegalStateException("Invalid shipping hub level: " + level);
        }
        drive.followTrajectorySequence(goToHub);
        hopper.hopper.setPosition(HOPPER_TOP);
        delay(1200);
        hopper.hopper.setPosition(HOPPER_BOTTOM);
        drive.followTrajectorySequence(enterWarehouse);
        drive.followTrajectorySequence(returnToHub);
        hopper.hopper.setPosition(HOPPER_TOP);
        delay(1200);
        hopper.hopper.setPosition(HOPPER_BOTTOM);
        drive.followTrajectorySequence(enterWarehouse);
        drive.followTrajectorySequence(returnToHub);
        hopper.hopper.setPosition(HOPPER_TOP);
        delay(1200);
        hopper.hopper.setPosition(HOPPER_BOTTOM);
        drive.followTrajectorySequence(enterWarehouse);
        drive.followTrajectorySequence(returnToHub);
        hopper.hopper.setPosition(HOPPER_TOP);
        delay(1200);
        hopper.hopper.setPosition(HOPPER_BOTTOM);
        drive.followTrajectorySequence(enterWarehouse);
        drive.followTrajectorySequence(returnToHub);
        hopper.hopper.setPosition(HOPPER_TOP);
        delay(1200);
        hopper.hopper.setPosition(HOPPER_BOTTOM);
        drive.followTrajectorySequence(finishInWarehouse);
    }


    public void delay(int time) {
        double startTime = runtime.milliseconds();
        while (runtime.milliseconds() - startTime < time && !isStopRequested()) {
        }
    }
}
