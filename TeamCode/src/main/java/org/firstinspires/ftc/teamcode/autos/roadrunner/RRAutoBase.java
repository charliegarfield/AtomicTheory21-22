package org.firstinspires.ftc.teamcode.autos.roadrunner;

import static org.firstinspires.ftc.teamcode.Constants.CAMERA_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.HOPPER_BOTTOM;
import static org.firstinspires.ftc.teamcode.Constants.HOPPER_TOP;
import static org.firstinspires.ftc.teamcode.Constants.LEVEL_1;
import static org.firstinspires.ftc.teamcode.Constants.LEVEL_2;
import static org.firstinspires.ftc.teamcode.Constants.LEVEL_3;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

public abstract class RRAutoBase extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    abstract Pose2d startingPose();
    abstract Vector2d hubPosition();
    abstract double hubAngle();
    abstract TrajectorySequence goToHub();
    abstract TrajectorySequence goToCarousel();
    abstract TrajectorySequence interruptableSpline();
    abstract TrajectorySequence interruptableStrafe();
    abstract TrajectorySequence goToWarehouse();
    abstract Color getColor();


    @Override
    public void runOpMode() throws InterruptedException {

        Carousel carousel = new Carousel(getColor());
        Lift lift = new Lift();
        Hopper hopper = new Hopper();
        Intake intake = new Intake();
        Webcam webcam = new Webcam();

        carousel.init(hardwareMap);
        lift.init(hardwareMap);
        hopper.init(hardwareMap);
        intake.init(hardwareMap);
        webcam.init(hardwareMap);

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startingPose());


        delay(500);

        int level = 3;
        LinkedList<Integer> levels = new LinkedList<>();
        // Make the level the most common one from the past 100 loops
        while (!isStarted() && !isStopRequested()) {
            levels.add(webcam.getShippingHubLevel());
            if (levels.size() > 100) {
                levels.removeFirst();
                telemetry.addData("Status","Initialized.");
            }
            telemetry.update();
        }
        level = AutoUtil.mostCommon(levels);
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
        drive.followTrajectorySequence(goToHub());
        hopper.hopper.setPosition(HOPPER_TOP);
        delay(1000);
        hopper.hopper.setPosition(HOPPER_BOTTOM);
        lift.goTo(0,0.8);
        drive.followTrajectorySequence(goToCarousel());
        carousel.turnCarouselSimple();
        delay(3000);
        carousel.carouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carousel.carouselMotor.setPower(0);
        drive.followTrajectorySequenceAsync(interruptableSpline());
        while (opModeIsActive() && webcam.calculateYaw(CAMERA_POSITION) == null && drive.isBusy()) {
            drive.update();
            // Wait for the camera to detect a duck
        }
        if (webcam.calculateYaw(CAMERA_POSITION) != null) {
            drive.turn(-webcam.calculateYaw(CAMERA_POSITION));
            telemetry.addData("Yaw", -webcam.calculateYaw(CAMERA_POSITION));
            telemetry.update();
        } else {
            drive.followTrajectorySequenceAsync(interruptableStrafe());
            while (opModeIsActive() && webcam.calculateYaw(CAMERA_POSITION) == null && drive.isBusy()) {
                drive.update();
                // Wait for the camera to detect a duck
            }
            if (webcam.calculateYaw(CAMERA_POSITION) != null) {
                drive.turn(-webcam.calculateYaw(CAMERA_POSITION));
                telemetry.addData("Yaw", -webcam.calculateYaw(CAMERA_POSITION));
                telemetry.update();
            }
        }
        intake.intakeMotor.setPower(.9);
        TrajectorySequence pickUpDuck = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(8)
                .build();
        drive.followTrajectorySequence(pickUpDuck);
        TrajectorySequence returnToHub = drive.trajectorySequenceBuilder(pickUpDuck.end())
                .setReversed(true)
                .splineTo(hubPosition(), hubAngle())
                // When you're .8 second away from the hub, put the lift up and stop the intake
                .addTemporalMarker(-.8, () -> {
                    lift.goTo(LEVEL_3, 0.8);
                    intake.intakeMotor.setPower(0);
                })
                .build();
        drive.followTrajectorySequence(returnToHub);
        hopper.hopper.setPosition(HOPPER_TOP);
        delay(1000);
        hopper.hopper.setPosition(HOPPER_BOTTOM);
        lift.goTo(0,0.8);
        telemetry.clear();
        telemetry.addData("Warehouse Pose", drive.getPoseEstimate());
        telemetry.update();
        drive.followTrajectorySequence(goToWarehouse());

    }


    public void delay(int time) {
        double startTime = runtime.milliseconds();
        while (runtime.milliseconds() - startTime < time && !isStopRequested()) {
            drive.update();
        }
    }
}
