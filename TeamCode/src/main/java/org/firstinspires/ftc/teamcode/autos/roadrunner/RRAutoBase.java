package org.firstinspires.ftc.teamcode.autos.roadrunner;

import static org.firstinspires.ftc.teamcode.Constants.CAMERA_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.HOPPER_BOTTOM;
import static org.firstinspires.ftc.teamcode.Constants.HOPPER_TOP;
import static org.firstinspires.ftc.teamcode.Constants.LEVEL_1;
import static org.firstinspires.ftc.teamcode.Constants.LEVEL_2;
import static org.firstinspires.ftc.teamcode.Constants.LEVEL_3;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autos.AutoUtil;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.mechanism.Color;
import org.firstinspires.ftc.teamcode.mechanism.Hopper;
import org.firstinspires.ftc.teamcode.mechanism.HopperContents;
import org.firstinspires.ftc.teamcode.mechanism.Intake;
import org.firstinspires.ftc.teamcode.mechanism.Lift;
import org.firstinspires.ftc.teamcode.mechanism.Webcam;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

import java.util.Arrays;
import java.util.LinkedList;

public abstract class RRAutoBase extends LinearOpMode {
    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    ElapsedTime runtime = new ElapsedTime();
    abstract Pose2d startingPose();
    abstract Vector2d hubPosition();
    abstract double hubAngle();
    abstract TrajectorySequence goToHub();
    abstract TrajectorySequence goToCarousel();
    abstract TrajectorySequence interruptableSpline();
    abstract TrajectorySequence interruptableStrafe();
    abstract TrajectorySequence goToWarehouse();
    abstract Color getColor();
    Carousel carousel = new Carousel(getColor());
    Lift lift = new Lift();
    Hopper hopper = new Hopper();
    Intake intake = new Intake();
    Webcam webcam = new Webcam();


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);



        carousel.init(hardwareMap);
        lift.init(hardwareMap);
        hopper.init(hardwareMap);
        intake.init(hardwareMap);
        webcam.init(hardwareMap);

        drive.setPoseEstimate(startingPose());

        delay(500);

        int level;
        LinkedList<Integer> levels = new LinkedList<>();
        // Make the level the most common one from the past 100 loops
        while (!isStarted() && !isStopRequested()) {
            levels.add(webcam.getShippingHubLevel());
            if (levels.size() > 100) {
                levels.removeFirst();
            }
            telemetry.addData("Level", AutoUtil.mostCommon(levels));
            if (levels.size() < 30){
                telemetry.addData("Confidence", "Low");
            } else if (levels.size() < 60){
                telemetry.addData("Confidence", "Medium");
            } else {
                telemetry.addData("Confidence", "High");
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
        delay(1300);
        hopper.hopper.setPosition(HOPPER_BOTTOM);
        drive.followTrajectorySequence(goToCarousel());
        runtime.reset();
        carousel.regenerateProfile();
        carousel.timer.reset();
        while (!carousel.turnCarousel() && opModeIsActive() && runtime.seconds() < 5);
        carousel.carouselMotor.setPower(0);
        delay(1000);
//        carousel.turnCarouselSimple();
//        delay(3000);
//        carousel.carouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        carousel.carouselMotor.setPower(0);
        drive.followTrajectorySequenceAsync(interruptableSpline());
        while (opModeIsActive() && drive.isBusy()) {
            drive.update();
            // Wait for the camera to detect a duck
        }
        if (webcam.calculateYaw(CAMERA_POSITION) != null) {
            delay(500);
            if (webcam.calculateYaw(CAMERA_POSITION) != null){
                telemetry.addData("Yaw", -webcam.calculateYaw(CAMERA_POSITION));
                drive.turn(-webcam.calculateYaw(CAMERA_POSITION));
                telemetry.update();
            }
        } else {
            drive.followTrajectorySequenceAsync(interruptableStrafe());
            while (opModeIsActive() && webcam.calculateYaw(CAMERA_POSITION) == null && drive.isBusy()) {
                drive.update();
                // Wait for the camera to detect a duck
            }
            if (webcam.calculateYaw(CAMERA_POSITION) != null) {
                delay(1000);
                if (webcam.calculateYaw(CAMERA_POSITION) != null){
                    telemetry.addData("Yaw", -webcam.calculateYaw(CAMERA_POSITION));
                    drive.turn(-webcam.calculateYaw(CAMERA_POSITION));
                    telemetry.update();
                }
            }
        }
        intake.intakeMotor.setPower(.9);
        TrajectorySequence pickUpDuck = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(10)
                .build();
        drive.followTrajectorySequence(pickUpDuck);
        runtime.reset();
        while(hopper.contents() == HopperContents.EMPTY && runtime.seconds() < 3) {
            drive.update();
        }
        TrajectorySequence returnToHub = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setReversed(true)
                .splineTo(hubPosition(), hubAngle())
                // When you're .5 second away from the hub, put the lift up and stop the intake
                .addTemporalMarker(-.5, () -> {
                    lift.goTo(LEVEL_3, 0.8);
                    intake.intakeMotor.setPower(0);
                })
                .build();
        drive.followTrajectorySequence(returnToHub);
        delay(100);
        hopper.hopper.setPosition(HOPPER_TOP);
        delay(1300);
        hopper.hopper.setPosition(HOPPER_BOTTOM);
        lift.goTo(0,0.8);
        telemetry.clear();
        telemetry.addData("Warehouse Pose", drive.getPoseEstimate());
        telemetry.update();
        drive.followTrajectorySequence(goToWarehouse());
    }

    public void delay(int time) {
        double startTime = runtime.milliseconds();
        //noinspection StatementWithEmptyBody
        while (runtime.milliseconds() - startTime < time && !isStopRequested()) {
        }
    }
}
