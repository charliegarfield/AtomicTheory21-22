package org.firstinspires.ftc.teamcode.autos.roadrunner;

import static org.firstinspires.ftc.teamcode.Constants.HOPPER_BOTTOM;
import static org.firstinspires.ftc.teamcode.Constants.HOPPER_TOP;
import static org.firstinspires.ftc.teamcode.Constants.LEVEL_1;
import static org.firstinspires.ftc.teamcode.Constants.LEVEL_2;
import static org.firstinspires.ftc.teamcode.Constants.LEVEL_3;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autos.AutoUtil;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.mechanism.Color;
import org.firstinspires.ftc.teamcode.mechanism.Hopper;
import org.firstinspires.ftc.teamcode.mechanism.HopperContents;
import org.firstinspires.ftc.teamcode.mechanism.Intake;
import org.firstinspires.ftc.teamcode.mechanism.Lift;
import org.firstinspires.ftc.teamcode.mechanism.Webcam;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.LinkedList;

public abstract class CycleAutoBase extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime intakeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    Carousel carousel = new Carousel(Color.RED);
    Lift lift = new Lift();
    Hopper hopper = new Hopper();
    Intake intake = new Intake();
    Webcam webcam = new Webcam();

    abstract Pose2d getStartPose();
    abstract Vector2d getHubVector();
    abstract Vector2d getWarehouseEntryVector();
    abstract Vector2d getInsideWarehouseVector();
    abstract double getHubAngle();


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


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

        drive.setPoseEstimate(getStartPose());

        TrajectorySequence goToHub = drive.trajectorySequenceBuilder(getStartPose())
                .setReversed(true)
                .splineTo(getHubVector(), getHubAngle())
                .build();
        TrajectorySequence enterWarehouse = drive.trajectorySequenceBuilder(goToHub.end())
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> lift.goTo(0, .8))
                .splineTo(getWarehouseEntryVector(), Math.toRadians(0))
                .addTemporalMarker(() -> intake.intakeMotor.setPower(.8))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineTo(getInsideWarehouseVector(), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-.5, () -> drive.setWeightedDrivePower(new Pose2d(.2, 0, 0)))
                .build();
        TrajectorySequence returnToHub = drive.trajectorySequenceBuilder(enterWarehouse.end())
                .setReversed(true)
                .addTemporalMarker(() -> intake.intakeMotor.setPower(-.3))
                .splineTo(getWarehouseEntryVector(), Math.toRadians(180))
                .splineTo(getHubVector(), getHubAngle())
                .UNSTABLE_addTemporalMarkerOffset(-2, () ->{
                    intake.intakeMotor.setPower(0);
                    lift.goTo(LEVEL_3,0.8);
                })
                .build();
        TrajectorySequence finishInWarehouse = drive.trajectorySequenceBuilder(goToHub.end())
                .setReversed(false)
                .splineTo(getWarehouseEntryVector(), Math.toRadians(0))
                .splineTo(getInsideWarehouseVector(), Math.toRadians(0))
                .build();


        waitForStart();
        runtime.reset();
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
        waitForEmpty(drive, hopper);
        delay(200, drive);
        hopper.hopper.setPosition(HOPPER_BOTTOM);
        while (runtime.seconds() < 23 && !isStopRequested()) {
            drive.followTrajectorySequenceAsync(enterWarehouse);
            waitForFullOrDone(drive, hopper);
            intakeTimer.reset();
            while (hopper.contents() == HopperContents.EMPTY && intakeTimer.seconds() < 3 && !isStopRequested()) {
                drive.update();
            }
            while (hopper.contents() == HopperContents.EMPTY && !isStopRequested()) {
                intake.intakeMotor.setPower(-.3);
                drive.setWeightedDrivePower(new Pose2d(-.2, 0, 0));
                delay(600, drive);
                intake.intakeMotor.setPower(.8);
                drive.setWeightedDrivePower(new Pose2d(.3, 0, 0));
                delay(1000, drive);
            }
            drive.followTrajectorySequence(returnToHub);
            hopper.hopper.setPosition(HOPPER_TOP);
            waitForEmpty(drive, hopper);
            delay(200, drive);
            hopper.hopper.setPosition(HOPPER_BOTTOM);
        }
        drive.followTrajectorySequence(finishInWarehouse);
    }

    private void waitForEmpty(SampleMecanumDrive drive, Hopper hopper) {
        while (hopper.contents() != HopperContents.EMPTY && !isStopRequested()) {
            drive.update();
        }
    }

    private void waitForFullOrDone(SampleMecanumDrive drive, Hopper hopper) {
        while (hopper.contents() == HopperContents.EMPTY && drive.isBusy() && !isStopRequested()) {
            drive.update();
        }
    }


    public void delay(int time, SampleMecanumDrive drive) {
        double startTime = runtime.milliseconds();
        while (runtime.milliseconds() - startTime < time && !isStopRequested()) {
            drive.update();
        }
    }
}