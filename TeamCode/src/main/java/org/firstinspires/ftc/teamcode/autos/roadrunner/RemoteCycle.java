package org.firstinspires.ftc.teamcode.autos.roadrunner;

import static org.firstinspires.ftc.teamcode.Constants.CAMERA_POSITION;
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

@Autonomous(name="Remote Cycle Auto", group="Autonomous")
public class RemoteCycle extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime intakeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    Carousel carousel = new Carousel(Color.RED);
    Lift lift = new Lift();
    Hopper hopper = new Hopper();
    Intake intake = new Intake();
    Webcam webcam = new Webcam();

    Pose2d getStartPose() {
        return new Pose2d(12, -64, Math.toRadians(-90));
    }
    Vector2d getHubVector() {
        return new Vector2d(-6, -40);
    }
    Vector2d getWarehouseEntryVector() {
        return new Vector2d(16, -66);
    }
    Vector2d getWarehouseExitVector() {
        return new Vector2d(42, -66);
    }
    Vector2d getInsideWarehouseVector() {
        return new Vector2d(45, -66);
    }
    Vector2d[] getWarehouseIntakeVectors() {
        return new Vector2d[] {
                new Vector2d(44.5, -66),
                new Vector2d(46,-66),
                new Vector2d(45,-64),
                new Vector2d(46,-64),
                new Vector2d(47,-66)
        };
    }
    double[] getWarehouseIntakeAngles() {
        return new double[] {
                Math.toRadians(0),
                Math.toRadians(0),
                Math.toRadians(40),
                Math.toRadians(40),
                Math.toRadians(0)
        };
    }
    double getHubAngle() {
        return Math.toRadians(120);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        carousel.init(hardwareMap);
        lift.init(hardwareMap);
        hopper.init(hardwareMap);
        intake.init(hardwareMap);
        webcam.init(hardwareMap);
        webcam.switchToFreightPipeline();

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
        int level = AutoUtil.mostCommon(levels);

        drive.setPoseEstimate(getStartPose());

        TrajectorySequence goToHub = drive.trajectorySequenceBuilder(getStartPose())
                .setReversed(true)
                .splineTo(getHubVector(), getHubAngle())
                .addTemporalMarker(1, () -> hopper.hopper.setPosition(HOPPER_TOP))
                .build();
        TrajectorySequence[] enterWarehouseSequences = new TrajectorySequence[5];
        for(int i = 0; i < enterWarehouseSequences.length; i++) {
            enterWarehouseSequences[i] = drive.trajectorySequenceBuilder(goToHub.end())
                    .setReversed(false)
                    .addTemporalMarker(() -> intake.intakeMotor.setPower(0))
                    .addTemporalMarker(1, () -> lift.goTo(0, .8))
                    .splineTo(getWarehouseEntryVector(), Math.toRadians(0))
                    .addTemporalMarker(() -> intake.intakeMotor.setPower(.8))
                    .splineTo(getWarehouseIntakeVectors()[i], getWarehouseIntakeAngles()[i])
                    .build();
        }
        TrajectorySequence finishInWarehouse = drive.trajectorySequenceBuilder(goToHub.end())
                .setReversed(false)
                .addTemporalMarker(() -> intake.intakeMotor.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> lift.goTo(0, .8))
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
        //hopper.hopper.setPosition(HOPPER_TOP);
        //waitForEmpty(drive, hopper);
        delay(100, drive);
        hopper.hopper.setPosition(HOPPER_BOTTOM);
        for (int i = 0; i < 3 && runtime.seconds() < 23 && !isStopRequested(); i++) {
            drive.followTrajectorySequence(enterWarehouseSequences[i]);
            //waitForFullOrDone(drive, hopper);
            /*drive.ping();
            delay(50, drive);
            drive.relocalize();*/
            TrajectorySequence returnToHub = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .setReversed(true)
                    .splineTo(getWarehouseExitVector(), Math.toRadians(180))
                    .splineTo(getWarehouseEntryVector(), Math.toRadians(180))
                    .splineTo(getHubVector(), getHubAngle())
                    .addTemporalMarker(0.7, () -> intake.intakeMotor.setPower(-.7))
                    .addTemporalMarker(1, () -> lift.goTo(LEVEL_3,0.8))
                    .addTemporalMarker(3, () -> hopper.hopper.setPosition(HOPPER_TOP))
                    .build();
            drive.followTrajectorySequence(returnToHub);
            //hopper.hopper.setPosition(HOPPER_TOP);
            //waitForEmpty(drive, hopper);
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
        /*telemetry.addData("UltrasonicX", drive.distanceSensorLocalizer.getPoseEstimate().getX());
        telemetry.addData("UltrasonicY", drive.distanceSensorLocalizer.getPoseEstimate().getY());
        telemetry.update();*/
    }


    public void delay(int time, SampleMecanumDrive drive) {
        double startTime = runtime.milliseconds();
        while (runtime.milliseconds() - startTime < time && !isStopRequested()) {
            drive.update();
        }
    }
}