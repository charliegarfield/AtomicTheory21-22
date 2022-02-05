package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.Constants.CAMERA_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.HOPPER_BOTTOM;
import static org.firstinspires.ftc.teamcode.Constants.HOPPER_TOP;
import static org.firstinspires.ftc.teamcode.Constants.LEVEL_1;
import static org.firstinspires.ftc.teamcode.Constants.LEVEL_2;
import static org.firstinspires.ftc.teamcode.Constants.LEVEL_3;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.mechanism.Color;
import org.firstinspires.ftc.teamcode.mechanism.Hopper;
import org.firstinspires.ftc.teamcode.mechanism.Intake;
import org.firstinspires.ftc.teamcode.mechanism.Lift;
import org.firstinspires.ftc.teamcode.opencv.DuckFinder;
import org.firstinspires.ftc.teamcode.opencv.ShippingElementRecognizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

@Autonomous(name="RoadRunner Carousel Auto Red", group="Autonomous")
public class RRPrimeAutoRed extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    SampleMecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        Carousel carousel = new Carousel(Color.RED);
        Lift lift = new Lift();
        Hopper hopper = new Hopper();
        Intake intake = new Intake();

        carousel.init(hardwareMap);
        lift.init(hardwareMap);
        hopper.init(hardwareMap);
        intake.init(hardwareMap);

        OpenCvWebcam webcam;
        OpenCvWebcam frontWebcam;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        2, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY); //Whether to split the container vertically or horizontally
        // Setup first camera
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), viewportContainerIds[0]);
        ShippingElementRecognizer pipeline = new ShippingElementRecognizer();
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
                // This will be called if the camera could not be opened
            }
        });

        // Second camera
        frontWebcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Front Webcam"), viewportContainerIds[1]);
        DuckFinder pipeline2 = new DuckFinder(78);
        frontWebcam.setPipeline(pipeline2);
        frontWebcam.setMillisecondsPermissionTimeout(2500);
        frontWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                frontWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // This will be called if the camera could not be opened
            }
        });

        drive.setPoseEstimate(new Pose2d(-36, -64, Math.toRadians(-90)));

        TrajectorySequence goToHub = drive.trajectorySequenceBuilder(new Pose2d(-36, -64, Math.toRadians(-90)))
//                .strafeLeft(1.5)
//                .back(12)
//                .turn(Math.toRadians(-40))
//                .back(19)
//                .build();
                .setReversed(true)
                .splineTo(new Vector2d(-19, -41), Math.toRadians(70))
                .build();
        TrajectorySequence goToCarousel = drive.trajectorySequenceBuilder(goToHub.end())
//                .forward(28)
//                .turn(Math.toRadians(130))
//                .strafeRight(3.35)
//                .back(15)
//                .build();
                .splineToLinearHeading(new Pose2d(-62, -63, Math.toRadians(-280)), Math.toRadians(280))
                .build();
        TrajectorySequence interruptableSpline = drive.trajectorySequenceBuilder(goToCarousel.end())
                .splineToLinearHeading(new Pose2d(-55, -58 , Math.toRadians(-90)), Math.toRadians(90))
                .build();
        TrajectorySequence goToWarehouse = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(-24, -37), Math.toRadians(225)))
                .setReversed(false)
                .forward(2)
                .splineTo(new Vector2d(27, -67), Math.toRadians(0))
                .splineTo(new Vector2d(44, -67), Math.toRadians(0))
                .build();


        delay(500);

        int level = 3;
        LinkedList<Integer> levels = new LinkedList<>();
        // Make the level the most common one from the past 100 loops
        while (!isStarted() && !isStopRequested()) {
            levels.add(pipeline.getShippingHubLevel());
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
        drive.followTrajectorySequence(goToHub);
        hopper.hopper.setPosition(HOPPER_TOP);
        delay(1000);
        hopper.hopper.setPosition(HOPPER_BOTTOM);
        lift.goTo(0,0.8);
        drive.followTrajectorySequence(goToCarousel);
        carousel.turnCarouselSimple();
        delay(3000);
        carousel.carouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carousel.carouselMotor.setPower(0);
        drive.followTrajectorySequenceAsync(interruptableSpline);
        while (opModeIsActive() && pipeline2.calculateYaw(CAMERA_POSITION) == null && drive.isBusy()) {
            drive.update();
            if (pipeline2.calculateYaw(CAMERA_POSITION) != null) {
                telemetry.addData("Yaw", -pipeline2.calculateYaw(CAMERA_POSITION));
            }
            // Wait for the camera to detect a duck
        }
        if (pipeline2.calculateYaw(CAMERA_POSITION) != null) {
            drive.turn(-pipeline2.calculateYaw(CAMERA_POSITION));
            telemetry.addData("Yaw", -pipeline2.calculateYaw(CAMERA_POSITION));
            telemetry.update();
        }
        intake.intakeMotor.setPower(.9);
        TrajectorySequence pickUpDuck = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(8)
                .build();
        drive.followTrajectorySequence(pickUpDuck);
        TrajectorySequence returnToHub = drive.trajectorySequenceBuilder(pickUpDuck.end())
                .setReversed(true)
                .splineTo(new Vector2d(-18, -40), Math.toRadians(70))
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
        drive.followTrajectorySequence(goToWarehouse);

    }

    public void delay(int time) {
        double startTime = runtime.milliseconds();
        while (runtime.milliseconds() - startTime < time && !isStopRequested()) {
            drive.update();
        }
    }
}
