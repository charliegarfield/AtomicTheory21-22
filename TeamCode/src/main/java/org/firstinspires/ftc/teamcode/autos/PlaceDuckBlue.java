package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.Constants.HOPPER_BOTTOM;
import static org.firstinspires.ftc.teamcode.Constants.HOPPER_TOP;
import static org.firstinspires.ftc.teamcode.Constants.LEVEL_1;
import static org.firstinspires.ftc.teamcode.Constants.LEVEL_2;
import static org.firstinspires.ftc.teamcode.Constants.LEVEL_3;
import static org.firstinspires.ftc.teamcode.Constants.LIFT_SPEED;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.mechanism.Color;
import org.firstinspires.ftc.teamcode.mechanism.Hopper;
import org.firstinspires.ftc.teamcode.mechanism.Intake;
import org.firstinspires.ftc.teamcode.mechanism.Lift;
import org.firstinspires.ftc.teamcode.mechanism.chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.opencv.DuckFinder;
import org.firstinspires.ftc.teamcode.opencv.ShippingElementRecognizer;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.LinkedList;

@Autonomous(name = "Auto (Blue)", group = "Autonomous")
public class PlaceDuckBlue extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumChassis chassis = new MecanumChassis();
    private Carousel carousel = new Carousel(Color.BLUE);
    private Lift lift = new Lift();
    private Hopper hopper = new Hopper();
    private Intake intake = new Intake();
    OpenCvWebcam webcam;
    OpenCvWebcam frontWebcam;

    public void runOpMode() throws InterruptedException {
        chassis.init(hardwareMap);
        carousel.init(hardwareMap);
        lift.init(hardwareMap);
        hopper.init(hardwareMap);
        intake.init(hardwareMap);

        // Setup for multiple cameras
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

        int level = 3;
        LinkedList<Integer> levels = new LinkedList<>();
        // Make the level the most common one from the past 100 loops
        while (!isStarted() && !isStopRequested()) {
            levels.add(pipeline.getShippingHubLevel());
            if (levels.size() > 100) {
                levels.removeFirst();
            }
            if (levels.size() < 30){
                telemetry.addData("Confidence", "Low");
            } else if (levels.size() < 60){
                telemetry.addData("Confidence", "Medium");
            } else {
                telemetry.addData("Confidence", "High");
            }
        }
        level = AutoUtil.mostCommon(levels);

        waitForStart();
        // Start button is pressed

        telemetry.addData("Shipping Hub Level", level);
        telemetry.update();

        // Drive to the the shipping hub
        chassis.moveBackwardWithEncoders(0.6,100);
        delay(350);
        chassis.strafeRightWithEncoders(0.5,1050);
        delay(350);
        chassis.moveBackwardWithEncoders(0.6,650);
        delay(100);

        // Deposit the box on the correct level
        if(level == 1) {
            lift.goTo(LEVEL_1,LIFT_SPEED);
            delay(500);
        } else if (level == 2) {
            lift.goTo(LEVEL_2,LIFT_SPEED);
            delay(700);
        } else {
            lift.goTo(LEVEL_3, LIFT_SPEED);
            delay(1000);
        }
        hopper.hopper.setPosition(HOPPER_TOP-0.05);
        delay(1200);
        hopper.hopper.setPosition(HOPPER_BOTTOM);
        delay(200);
        lift.goTo(0,LIFT_SPEED);

        // Move to the carousel and spin it
        chassis.moveForwardWithEncoders(0.6,250);
        chassis.rotateToGlobalAngle(-90,0.6);
        chassis.moveBackwardWithEncoders(0.6,2200);
        chassis.moveBackwardWithEncoders(0.3,200);
        chassis.moveForwardWithEncoders(0.5,50);
        chassis.strafeLeftWithEncoders(0.3,350);
        delay(150);
        chassis.moveForwardWithEncoders(0.5,25);
        carousel.turnCarouselSimple();
        delay(3000);
        carousel.carouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carousel.carouselMotor.setPower(0);

        // Locate and move towards the duck
        chassis.strafeRightWithEncoders(0.6,50);
        chassis.moveBackwardWithEncoders(0.3,150);
        chassis.moveForwardWithEncoders(0.6,400);
        chassis.rotateToGlobalAngle(0,0.5);
        delay(100);
        int startPos = chassis.frontLeft.getCurrentPosition();
        Point initialPoint = pipeline2.getDuckCenter();
        if(initialPoint == null) {
            telemetry.addLine("No duck found (initially)");
        } else {
            telemetry.addData("Duck x position", initialPoint.x);
        }
        telemetry.update();
        if(initialPoint == null || initialPoint.x > 110) {
            chassis.frontLeft.setPower(0.5);
            chassis.frontRight.setPower(-0.5);
            chassis.backLeft.setPower(-0.5);
            chassis.backRight.setPower(0.5);
            while(pipeline2.getDuckCenter() == null || pipeline2.getDuckCenter().x > 140) {
                // Wait until the duck is even with the intake
                if(chassis.frontLeft.getCurrentPosition() - startPos > 1800 && pipeline2.getDuckCenter() == null) {
                    // If the robot failed to spin the carousel and/or the duck isn't there, only search for 1800 ticks
                    break;
                }
            }
            chassis.frontLeft.setPower(0);
            chassis.frontRight.setPower(0);
            chassis.backLeft.setPower(0);
            chassis.backRight.setPower(0);
        } else if(initialPoint.x < 70) {
            chassis.strafeLeftWithEncoders(0.3,150 - (int)initialPoint.x);
        }
        int deltaPos = chassis.frontLeft.getCurrentPosition() - startPos;

        // Pick up the duck
        intake.intakeMotor.setPower(0.9);
        chassis.moveForwardWithEncoders(0.2,600);
        delay(1000);

        // Place the duck
        chassis.moveBackwardWithEncoders(0.6,100);
        delay(200);
        //chassis.rotateToGlobalAngle(0,0.3);
        chassis.strafeRightWithEncoders(0.8,1700 - deltaPos); // Account for movement to get the duck
        delay(200);
        chassis.moveBackwardWithEncoders(0.6,700);
        intake.intakeMotor.setPower(0);
        lift.goTo(LEVEL_3,LIFT_SPEED);
        delay(700);
        hopper.hopper.setPosition(HOPPER_TOP);
        delay(1200);
        hopper.hopper.setPosition(HOPPER_BOTTOM);
        delay(200);
        lift.goTo(0,LIFT_SPEED);

        // Drive into the warehouse
        chassis.moveForwardWithEncoders(0.6,350);
        chassis.rotateToGlobalAngle(-90,0.5);
        chassis.moveForwardWithEncoders(0.6, 3000);

    }

    public void delay(int time) {
        double startTime = runtime.milliseconds();
        while (runtime.milliseconds() - startTime < time) {
        }
    }
}
