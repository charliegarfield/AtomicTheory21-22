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

@Autonomous(name = "Auto (Red)", group = "Autonomous")
public class PlaceDuckRed extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumChassis chassis = new MecanumChassis();
    private Carousel carousel = new Carousel(Color.RED);
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

        waitForStart();
        // Start button is pressed

        // Get the placement of the shipping element 100 times and pick the most frequent position
        int level;
        int[] counts = {0,0,0};
        for(int i=0;i<50;i++) {
            delay(10);
            if(pipeline.getShippingHubLevel() == 0) {
                i = 0;
                continue;
            }
            counts[pipeline.getShippingHubLevel() - 1]++;
        }
        if(counts[0] > counts[1] && counts[0] > counts[2]) {
            level = 1;
        } else if(counts[1] > counts[0] && counts[1] > counts[2]) {
            level = 2;
        } else {
            level = 3;
        }
        telemetry.addData("Shipping Hub Level", level);
        telemetry.update();

        // Drive to the the shipping hub
        chassis.moveBackwardWithEncoders(0.6,100);
        delay(300);
        chassis.strafeLeftWithEncoders(0.6,1050);
        delay(300);
        chassis.moveBackwardWithEncoders(0.6,640);
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
        hopper.hopper.setPosition(HOPPER_TOP);
        delay(1200);
        hopper.hopper.setPosition(HOPPER_BOTTOM);
        delay(200);
        lift.goTo(0,LIFT_SPEED);

        // Move to the carousel and spin it
        chassis.moveForwardWithEncoders(0.6,700);
        chassis.rotateToGlobalAngle(-180,0.6);
        chassis.moveBackwardWithEncoders(0.3,400);
        chassis.moveForwardWithEncoders(0.6,150);
        delay(250);
        chassis.strafeLeftWithEncoders(0.6,1850);
        chassis.strafeLeftWithEncoders(0.3,300);
        delay(150);
        carousel.turnCarouselSimple();
        delay(3000);
        carousel.carouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carousel.carouselMotor.setPower(0);

        // Commented out for competition
        /*
        // Locate and move towards the duck
        chassis.strafeRightWithEncoders(0.6,50);
        chassis.moveForwardWithEncoders(0.6,200);
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
        if(initialPoint == null || initialPoint.x < 70) {
            chassis.frontLeft.setPower(-0.5);
            chassis.frontRight.setPower(0.5);
            chassis.backLeft.setPower(0.5);
            chassis.backRight.setPower(-0.5);
            while(pipeline2.getDuckCenter() == null || pipeline2.getDuckCenter().x < 30) {
                // Wait until the duck is even with the intake
                if(chassis.frontLeft.getCurrentPosition() - startPos < -1800 && pipeline2.getDuckCenter() == null) {
                    // If the robot failed to spin the carousel and/or the duck isn't there, only search for 1800 ticks
                    break;
                }
            }
            chassis.frontLeft.setPower(0);
            chassis.frontRight.setPower(0);
            chassis.backLeft.setPower(0);
            chassis.backRight.setPower(0);
        } else if(initialPoint.x > 110) {
            chassis.strafeRightWithEncoders(0.3,(int)initialPoint.x - 50);
        }
        int deltaPos = chassis.frontLeft.getCurrentPosition() - startPos;
        delay(200);

        // Pick up the duck
        intake.intakeMotor.setPower(0.9);
        chassis.moveForwardWithEncoders(0.2,600);
        delay(1000);

        // Place the duck
        chassis.moveBackwardWithEncoders(0.6,100);
        delay(200);
        chassis.strafeLeftWithEncoders(0.6,2050 + deltaPos); // Account for movement to get the duck
        delay(200);
        chassis.moveBackwardWithEncoders(0.6,630);
        intake.intakeMotor.setPower(0);
        lift.goTo(LEVEL_3,LIFT_SPEED);
        delay(700);
        hopper.hopper.setPosition(HOPPER_TOP);
        delay(1200);
        hopper.hopper.setPosition(HOPPER_BOTTOM);
        delay(200);
        lift.goTo(0,LIFT_SPEED);

        // Drive into the warehouse
        chassis.moveForwardWithEncoders(0.6,500);
        chassis.rotateToGlobalAngle(90,0.5);
        chassis.strafeRightWithEncoders(0.5,750);
        chassis.strafeLeftWithEncoders(0.5,10);
        intake.intakeMotor.setPower(0.8);
        chassis.moveForwardWithEncoders(0.7, 2300);

         */

        // Drive into the warehouse (NEW)
        chassis.strafeRightWithEncoders(0.6,50);
        chassis.moveForwardWithEncoders(0.6,200);
        chassis.rotateToGlobalAngle(90,0.5);
        chassis.moveForwardWithEncoders(0.6,4500);

    }

    public void delay(int time) {
        double startTime = runtime.milliseconds();
        while (runtime.milliseconds() - startTime < time) {
        }
    }
}
