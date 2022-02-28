package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.opencv.DuckFinder;
import org.firstinspires.ftc.teamcode.opencv.ShippingElementRecognizer;
import org.firstinspires.ftc.teamcode.opencv.kotlin.FreightFinder;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

public class Webcam {
    OpenCvWebcam webcam;
    OpenCvWebcam frontWebcam;
    OpenCvPipeline activePipeline;
    ShippingElementRecognizer shippingElementRecognizer;
    DuckFinder duckFinder;
    FreightFinder freightFinder;

    public int getShippingHubLevel() {
        return shippingElementRecognizer.getShippingHubLevel();
    }

    public void switchToDuckPipeline(){
        frontWebcam.setPipeline(duckFinder);
        activePipeline = duckFinder;
    }

    public void switchToFreightPipeline(){
        frontWebcam.setPipeline(freightFinder);
        activePipeline = freightFinder;
    }
    public Point getDuckCenter() {
        return duckFinder.getDuckCenter();
    }

    public double getDuckAngle() {
        return -duckFinder.calculateYaw(Constants.CAMERA_POSITION);
    }

    public Double calculateYaw(double cameraPosition) {
        if (activePipeline == duckFinder) {
            return duckFinder.calculateYaw(cameraPosition);
        } else {
            return freightFinder.calculateYaw(cameraPosition);
        }
    }

    public void init(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        2, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY); //Whether to split the container vertically or horizontally
        // Setup first camera
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), viewportContainerIds[0]);
        shippingElementRecognizer = new ShippingElementRecognizer();
        webcam.setPipeline(shippingElementRecognizer);
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
        duckFinder = new DuckFinder(78);
        freightFinder = new FreightFinder(78, 0,0);
        frontWebcam.setPipeline(duckFinder);
        activePipeline = duckFinder;
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
    }
}
