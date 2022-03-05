package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.opmode.PoseUpdate;
import org.firstinspires.ftc.teamcode.mechanism.MB1242;
import org.firstinspires.ftc.teamcode.util.MathUtils;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class UltrasonicLocalizer implements Localizer {
    public static final double DIST_TOLERANCE = 2.0;
    public final double MAX_SENSOR_DISTANCE = 142;
    PoseUpdate poseUpdate = new PoseUpdate();
    Pose2d poseEstimate;
    Pose2d previousPose;
    SampleMecanumDrive drive;
    ElapsedTime pingTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private final Map<MB1242, Pose2d> sensorMap;

    public UltrasonicLocalizer(SampleMecanumDrive drive, Map<MB1242, Pose2d> map) {
        this.drive = drive;
        sensorMap = map;
    }


    double theta = 0;
    List<Double> lastWheelPositions = new ArrayList<>();


    public void init(HardwareMap hardwareMap) {
        pingTimer.reset();
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    // A guy on discord said this is about right
    int MILLIS_PER_CYCLE = 40;


    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        previousPose = pose2d;
        poseEstimate = pose2d;
        lastWheelPositions = new ArrayList<>();
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    @Override
    public void update() {
        double heading = drive.getExternalHeading();
        double accumX = 0, accumY = 0;
        int totalX = 0, totalY = 0;
        for (Map.Entry<MB1242, Pose2d> entry : sensorMap.entrySet()) {
            MB1242 sensor = entry.getKey();
            if (pingTimer.milliseconds() > MILLIS_PER_CYCLE) {
                pingTimer.reset();
                Pose2d sensorPose = entry.getValue();
                double distance = sensor.getDistance(DistanceUnit.INCH);
                if (distance < MAX_SENSOR_DISTANCE && distance > 8) {
                    sensorPose = new Pose2d(sensorPose.vec().rotated(heading), Angle.norm(sensorPose.getHeading() + heading));
                    double change;
                    switch (MathUtils.closestTo(2 * sensorPose.getHeading() / Math.PI, 0, 1, 2, 3, 4)) {
                        case 0: case 4:
                            change = 144 - sensorPose.getX() - (Math.cos(sensorPose.getHeading()) * distance);
                            if (Math.abs(poseEstimate.getX() - change) > 10) break;
                            accumX += change;
                            totalX++;
                            break;
                        case 1:
                            change = -144 - sensorPose.getY() + Math.sin(sensorPose.getHeading()) * distance;
                            if (Math.abs(poseEstimate.getY() - change) > 10) break;
                            accumY += change;
                            totalY++;
                            break;
                        case 2:
                            change = -144 - sensorPose.getX() + Math.cos(sensorPose.getHeading()) * distance;
                            if (Math.abs(poseEstimate.getX() + change) > 10) break;
                            accumX -= change;
                            totalX++;
                            break;
                        case 3:
                            change = 144 - sensorPose.getY() - Math.sin(sensorPose.getHeading()) * distance;
                            if (Math.abs(poseEstimate.getY() + change) > 10) break;
                            accumY -= change;
                            totalY++;
                            break;
                    }
                }
                sensor.ping();
            }
        }
        // This will also be true if the sensors weren't pinged this cycle
        if (totalX != 0 && totalY != 0) {
            poseEstimate = new Pose2d(accumX / totalX, accumY / totalY, heading);
            poseUpdate.set_poseEstimate(poseEstimate);
        } else {
            poseEstimate = calculatePoseEncoders();
        }
    }


    /**
     * @see com.acmerobotics.roadrunner.drive.MecanumDrive.MecanumLocalizer
     * Basically just that math added to this class because some of it is private
     */
    public Pose2d calculatePoseEncoders() {
        return poseUpdate.get_poseEstimate();
    }
}
