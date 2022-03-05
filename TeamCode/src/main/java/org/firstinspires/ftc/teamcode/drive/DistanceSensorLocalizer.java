package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.MB1242;
import org.firstinspires.ftc.teamcode.util.MathUtils;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class DistanceSensorLocalizer implements Localizer {


    private Pose2d poseEstimate;
    private final SampleMecanumDrive drive;

    private final double maxSensorDistance = 144;
    private final double minSensorDistance = 7.7;

    private final Map<MB1242, Pose2d> sensorMap;

    public DistanceSensorLocalizer(SampleMecanumDrive drive, Map<MB1242, Pose2d> map){
        this.drive = drive;
        sensorMap = map;
        poseEstimate = new Pose2d();
    }


    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        poseEstimate = pose2d;
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    @Override
    public void update() {
        update(null);
    }

    public void pingSensors(){
        for(MB1242 sensor : sensorMap.keySet()){
            sensor.ping();
        }
    }

    public void update(@Nullable Pose2d old){
        double heading = getHeading();
        double accumX = 0, accumY = 0;
        int totalX = 0, totalY = 0;
        for(Map.Entry<MB1242, Pose2d> entry : sensorMap.entrySet()){
            DistanceSensor sensor = entry.getKey();
            Pose2d sensorPose = entry.getValue();
            double distance = sensor.getDistance(DistanceUnit.INCH);
            if(distance  < maxSensorDistance && distance > minSensorDistance) {
                sensorPose = new Pose2d(sensorPose.vec().rotated(heading), Angle.norm(sensorPose.getHeading() + heading));
                double change;
                switch (MathUtils.closestTo(2*sensorPose.getHeading()/Math.PI,  0, 1, 2, 3, 4)){
                    case 0: case 4:
                        change=71-sensorPose.getX()-Math.cos(sensorPose.getHeading())*distance;
                        if(old != null && Math.abs(old.getX()-change) > 10) break;
                        accumX+=change;
                        totalX++;
                        break;
                    case 1:
                        change=71-sensorPose.getY()-Math.sin(sensorPose.getHeading())*distance;
                        if(old != null && Math.abs(old.getY()-change) > 10) break;
                        accumY+=change;
                        totalY++;
                        break;
                    case 2:
                        change=71+sensorPose.getX()+Math.cos(sensorPose.getHeading())*distance;
                        if(old != null && Math.abs(old.getX()+change) > 10) break;
                        accumX-=change;
                        totalX++;
                        break;
                    case 3:
                        change=71+sensorPose.getY()+Math.sin(sensorPose.getHeading())*distance;
                        if(old != null && Math.abs(old.getY()+change) > 10) break;
                        accumY-=change;
                        totalY++;
                        break;

                }
            }
        }
        if(old == null) old = new Pose2d();
        poseEstimate = new Pose2d(totalX != 0 ? accumX/totalX : old.getX(), totalY != 0 ? accumY/totalY : old.getY() , heading);
    }

    public double getHeading(){
        return Angle.norm(drive.getExternalHeading());
    }

}