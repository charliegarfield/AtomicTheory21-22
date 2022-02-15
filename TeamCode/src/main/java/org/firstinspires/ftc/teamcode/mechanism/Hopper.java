package org.firstinspires.ftc.teamcode.mechanism;

import static org.firstinspires.ftc.teamcode.Constants.HOPPER_BOTTOM;
import static org.firstinspires.ftc.teamcode.Constants.HOPPER_TOP;
import static org.firstinspires.ftc.teamcode.Constants.HOPPER_BACK;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Hopper implements Mechanism {
    public Servo hopper;
    RevColorSensorV3 colorSensor;
    int state = 0;
    final float[] hsvValues = new float[3];

    @Override
    public void init(HardwareMap hardwareMap) {
        hopper = hardwareMap.get(Servo.class, "hopper");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "sensor_color");
    }

    @Override
    public void run(Gamepad gamepad) {
        if (gamepad.right_bumper) {
            //Make sure it doesn't set the position constantly
            if (state != 1) {
                hopper.setPosition(HOPPER_TOP);
                state = 1;
            }
        } else if (gamepad.left_bumper) {
            if (state != 2) {
                hopper.setPosition(HOPPER_BACK);
                state = 2;
            }
        }
        else {
            //Make sure it doesn't set the position constantly
            if (state != 0) {
                hopper.setPosition(HOPPER_BOTTOM);
                state = 0;
            }
        }
    }

    public HopperContents contents() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        double dist = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
        double hue = hsvValues[0];
        double value = hsvValues[2];

        if(dist < 2.5) {
            if(60 < hue && hue < 90 || hue > 100 && value > 0.1) {
                return HopperContents.BLOCK;
            }
            if (90 < hue && hue < 125) {
                return HopperContents.DUCK;
            }
            return HopperContents.BALL;
        }

        return HopperContents.EMPTY;
    }
}
