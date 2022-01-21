package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Sticky implements Mechanism {
    CRServo crServo;
    Servo rotateServo;
    Servo heightServo;
    double rotatePosition = 0;
    double heightPosition = 0;


    @Override
    public void init(HardwareMap hardwareMap) {
        crServo = hardwareMap.get(CRServo.class, "crservo");
        rotateServo = hardwareMap.get(Servo.class, "rotateServo");
        heightServo = hardwareMap.get(Servo.class, "heightServo");
    }

    @Override
    public void run(Gamepad gamepad) {
        if (gamepad.left_trigger > .2 || gamepad.right_trigger > .2) {
            crServo.setPower(gamepad.left_trigger - gamepad.right_trigger);
        }
        if (Math.abs(gamepad.right_stick_x) > .2) {
            rotatePosition += gamepad.right_stick_x * .1;
            rotateServo.setPosition(rotatePosition);
        }
        if (Math.abs(gamepad.right_stick_y) > .2) {
            heightPosition += -gamepad.right_stick_y * .1;
            heightServo.setPosition(heightPosition);
        }
    }
}
