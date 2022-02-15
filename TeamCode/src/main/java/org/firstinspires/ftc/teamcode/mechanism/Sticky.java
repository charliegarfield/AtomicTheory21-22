package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Sticky implements Mechanism {
    CRServo measureServo;
    Servo rotateServo;
    Servo heightServo;
    double rotatePosition = 0;
    double heightPosition = 0;


    @Override
    public void init(HardwareMap hardwareMap) {
        measureServo = hardwareMap.get(CRServo.class, "stickyMotor");
        rotateServo = hardwareMap.get(Servo.class, "rotateServo");
        heightServo = hardwareMap.get(Servo.class, "heightServo");
    }

    @Override
    public void run(Gamepad gamepad) {
        if (Math.abs(gamepad.left_stick_y) > .2) {
            measureServo.setPower(-gamepad.left_stick_y * 0.8);
        }
        if (Math.abs(gamepad.left_stick_x) > .2) {
            rotatePosition += gamepad.right_stick_x * .05;
            rotateServo.setPosition(rotatePosition);
        }
        if (Math.abs(gamepad.right_stick_y) > .2) {
            heightPosition += -gamepad.right_stick_y * .05;
            heightServo.setPosition(heightPosition);
        }
    }
}
