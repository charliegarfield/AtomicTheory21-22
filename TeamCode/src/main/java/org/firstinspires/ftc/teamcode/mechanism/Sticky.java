package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Sticky implements Mechanism {
    CRServo measureServo;
    CRServo rotateServo;
    CRServo heightServo;
    public double rotatePosition = 0;
    public double heightPosition = 0;


    @Override
    public void init(HardwareMap hardwareMap) {
        measureServo = hardwareMap.get(CRServo.class, "measureServo");
        rotateServo = hardwareMap.get(CRServo.class, "rotateServo");
        heightServo = hardwareMap.get(CRServo.class, "heightServo");
        measureServo.setDirection(DcMotorSimple.Direction.REVERSE);
        rotateServo.setDirection(DcMotorSimple.Direction.REVERSE);
        heightServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void run(Gamepad gamepad) {
        if (Math.abs(gamepad.left_stick_y) > .2) {
            measureServo.setPower(-gamepad.left_stick_y * 0.8);
        } else {
            measureServo.setPower(0);
        }

        if (Math.abs(gamepad.right_stick_x) > .2) {
            rotateServo.setPower(gamepad.right_stick_x * 0.3);
        } else {
            rotateServo.setPower(0);
        }

        if (Math.abs(gamepad.right_stick_y) > .2) {
            heightServo.setPower(-gamepad.right_stick_y * 0.6);
        } else {
            heightServo.setPower(0);
        }
    }
}
