package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class ServoTest extends OpMode {
    public abstract String getServoName();
    Servo servo;
    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, getServoName());
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        servo.setPosition(gamepad1.left_stick_y);
    }
}
