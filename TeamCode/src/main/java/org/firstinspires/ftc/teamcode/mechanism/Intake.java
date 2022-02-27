package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake implements Mechanism {
    public DcMotor intakeMotor;
    public boolean intakeOverrideActive = false;

    @Override
    public void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
    }

    @Override
    public void run(Gamepad gamepad) {
        if (gamepad.right_trigger > 0.5) {
            intakeMotor.setPower(gamepad.right_trigger);
            intakeOverrideActive = false;
        } else if (gamepad.left_trigger > 0.5) {
            intakeMotor.setPower(-gamepad.left_trigger);
            intakeOverrideActive = false;
        } else {
            if (!intakeOverrideActive) {
                intakeMotor.setPower(0);
            }
        }
    }

    public void overrideOut() {
        intakeMotor.setPower(-.3);
        intakeOverrideActive = true;
    }

    public void overrideIn() {
        intakeMotor.setPower(.3);
        intakeOverrideActive = true;
    }

    public void overrideStop() {
        intakeMotor.setPower(0);
        intakeOverrideActive = false;
    }

    public boolean isOverrideActive() {
        return intakeOverrideActive;
    }
}
