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
        if (!intakeOverrideActive) {
            if (gamepad.left_trigger + gamepad.right_trigger > .5) {
                intakeMotor.setPower(gamepad.right_trigger - gamepad.left_trigger);
            } else {
                intakeMotor.setPower(0);
            }
        } else if (gamepad.right_trigger > 0.5 && gamepad.left_trigger > 0.5) {
            intakeOverrideActive = false;
        }
    }

    public void overrideOut() {
        intakeMotor.setPower(-.3);
        intakeOverrideActive = true;
    }

    public void overrideIn() {
        intakeMotor.setPower(.8);
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
