package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake implements Mechanism {
    public DcMotor intakeMotor;
    @Override
    public void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
    }

    @Override
    public void run(Gamepad gamepad, boolean stickyMode) {
        if(!stickyMode) {
            if (gamepad.right_trigger > 0.5) {
                intakeMotor.setPower(gamepad.right_trigger);
            } else if (gamepad.left_trigger > 0.5) {
                intakeMotor.setPower(-gamepad.left_trigger);
            } else {
                intakeMotor.setPower(0);
            }
        }
    }
}
