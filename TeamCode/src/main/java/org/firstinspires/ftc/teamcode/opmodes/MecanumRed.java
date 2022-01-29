package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanism.*;
import org.firstinspires.ftc.teamcode.mechanism.chassis.MecanumChassis;

@TeleOp(name = "Mecanum OpMode (Red)", group = "Remote")
public class MecanumRed extends OpMode {
    MecanumChassis chassis = new MecanumChassis();
    Carousel carousel = new Carousel(Color.RED);
    Lift lift = new Lift();
    Intake intake = new Intake();
    Hopper hopper = new Hopper();
    Capper capper = new Capper();
    Sticky sticky = new Sticky();
    boolean stickyMode = false;
    boolean yIsPressed = true;

    public void init() {
        // Initialize each mechanism
        chassis.init(hardwareMap);
        carousel.init(hardwareMap);
        lift.init(hardwareMap);
        intake.init(hardwareMap);
        hopper.init(hardwareMap);
        //capper.init(hardwareMap);
        //sticky.init(hardwareMap);
    }

    @Override
    public void loop() {
        // Run each mechanism
        chassis.run(gamepad1);
        carousel.run(gamepad1, stickyMode);
        lift.run(gamepad2, stickyMode);
        intake.run(gamepad2, stickyMode);
        hopper.run(gamepad2, stickyMode);
        //capper.run(gamepad2, stickyMode);
        //sticky.run(gamepad2, stickyMode);

        // Toggling Sticky Mode
        if(gamepad2.y && !yIsPressed) {
            yIsPressed = true;
            stickyMode = !stickyMode;
        } else if (!gamepad2.y && yIsPressed) {
            yIsPressed = false;
        }

        telemetry.addData("lift level", lift.liftMotor.getCurrentPosition());
        telemetry.addData("front left pos", chassis.frontLeft.getCurrentPosition());
        telemetry.addData("front right pos", chassis.frontRight.getCurrentPosition());
        telemetry.addData("back left pos", chassis.backLeft.getCurrentPosition());
        telemetry.addData("back right pos", chassis.backRight.getCurrentPosition());
        telemetry.addData("sticky mode", stickyMode);
    }
}
