package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanism.Capper;
import org.firstinspires.ftc.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.mechanism.Color;
import org.firstinspires.ftc.teamcode.mechanism.Hopper;
import org.firstinspires.ftc.teamcode.mechanism.Intake;
import org.firstinspires.ftc.teamcode.mechanism.Lift;
import org.firstinspires.ftc.teamcode.mechanism.chassis.MecanumChassis;

@TeleOp(name = "Mecanum OpMode (Blue)", group = "Remote")
public class MecanumBlue extends OpMode {
    MecanumChassis chassis = new MecanumChassis();
    Carousel carousel = new Carousel(Color.BLUE);
    Lift lift = new Lift();
    Intake intake = new Intake();
    Hopper hopper = new Hopper();
    Capper capper = new Capper();

    @Override
    public void init() {
        // Initialize each mechanism
        chassis.init(hardwareMap);
        carousel.init(hardwareMap);
        lift.init(hardwareMap);
        intake.init(hardwareMap);
        hopper.init(hardwareMap);
        capper.init(hardwareMap);
    }

    @Override
    public void loop() {
        // Run each mechanism
        chassis.run(gamepad1);
        carousel.run(gamepad1);
        lift.run(gamepad2);
        intake.run(gamepad2);
        hopper.run(gamepad2);
        capper.run(gamepad2);
    }
}
