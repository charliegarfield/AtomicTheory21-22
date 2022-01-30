package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanism.*;
import org.firstinspires.ftc.teamcode.mechanism.chassis.MecanumChassis;

@TeleOp(name = "Mecanum OpMode (Blue)", group = "Remote")
public class MecanumBlue extends OpMode {
    MecanumChassis chassis = new MecanumChassis();
    Carousel carousel = new Carousel(Color.BLUE);
    Lift lift = new Lift();
    Intake intake = new Intake();
    Hopper hopper = new Hopper();
    Sticky sticky = new Sticky();
    boolean stickyMode = false;
    boolean yIsPressed = false;

    @Override
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
        carousel.run(gamepad1);

        if (stickyMode) {
            //TODO: Uncomment when sticky is ready
            //sticky.run(gamepad2);
        } else {

            lift.run(gamepad2);
            intake.run(gamepad2);
            hopper.run(gamepad2);
        }

        // Toggling Sticky Mode
        if (gamepad2.y && !yIsPressed) {
            yIsPressed = true;
            stickyMode = !stickyMode;
        } else if (!gamepad2.y && yIsPressed) {
            yIsPressed = false;
        }
    }
}
