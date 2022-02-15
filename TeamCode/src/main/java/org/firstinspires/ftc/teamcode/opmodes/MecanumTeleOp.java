package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.mechanism.Color;
import org.firstinspires.ftc.teamcode.mechanism.Hopper;
import org.firstinspires.ftc.teamcode.mechanism.Intake;
import org.firstinspires.ftc.teamcode.mechanism.Lift;
import org.firstinspires.ftc.teamcode.mechanism.Sticky;
import org.firstinspires.ftc.teamcode.mechanism.chassis.MecanumChassis;

abstract public class MecanumTeleOp extends OpMode {
    ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    abstract Color getColor();
    MecanumChassis chassis = new MecanumChassis();
    Carousel carousel = new Carousel(getColor());
    Lift lift = new Lift();
    Intake intake = new Intake();
    Hopper hopper = new Hopper();
    Sticky sticky = new Sticky();
    boolean stickyMode = false;
    boolean yIsPressed = false;
    boolean driversNotifiedEndgame = false;

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
    public void start(){
        //Reset time when start button pressed
        elapsedTime.reset();
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


        //Check if it is endgame yet

        if (elapsedTime.seconds() >= 85 && !driversNotifiedEndgame) { // 85 = 5 seconds before
            gamepad1.rumbleBlips(3);
            gamepad2.rumbleBlips(3);
            driversNotifiedEndgame = true;
        }

        telemetry.addData("lift level", lift.liftMotor.getCurrentPosition());
        telemetry.addData("front left pos", chassis.frontLeft.getCurrentPosition());
        telemetry.addData("front right pos", chassis.frontRight.getCurrentPosition());
        telemetry.addData("back left pos", chassis.backLeft.getCurrentPosition());
        telemetry.addData("back right pos", chassis.backRight.getCurrentPosition());
        telemetry.addData("sticky mode", stickyMode);
    }
}
