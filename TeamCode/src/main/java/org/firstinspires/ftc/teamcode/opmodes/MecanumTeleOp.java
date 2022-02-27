package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.mechanism.Color;
import org.firstinspires.ftc.teamcode.mechanism.Hopper;
import org.firstinspires.ftc.teamcode.mechanism.HopperContents;
import org.firstinspires.ftc.teamcode.mechanism.Intake;
import org.firstinspires.ftc.teamcode.mechanism.Lift;
import org.firstinspires.ftc.teamcode.mechanism.Sticky;
import org.firstinspires.ftc.teamcode.mechanism.chassis.MecanumChassis;

abstract public class MecanumTeleOp extends OpMode {
    ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    double gainedCargoTime;
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
    boolean hadCargo = false;

    @Override
    public void init() {
        // Initialize each mechanism
        chassis.init(hardwareMap);
        carousel.init(hardwareMap);
        lift.init(hardwareMap);
        intake.init(hardwareMap);
        hopper.init(hardwareMap);
        sticky.init(hardwareMap);
    }

    @Override
    public void start(){
        // Reset time when start button pressed
        elapsedTime.reset();
    }

    @Override
    public void loop() {
        // Run each mechanism

        chassis.run(gamepad1);
        carousel.run(gamepad1);

        if (stickyMode) {
            sticky.run(gamepad2);
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

        // Notify drivers when cargo is in the hopper
        if (!hadCargo && hopper.contents() != HopperContents.EMPTY) {
            if (hopper.contents() == HopperContents.BALL) {
                gamepad1.rumbleBlips(1);
                gamepad2.rumbleBlips(1);
            } else {
                gamepad1.rumbleBlips(2);
                gamepad2.rumbleBlips(2);
            }
            hadCargo = true;
            // Start auto outtake and disable the auto stop
            intake.overrideOut();
            gainedCargoTime = elapsedTime.seconds();
        } else if (hadCargo && hopper.contents() == HopperContents.EMPTY) {
            hadCargo = false;
        }

        // When it's been 3 seconds since we intook, stop the auto outtake
        if (elapsedTime.time() - gainedCargoTime > 3) {
            intake.overrideStop();
        }


        // Check if it is endgame yet

        if (elapsedTime.seconds() >= 85 && !driversNotifiedEndgame) { // 85 = 5 seconds before
            gamepad1.rumbleBlips(4);
            gamepad2.rumbleBlips(4);
            driversNotifiedEndgame = true;
        }

        telemetry.addData("lift level", lift.liftMotor.getCurrentPosition());
        telemetry.addData("front left pos", chassis.frontLeft.getCurrentPosition());
        telemetry.addData("front right pos", chassis.frontRight.getCurrentPosition());
        telemetry.addData("back left pos", chassis.backLeft.getCurrentPosition());
        telemetry.addData("back right pos", chassis.backRight.getCurrentPosition());
        telemetry.addData("sticky mode", stickyMode);
        telemetry.addData("rotate servo pos", sticky.rotatePosition);
        telemetry.addData("height servo pos", sticky.heightPosition);
        telemetry.addData("hopper contents", hopper.contents());
    }
}
