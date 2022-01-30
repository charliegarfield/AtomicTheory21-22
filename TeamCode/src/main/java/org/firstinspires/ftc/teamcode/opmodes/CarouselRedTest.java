package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.mechanism.Color;

@TeleOp
public class CarouselRedTest extends OpMode {
    Carousel carousel = new Carousel(Color.RED);

    @Override
    public void init() {
        carousel.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        carousel.run(gamepad1);
        telemetry.addData("Current Velocity", carousel.currentVelocity);
        telemetry.addData("Target Velocity", carousel.targetVelocity);
        telemetry.addData("Velocity Error", carousel.velocityError);
        telemetry.addData("Current Position", carousel.currentPosition);
        telemetry.addData("Target Position", carousel.targetPosition);
        telemetry.addData("Correction", carousel.correction);
    }
}
