package org.firstinspires.ftc.teamcode.autos.roadrunner;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanism.Color;
@TeleOp
public class AvoidingAutoBlue extends RRAvoidingAutoBase{
    @Override
    Color getColor() {
        return Color.BLUE;
    }
}
