package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechanism.MB1242;
@TeleOp
public class UltrasonicTest extends OpMode {
    MB1242 sensor;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    @Override
    public void init() {
        sensor = hardwareMap.get(MB1242.class, "distance1");
        sensor.ping();
    }

    @Override
    public void loop() {
        if (timer.milliseconds() > 100) {
            telemetry.addData("Distance", sensor.getDistance(DistanceUnit.INCH));
            sensor.ping();
            timer.reset();
        }
    }
}
