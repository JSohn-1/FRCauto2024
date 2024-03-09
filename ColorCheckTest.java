package org.firstins2pires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ColorDetection;

@Autonomous(name = "color test")
public class ColorCheckTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        ColorDetection.initColorSensor(this, Color.BLUE);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.addData("Color", getColor());
                telemetry.addData("Correct", ColorDetection.check());
                telemetry.update();
            }
        }
    }
}