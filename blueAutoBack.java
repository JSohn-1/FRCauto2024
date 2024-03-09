package org.firstins2pires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.List;

import org.firstinspires.ftc.teamcode.DriveMethods;
import org.firstinspires.ftc.teamcode.ColorDetection;

@Autonomous(name = "blueAutoBack")
public class BlueAutoBack extends LinearOpMode, DriveMethods {
    @Override
    public void runOpMode() {
        initDevices(this);
        initColorSensor(this, Color.BLUE);

        initAprilTag();

        waitForStart();
        if (opModeIsActive()) {
            leftObject();
        }
    }

    private void leftObject() {
        driveDistance(20)
    }
}
