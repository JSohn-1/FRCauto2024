// Class that contains the color detection methods for the robot. 

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class colorDetection {
    private static ColorSensor color_REV_ColorRangeSensor;

    public static void initColorSensor(LinearOpMode opMode) {
        color_REV_ColorRangeSensor = opMode.hardwareMap.get(ColorSensor.class, "color_REV_ColorRangeSensor");
    }
}