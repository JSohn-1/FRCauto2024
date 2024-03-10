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

// public enum Colors {
//     RED, BLUE, ERROR
// }

public class ColorDetection {
    private static ColorSensor color_REV_ColorRangeSensor;
    private static Colors color;

    public static void initColorSensor(LinearOpMode opMode, Colors colors) {
        color_REV_ColorRangeSensor = opMode.hardwareMap.get(ColorSensor.class, "color");
        color = colors;
    }

    public static boolean check(){
        if (((DistanceSensor)color_REV_ColorRangeSensor).getDistance(DistanceUnit.CM) < 30) {
            return getColor() == color;
        }
        return false;
    }

    public static Colors getColor(){
        NormalizedRGBA colors = ((NormalizedColorSensor)color_REV_ColorRangeSensor).getNormalizedColors();
        int color = colors.toColor();
        // telemetry.addData("Color", color);

        return color == 0 ? Colors.RED : Colors.BLUE;
    }
}