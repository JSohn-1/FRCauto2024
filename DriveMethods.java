package org.firstins2pires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.HashMap;
import java.util.List;

enum Hand {
    LEFT,
    RIGHT
}

public class DriveMethods {
    private static Servo leftHand;
    private static Servo rightHand;
    private static DcMotor frontRight;
    private static DcMotor frontLeft;
    private static DcMotor backRight;
    private static DcMotor backLeft;
    private static DcMotor arm;

    final static double ticksPerInch = 56;
    final static double ticksPerInchSideways = 68;

    public static initDevices(LinearOpMode opMode) {
        leftHand = opMode.hardwareMap.get(Servo.class, "leftHand");
        rightHand = opMode.hardwareMap.get(Servo.class, "rightHand");
        frontRight = opMode.hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = opMode.hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = opMode.hardwareMap.get(DcMotor.class, "backRight");
        backLeft = opMode.hardwareMap.get(DcMotor.class, "backLeft");
        arm = opMode.hardwareMap.get(DcMotor.class, "arm");

        leftHand.setPosition(-1);
        rightHand.setPosition(1);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetMotors();

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private static void drive(double backRightPower, double backLeftPower, double frontRightPower, double frontLeftPower) {
        backRight.setPower(backRightPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        frontLeft.setPower(frontLeftPower);

        telemetry.update();
    }

    private void driveDistance(double distance){
        int ticks = (int) (distance * ticksPerInch);
        backLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(-ticks);
        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(-ticks);

        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        final double power = 0.3;
        drive(-power, power, -power, power);

        while (backLeft.isBusy() && backRight.isBusy() && frontLeft.isBusy() && frontRight.isBusy()) {
            telemetry.addData("backLeft", backLeft.getCurrentPosition());
            telemetry.addData("backRight", backRight.getCurrentPosition());
            telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
            telemetry.addData("frontRight", frontRight.getCurrentPosition());
            telemetry.update();
        }

        drive(0, 0, 0, 0);

        resetMotors();
    }

    static private void driveDistance(double distance, double power){
        int ticks = (int) (distance * ticksPerInch);
        backLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(-ticks);
        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(-ticks);

        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        drive(-power, power, -power, power);

        while (backLeft.isBusy() && backRight.isBusy() && frontLeft.isBusy() && frontRight.isBusy()) {
            telemetry.addData("backLeft", backLeft.getCurrentPosition());
            telemetry.addData("backRight", backRight.getCurrentPosition());
            telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
            telemetry.addData("frontRight", frontRight.getCurrentPosition());
            telemetry.update();
        }

        drive(0, 0, 0, 0);

        resetMotors();
    } 

    static private void strafe(double distance) {
        int ticks = (int) (distance * ticksPerInchSideways);
        backLeft.setTargetPosition(-ticks);
        backRight.setTargetPosition(-ticks);
        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(ticks);

        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        final double power = 0.3;
        drive(-power, -power, power, power);

        while (backLeft.isBusy() && backRight.isBusy() && frontLeft.isBusy() && frontRight.isBusy()) {
            telemetry.addData("backLeft", backLeft.getCurrentPosition());
            telemetry.addData("backRight", backRight.getCurrentPosition());
            telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
            telemetry.addData("frontRight", frontRight.getCurrentPosition());
            telemetry.update();
        }

        drive(0, 0, 0, 0);

        resetMotors();
    }
    
    static private void strafe(double distance, double power) {
        int ticks = (int) (distance * ticksPerInchSideways);
        backLeft.setTargetPosition(-ticks);
        backRight.setTargetPosition(-ticks);
        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(ticks);

        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        drive(-power, -power, power, power);

        while (backLeft.isBusy() && backRight.isBusy() && frontLeft.isBusy() && frontRight.isBusy()) {
            telemetry.addData("backLeft", backLeft.getCurrentPosition());
            telemetry.addData("backRight", backRight.getCurrentPosition());
            telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
            telemetry.addData("frontRight", frontRight.getCurrentPosition());
            telemetry.update();
        }

        drive(0, 0, 0, 0);

        resetMotors();
    }    

    static private void turn(double degrees) {
        final double ticksPerDegree = 15.5;
        int ticks = (int) (degrees * ticksPerDegree);
        backLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks);
        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(ticks);

        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        final double power = 0.2;
        drive(power, power, power, power);

        while (backLeft.isBusy() && backRight.isBusy() && frontLeft.isBusy() && frontRight.isBusy()) {
            telemetry.addData("backLeft", backLeft.getCurrentPosition());
            telemetry.addData("backRight", backRight.getCurrentPosition());
            telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
            telemetry.addData("frontRight", frontRight.getCurrentPosition());
            telemetry.update();
        }

        drive(0, 0, 0, 0);

        resetMotors();
    }

    private static void dropArm() {
        final double armTarget = 2750;
        arm.setTargetPosition((int) armTarget);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm.setPower(0.2);

        while (arm.isBusy()) {
            telemetry.addData("arm", arm.getCurrentPosition());
            telemetry.update();
        }

        arm.setPower(0);
        }
        
    private static void partialDropArm(){
        final double armTarget = 2100;
        arm.setTargetPosition((int) armTarget);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm.setPower(0.2);

        while (arm.isBusy()) {
            telemetry.addData("arm", arm.getCurrentPosition());
            telemetry.update();
        }

        arm.setPower(0);
    }
        
    private static void liftArm() {
        final double armTarget = 30;
        arm.setTargetPosition((int) armTarget);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm.setPower(-0.5);

        while (arm.isBusy()) {
            telemetry.addData("arm", arm.getCurrentPosition());
            telemetry.update();
        }

        arm.setPower(0);
    }

    private static void customArm(int seconds, double power){
        arm.setPower(power);
        delay(seconds);
        arm.setPower(0);
    }

    private static void openHand(Hand hand) {
        if (hand == Hand.LEFT) {
            leftHand.setPosition(1);
        } else {
            rightHand.setPosition(-1);
        }
    }

    private static void delay(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.milliseconds() < seconds * 1000) {
            telemetry.addData("Time", timer.seconds());
            telemetry.update();
        }
    }

    private static void resetMotors() {
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}