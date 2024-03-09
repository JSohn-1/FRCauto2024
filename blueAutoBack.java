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

// enum Hand {
//     LEFT,
//     RIGHT
// }

@Autonomous(name = "blueAutoBack")
public class BlueAutoBack extends LinearOpMode {
    private Servo leftHand;
    private Servo rightHand;
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor arm;

    final double ticksPerInch = 56;
    final double ticksPerInchSideways = 68;

    List<AprilTagDetection> myAprilTagDetections;
    AprilTagProcessor myAprilTagProcessor;
    AprilTagDetection myAprilTagDetection;
    VisionPortal myVisionPortal;

    @Override
    public void runOpMode() {
        double currTime;

        backLeft = hardwareMap.get(DcMotor.class, "motor0");
        backRight = hardwareMap.get(DcMotor.class, "motor1");
        frontRight = hardwareMap.get(DcMotor.class, "motor2");
        frontLeft = hardwareMap.get(DcMotor.class, "motor3");
        arm = hardwareMap.get(DcMotor.class, "arm");
        leftHand = hardwareMap.get(Servo.class, "leftHand");
        rightHand = hardwareMap.get(Servo.class, "rightHand");

        // Put initialization blocks here.
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
        
        initAprilTag();

        // colorDetection.initColorSensor(this);

        waitForStart();
        if (opModeIsActive()) {
            leftObject();
            // Move to line
            // strafe(-2.5);
            // driveDistance(21);
            // dropArm();
            // openHand(Hand.RIGHT);
            // delay(0.3);
            // liftArm();
            // // driveDistance(4);
            // turn(-90);
            // driveDistance(30);
            // strafe(10);
            // // driveUntilAprilTag();
            // // alignToAprilTag();
            // partialDropArm();
            // delay(0.5);
            // openHand(Hand.LEFT);
            // delay(0.5);
            // liftArm();
            // strafe(-30, 1);
            // driveDistance(15, 1);
        }
    }

    private void drive(double backRightPower, double backLeftPower, double frontRightPower, double frontLeftPower) {
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
    
    private void driveDistance(double distance, double power){
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

    private void strafe(double distance) {
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
    
    private void strafe(double distance, double power) {
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

    private void turn(double degrees) {
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

    private void driveUntilAprilTag() {
        HashMap<Character, Double> pos = getPosToAprilTag();
        final double power = 0.1;

        while (pos == null) {
            drive(power, -power, power, -power);
            pos = getPosToAprilTag();
        }
    }
    
    private void driveUntilAprilTagBackwards() {
        HashMap<Character, Double> pos = getPosToAprilTag();
        final double power = -0.1;

        while (pos == null) {
            drive(power, -power, power, -power);
            pos = getPosToAprilTag();
        }
    }

    private void alignToAprilTag() {
        HashMap<Character, Double> pos = getPosToAprilTag();
        final double power = 0.1;
        final double distanceFromTag = 5;

        while (Math.abs(pos.get('x')) > 3 || Math.abs(pos.get('y')) > distanceFromTag) {
            
            strafe(pos.get('x')); 
            driveDistance(pos.get('y') - distanceFromTag);
            pos = getPosToAprilTag();
            
            if (pos == null){
                driveUntilAprilTagBackwards();
                pos = getPosToAprilTag();
            }
        }
    }

    private void dropArm() {
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
        
    private void partialDropArm(){
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
        
    private void liftArm() {
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

    private void openHand(Hand hand) {
        if (hand == Hand.LEFT) {
            leftHand.setPosition(1);
        } else {
            rightHand.setPosition(-1);
        }
    }

    private void delay(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.milliseconds() < seconds * 1000) {
            telemetry.addData("Time", timer.seconds());
            telemetry.update();
        }
    }

    private void resetMotors() {
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // April tag stuff
    private void initAprilTag() {
        AprilTagProcessor.Builder myAprilTagProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;

        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        myAprilTagProcessor = myAprilTagProcessorBuilder.build();
        myVisionPortalBuilder = new VisionPortal.Builder();
        myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
        myVisionPortal = myVisionPortalBuilder.build();
    }

    private HashMap<Character, Double> getPosToAprilTag() {
        myAprilTagDetections = myAprilTagProcessor.getDetections();
        HashMap<Character, Double> finalPos = new HashMap<Character, Double>();
        HashMap<Integer, HashMap<Character, Double>> aprilTagPos = new HashMap<Integer, HashMap<Character, Double>>();

        telemetry.addData("# AprilTags Detected", JavaUtil.listLength(myAprilTagDetections));
        // Iterate through list and call a function to display info for each recognized
        // AprilTag.
        for (AprilTagDetection myAprilTagDetection_item2 : myAprilTagDetections) {
            myAprilTagDetection = myAprilTagDetection_item2;
            // Display info about the detection.
            telemetry.addLine("");
            if (myAprilTagDetection.metadata != null) {
                telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") " + myAprilTagDetection.metadata.name);
                telemetry.addLine("XYZ " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.x, 6, 1) + " "
                        + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.y, 6, 1) + " "
                        + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.z, 6, 1) + "  (inch)");
                telemetry.addLine("PRY " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.pitch, 6, 1) + " "
                        + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.roll, 6, 1) + " "
                        + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.yaw, 6, 1) + "  (deg)");
                telemetry.addLine("RBE " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.range, 6, 1) + " "
                        + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.bearing, 6, 1) + " "
                        + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.elevation, 6, 1) + "  (inch, deg, deg)");

                HashMap<Character, Double> pos = new HashMap<Character, Double>();
                pos.put('x', myAprilTagDetection.ftcPose.x);
                pos.put('y', myAprilTagDetection.ftcPose.y);
                pos.put('z', myAprilTagDetection.ftcPose.z);

                aprilTagPos.put(myAprilTagDetection.id, pos);
            } else {
                telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") Unknown");
                telemetry.addLine("Center " + JavaUtil.formatNumber(myAprilTagDetection.center.x, 6, 0) + ""
                        + JavaUtil.formatNumber(myAprilTagDetection.center.y, 6, 0) + " (pixels)");
            }
        }

        // Red side
        if (aprilTagPos.containsKey(5)) {
            return aprilTagPos.get(5);
        }
        if (aprilTagPos.containsKey(4)) {
            return aprilTagPos.get(4);
        }
        if (aprilTagPos.containsKey(6)) {
            return aprilTagPos.get(6);
        }

        // Blue side
        if (aprilTagPos.containsKey(2)) {
            return aprilTagPos.get(2);
        }
        if (aprilTagPos.containsKey(1)) {
            return aprilTagPos.get(1);
        }
        if (aprilTagPos.containsKey(3)) {
            return aprilTagPos.get(3);
        }
        return null;
    }
    private HashMap<Character, Double> getRotToAprilTag() {
        myAprilTagDetections = myAprilTagProcessor.getDetections();
        HashMap<Integer, HashMap<Character, Double>> aprilTagRot = new HashMap<Integer, HashMap<Character, Double>>();

        for (AprilTagDetection myAprilTagDetection_item2 : myAprilTagDetections) {
            myAprilTagDetection = myAprilTagDetection_item2;

            if (myAprilTagDetection.metadata != null) {
                HashMap<Character, Double> rot = new HashMap<Character, Double>();
                rot.put('p', myAprilTagDetection.ftcPose.pitch);
                rot.put('r', myAprilTagDetection.ftcPose.roll);
                rot.put('y', myAprilTagDetection.ftcPose.yaw);

                aprilTagRot.put(myAprilTagDetection.id, rot);
            }
        }

        // Red side
        if (aprilTagRot.containsKey(5)) {
            return aprilTagRot.get(5);
        }
        if (aprilTagRot.containsKey(4)) {
            return aprilTagRot.get(4);
        }
        if (aprilTagRot.containsKey(6)) {
            return aprilTagRot.get(6);
        }

        // Blue side
        if (aprilTagRot.containsKey(2)) {
            return aprilTagRot.get(2);
        }
        if (aprilTagRot.containsKey(1)) {
            return aprilTagRot.get(1);
        }
        if (aprilTagRot.containsKey(3)) {
            return aprilTagRot.get(3);
        }
        return null;

    }

    private void leftObject() {
        strafe(-7);
        delay(0.5);
        driveDistance(13);
        delay(0.5);
        //jacob check Iterate
        driveDistance(9);
        delay(0.5);
        strafe(10);
        delay(0.5);
        // JAOCB DO SOMETHING
        driveDistance(-1);
        delay(0.5);
        turn(90);
        delay(0.5);
        driveDistance(-2);
       // Place, boringly
    }
}
