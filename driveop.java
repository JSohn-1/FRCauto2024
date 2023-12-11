package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.HashMap;
import java.util.List;

@TeleOp(name = "DriveOPAuto")

public class DriveOPAuto extends LinearOpMode {
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor arm;
    private Servo leftHand;
    private Servo rightHand;
    private Servo droneLauncher;

    List<AprilTagDetection> myAprilTagDetections;
    boolean USE_WEBCAM;
    AprilTagProcessor myAprilTagProcessor;
    AprilTagDetection myAprilTagDetection;
    VisionPortal myVisionPortal;

    @Override
    public void runOpMode() {
        backLeft = hardwareMap.get(DcMotor.class, "motor0");
        backRight = hardwareMap.get(DcMotor.class, "motor1");
        frontRight = hardwareMap.get(DcMotor.class, "motor2");
        frontLeft = hardwareMap.get(DcMotor.class, "motor3");
        arm = hardwareMap.get(DcMotor.class, "arm");
        leftHand = hardwareMap.get(Servo.class, "leftHand");
        rightHand = hardwareMap.get(Servo.class, "rightHand");
        droneLauncher = hardwareMap.get(Servo.class, "droneLauncher");

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        USE_WEBCAM = false;
        initAprilTag();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            driveAdvanced();
            // rotate();
            armControl();
            setDroneLauncher();
            align();
            hold();
            telemetry.update();

        }
    }

    private void drive(double backRightPower, double backLeftPower, double frontRightPower, double frontLeftPower) {
        backRight.setPower(backRightPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        frontLeft.setPower(frontLeftPower);

        telemetry.update();
    }

    private void halt() {
        drive(0, 0, 0, 0);
    }

    private void driveAdvanced() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x; // Counteract imperfect strafing
        double x_two = gamepad1.right_stick_x;

        if(Math.abs(y)<0.01 && Math.abs(x) < 0.01 && Math.abs(x_two) < 0.01){
            return;
        }
        
        // Determine the angle to robot needs to travel in
        double angle = Math.atan2(y, x);
        double magnitude = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)) * 2;

        double frontRightPower = Math.sin(angle - (Math.PI / 4)) * magnitude;
        double frontLeftPower = Math.sin(angle + (Math.PI / 4)) * magnitude; 
        double backRightPower = Math.sin(angle + (Math.PI / 4)) * magnitude;
        double backLeftPower = Math.sin(angle - (Math.PI / 4)) * magnitude;
        
        frontRightPower += -x_two*2;
        frontLeftPower += x_two*2;
        backRightPower += -x_two*2;
        backLeftPower += x_two*2;
        
        boolean precision = gamepad1.left_bumper && gamepad1.right_bumper;

        if (precision) {
            frontLeftPower *= 0.1;
            backLeftPower *= 0.1;
            frontRightPower *= 0.1;
            backRightPower *= 0.1;
        }

        drive(-backRightPower, backLeftPower, -frontRightPower, frontLeftPower);
    }

    private void driveAdvanced(double x, double y, double magnitude) {
        double angle = Math.atan2(y, x);

        double frontRightPower = Math.sin(angle - (Math.PI / 4)) * magnitude;
        double frontLeftPower = Math.sin(angle + (Math.PI / 4)) * magnitude;
        double backRightPower = Math.sin(angle + (Math.PI / 4)) * magnitude;
        double backLeftPower = Math.sin(angle - (Math.PI / 4)) * magnitude;

        drive(-backRightPower, backLeftPower, -frontRightPower, frontLeftPower);
    }

    private void rotate() {
        double x = gamepad1.right_stick_x;
        x*=2;
        if(Math.abs(x)<0.1){
            return;
        }
        
        double frontRightPower = -x;
        double frontLeftPower = x;
        double backRightPower = -x;
        double backLeftPower = x;

        boolean precision = gamepad1.left_bumper && gamepad1.right_bumper;

        if (precision) {
            frontLeftPower *= 0.5;
            backLeftPower *= 0.5;
            frontRightPower *= 0.5;
            backRightPower *= 0.5;
        }

        drive(-backRightPower, backLeftPower, -frontRightPower, frontLeftPower);
    }

    private void hold(){
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x; // Counteract imperfect strafing
        double x_two = gamepad1.right_stick_x;
                if(Math.abs(y)<0.01 && Math.abs(x) < 0.01 && Math.abs(x_two) < 0.01){
            halt();
        }
    }

    private void armControl() {
        double y = gamepad2.left_stick_y * 0.4;

        arm.setPower(y);

        if (gamepad2.left_bumper) {
            leftHand.setPosition(1);
        } else if (toBoolean(gamepad2.left_trigger)) {
            leftHand.setPosition(-1);
        }

        if (gamepad2.right_bumper) {
            rightHand.setPosition(-1);
        } else if (toBoolean(gamepad2.right_trigger)) {
            rightHand.setPosition(1);
        }
    }

    private void setDroneLauncher() {
        boolean fire = gamepad1.dpad_up && gamepad2.dpad_up;

        if (fire) {
            droneLauncher.setPosition(1);
        }
    }

    private void initAprilTag() {
        AprilTagProcessor.Builder myAprilTagProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;

        // First, create an AprilTagProcessor.Builder.
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        // Create an AprilTagProcessor by calling build.
        myAprilTagProcessor = myAprilTagProcessorBuilder.build();
        // Next, create a VisionPortal.Builder and set attributes related to the camera.
        myVisionPortalBuilder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            // Use a webcam.
            myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            // Use the device's back camera.
            myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
        }
        // Add myAprilTagProcessor to the VisionPortal.Builder.
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
        // Create a VisionPortal by calling build.
        myVisionPortal = myVisionPortalBuilder.build();
    }

    private HashMap<Character, Double> getPosToAprilTag() {
        // Return the relative position to the AprilTags on the Board as a whole
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

    private boolean turn() {
        HashMap<Character, Double> tag = getRotToAprilTag();
        if (tag == null) {
            telemetry.addLine("none");
            halt();
            return false;
        }
        double rot = tag.get('y');
        double output = 0.4;
        if (Math.abs(rot) < 5) {
            halt();
            return true;
        }
        telemetry.addLine(Double.valueOf(rot).toString());
        if (rot > 0) {
            drive(output, output, output, output);
        } else {
            drive(-output, -output, -output, -output);
        }
        telemetry.update();
        return false;
    }

    private boolean moveToPos() {
        HashMap<Character, Double> tag = getPosToAprilTag();
        if (tag == null) {
            telemetry.addLine("none");
            halt();
            return false;
        }
        final Double DISTANCE = 7.0;

        if (Math.abs(tag.get('y')) < DISTANCE && Math.abs(tag.get('x')) < 1) {
            halt();
            return true;
        }

        driveAdvanced(tag.get('x'), tag.get('y') + DISTANCE, 0.5);
        return false;
    }

    private void align() {
        if (gamepad1.a) {
            while (gamepad1.a) {
                telemetry.addLine("aligning");
                telemetry.update();
                if (turn()) {
                    if (moveToPos())
                        telemetry.addLine("aligned");
                }
            }
            halt();
        }

        if (gamepad1.b) {
            while (gamepad1.b) {
                telemetry.addLine("aligning");
                telemetry.update();
                if (moveToPos()) {
                    if (turn())
                        telemetry.addLine("aligned");
                }
            }
            halt();
        }
    }

    private boolean toBoolean(double value) {
        return value >= 0.5;
    }
}
