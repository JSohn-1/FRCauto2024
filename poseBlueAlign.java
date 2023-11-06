package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.Map;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import java.util.HashMap;
import java.lang.Math;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "poseBlueAlign (Blocks to Java)")
public class poseBlueAlign extends LinearOpMode {
  
  private DcMotor motor0;
  private DcMotor motor1;
  private DcMotor motor2;
  private DcMotor motor3;
  List<AprilTagDetection> myAprilTagDetections;
  boolean USE_WEBCAM;
  AprilTagProcessor myAprilTagProcessor;
  AprilTagDetection myAprilTagDetection;
  VisionPortal myVisionPortal;

  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    motor0 = hardwareMap.get(DcMotor.class, "motor0");
    motor1 = hardwareMap.get(DcMotor.class, "motor1");
    motor2 = hardwareMap.get(DcMotor.class, "motor2");
    motor3 = hardwareMap.get(DcMotor.class, "motor3");
    
    motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    // This 2023-2024 OpMode illustrates the basics of AprilTag recognition and pose estimation.
    USE_WEBCAM = false;
    // Initialize AprilTag before waitForStart.
    initAprilTag();
    // Wait for the match to begin.
    telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
    telemetry.addData(">", "Touch Play to start OpMode");
    telemetry.update();
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        // telemetryAprilTag();
        HashMap<Character, Double> pos = getRotToAprilTag();
        if(pos != null){
        telemetry.addLine(pos.get('p').toString());
        }
        // Push telemetry to the Driver Station.
        telemetry.update();
        if (gamepad1.dpad_down) {
          // Temporarily stop the streaming session. This can save CPU
          // resources, with the ability to resume quickly when needed.
          myVisionPortal.stopStreaming();
        } else if (gamepad1.dpad_up) {
          // Resume the streaming session if previously stopped.
          myVisionPortal.resumeStreaming();
        }
        turn();
        // Share the CPU.
        sleep(20);
      }
    }
  }

  /**
   * Initialize AprilTag Detection.
   */
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

  /**
   * Display info (using telemetry) for a recognized AprilTag.
   */
  private void telemetryAprilTag() {
    // Get a list of AprilTag detections.
    myAprilTagDetections = myAprilTagProcessor.getDetections();
    telemetry.addData("# AprilTags Detected", JavaUtil.listLength(myAprilTagDetections));
    // Iterate through list and call a function to display info for each recognized AprilTag.
    for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
      myAprilTagDetection = myAprilTagDetection_item;
      // Display info about the detection.
      telemetry.addLine("");
      if (myAprilTagDetection.metadata != null) {
        telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") " + myAprilTagDetection.metadata.name);
        telemetry.addLine("XYZ " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.x, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.y, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.z, 6, 1) + "  (inch)");
        telemetry.addLine("PRY " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.pitch, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.roll, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.yaw, 6, 1) + "  (deg)");
        telemetry.addLine("RBE " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.range, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.bearing, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.elevation, 6, 1) + "  (inch, deg, deg)");
      } else {
        telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") Unknown");
        telemetry.addLine("Center " + JavaUtil.formatNumber(myAprilTagDetection.center.x, 6, 0) + "" + JavaUtil.formatNumber(myAprilTagDetection.center.y, 6, 0) + " (pixels)");
      }
    }
    telemetry.addLine("");
    telemetry.addLine("key:");
    telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up) dist.");
    telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
    telemetry.addLine("RBE = Range, Bearing & Elevation");
  }

  /**
   * Describe this function...
   */
  // TODO: Enter the correct return type for function named getPosToAprilTag
  private HashMap<Character, Double> getPosToAprilTag() {
    // Return the relative position to the AprilTags on the Board as a whole
    myAprilTagDetections = myAprilTagProcessor.getDetections();
    HashMap<Character, Double> finalPos = new HashMap<Character, Double>();
    HashMap<Integer, HashMap<Character, Double>> aprilTagPos = new HashMap<Integer, HashMap<Character, Double>>();
    
    telemetry.addData("# AprilTags Detected", JavaUtil.listLength(myAprilTagDetections));
    // Iterate through list and call a function to display info for each recognized AprilTag.
    for (AprilTagDetection myAprilTagDetection_item2 : myAprilTagDetections) {
      myAprilTagDetection = myAprilTagDetection_item2;
      // Display info about the detection.
      telemetry.addLine("");
      if (myAprilTagDetection.metadata != null) {
        telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") " + myAprilTagDetection.metadata.name);
        telemetry.addLine("XYZ " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.x, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.y, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.z, 6, 1) + "  (inch)");
        telemetry.addLine("PRY " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.pitch, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.roll, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.yaw, 6, 1) + "  (deg)");
        telemetry.addLine("RBE " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.range, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.bearing, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.elevation, 6, 1) + "  (inch, deg, deg)");
        
        HashMap<Character, Double> pos = new HashMap<Character, Double>();
        pos.put('x', myAprilTagDetection.ftcPose.x);
        pos.put('y', myAprilTagDetection.ftcPose.y);
        pos.put('z', myAprilTagDetection.ftcPose.z);
        
        aprilTagPos.put(myAprilTagDetection.id, pos);
      } else {
        telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") Unknown");
        telemetry.addLine("Center " + JavaUtil.formatNumber(myAprilTagDetection.center.x, 6, 0) + "" + JavaUtil.formatNumber(myAprilTagDetection.center.y, 6, 0) + " (pixels)");
      }
    }
    if(aprilTagPos.containsKey(2)){
      return aprilTagPos.get(2);
    }
    if(aprilTagPos.containsKey(1)){
      return aprilTagPos.get(1);
    }
    if(aprilTagPos.containsKey(3)){
      return aprilTagPos.get(3);
    }
    return null; 
  }
  
  private HashMap<Character, Double>getRotToAprilTag(){
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
    if(aprilTagRot.containsKey(2)){
      return aprilTagRot.get(2);
    }
    if(aprilTagRot.containsKey(1)){
      return aprilTagRot.get(1);
    }
    if(aprilTagRot.containsKey(3)){
      return aprilTagRot.get(3);
    }
    return null; 
    
  }
  
  private void drive(double one, double two, double three, double four){
    motor0.setPower(one);
    motor1.setPower(two);
    motor2.setPower(three);
    motor3.setPower(four);
  }
  
  private void turn(){
    while(true){
      HashMap<Character, Double> tag = getRotToAprilTag();
      if(tag == null){
        telemetry.addLine("nul");
        drive(0,0,0,0);
        telemetry.update();
        continue;
      }
    double rot = tag.get('y');
    if(Math.abs(rot) < 3){
      break;
    }
      telemetry.addLine(Double.valueOf(rot).toString());
      if(rot > 0){
        drive(0.5, 0.5, 0.5, 0.5);
      }else{
        drive(-0.5, -0.5, -0.5, -0.5);
      }
      telemetry.update();    }
  }
}