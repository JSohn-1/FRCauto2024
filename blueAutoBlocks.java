package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "placepixelbluesidebackstageauto (Blocks to Java)")
public class placepixelbluesidebackstageauto extends LinearOpMode {

  private Servo leftHand;
  private Servo rightHand;
  private DcMotor motor0;
  private DcMotor motor1;
  private DcMotor motor2;
  private DcMotor motor3;
  private DcMotor arm;

  ElapsedTime Timer;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    double currTime;

    leftHand = hardwareMap.get(Servo.class, "leftHand");
    rightHand = hardwareMap.get(Servo.class, "rightHand");
    motor0 = hardwareMap.get(DcMotor.class, "motor0");
    motor1 = hardwareMap.get(DcMotor.class, "motor1");
    motor2 = hardwareMap.get(DcMotor.class, "motor2");
    motor3 = hardwareMap.get(DcMotor.class, "motor3");
    arm = hardwareMap.get(DcMotor.class, "arm");

    // Put initialization blocks here.
    leftHand.setPosition(-1);
    rightHand.setPosition(1);
    motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Timer = new ElapsedTime();
    Timer.reset();
    currTime = Timer.time();
    waitForStart();
    if (opModeIsActive()) {
      wait(0.5);
      Right(3.16687, 0.4);
      wait(0.5);
      Backard(3.11, 0.3);
      wait(0.5);
      arms(2.7, 0.4);
      wait(0.5);
      leftHand.setPosition(1);
      rightHand.setPosition(-1);
      wait(0.5);
      arms(0.4, -0.7);
      wait(0.5);
      Forward(0.4, 0.3);
      wait(0.5);
      Right(2.85, 0.4);
      wait(0.5);
      Backard(2.16687, 0.3);
    }
  }

  /**
   * move arm
   */
  private void arms(double secs, double power) {
    Timer.reset();
    while (Timer.milliseconds() <= secs * 1000) {
      // Put loop blocks here.
      arm.setPower(power);
      telemetry.update();
    }
    arm.setPower(0);
    telemetry.update();
  }

  /**
   * Describe this function...
   */
  private void Backard(double secs, double power) {
    Timer.reset();
    while (Timer.milliseconds() <= secs * 1000) {
      // Put loop blocks here.
      motor2.setPower(-power);
      motor1.setPower(-power);
      motor0.setPower(power);
      motor3.setPower(power);
    }
    motor2.setPower(0);
    motor1.setPower(0);
    motor0.setPower(0);
    motor3.setPower(0);
    telemetry.update();
  }

  /**
   * Describe this function...
   */
  private void Forward(double secs, double power) {
    Timer.reset();
    while (Timer.milliseconds() <= secs * 1000) {
      // Put loop blocks here.
      motor2.setPower(power);
      motor1.setPower(power);
      motor0.setPower(-power);
      motor3.setPower(-power);
    }
    motor2.setPower(0);
    motor1.setPower(0);
    motor0.setPower(0);
    motor3.setPower(0);
    telemetry.update();
  }

  /**
   * Describe this function...
   */
  private void wait(double secs) {
    Timer.reset();
    while (Timer.milliseconds() <= secs * 1000) {
      // Put loop blocks here.
      motor2.setPower(0);
      motor1.setPower(0);
      motor0.setPower(0);
      motor3.setPower(0);
    }
    telemetry.update();
  }

  /**
   * Describe this function...
   */
  private void Right(double secs, double power) {
    Timer.reset();
    while (Timer.milliseconds() <= secs * 1000) {
      // Put loop blocks here.
      motor2.setPower(power);
      motor1.setPower(-power);
      motor0.setPower(-power);
      motor3.setPower(power);
    }
    motor2.setPower(0);
    motor1.setPower(0);
    motor0.setPower(0);
    motor3.setPower(0);
    telemetry.update();
  }

  /**
   * Describe this function...
   */
  private void Left(double secs, double power) {
    Timer.reset();
    while (Timer.milliseconds() <= secs * 1000) {
      // Put loop blocks here.
      motor0.setPower(-power);
      motor1.setPower(power);
      motor2.setPower(power);
      motor3.setPower(-power);
    }
    motor2.setPower(0);
    motor1.setPower(0);
    motor0.setPower(0);
    motor3.setPower(0);
    telemetry.update();
  }
}
