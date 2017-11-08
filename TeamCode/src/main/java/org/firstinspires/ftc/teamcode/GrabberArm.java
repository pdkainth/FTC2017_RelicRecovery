package org.firstinspires.ftc.teamcode;

import java.lang.Math;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class GrabberArm {

  private Servo leftArm = null;
  private Servo rightArm = null;

  public void init(HardwareMap hardwaremap) {
    leftArm  = hardwaremap.get(Servo.class, "Servo1");
    rightArm = hardwaremap.get(Servo.class, "Servo2");

    leftArm.setDirection(Servo.Direction.FORWARD);
    rightArm.setDirection(Servo.Direction.REVERSE);

    leftArm.setPosition(1.0);
    rightArm.setPosition(1.0);

  }

  public void update(double positionLeft, double positionRight, Telemetry telemetry) {
    // convery from -1.0 to 1.0 -> 0 to 1.0 range
    double servoPositionLeft  = (positionLeft  + 1.0) / 2;
    double servoPositionRight = (positionRight + 1.0) / 2;

    leftArm.setPosition(servoPositionLeft);
    rightArm.setPosition(servoPositionRight);

    telemetry.addData("grabberArm", "powerLeft %.2f powerRight %.2f posLeft %.2f posRight %.2f",
      positionLeft, positionRight, servoPositionLeft, servoPositionRight);
  }

  public void open(Telemetry telemetry){
    leftArm.setPosition(1.0);
    rightArm.setPosition(1.0);

    telemetry.addData("grabberArm", "open posLeft %.2f posRight %.2f", 1.0, 1.0);
  }

  public void close(Telemetry telemetry){
    leftArm.setPosition(0.35);
    rightArm.setPosition(0.35);

    telemetry.addData("grabberArm", "close posLeft %.2f posRight %.2f", 0.39, 0.39);
  }


  public void stop() {
    leftArm.setPosition(0.5);
    rightArm.setPosition(0.5);
  }
}
