package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by pdkainth on 11/22/2017.
 */

public class GrabberArm {

 private MyTeleOp.ButtonState currentButtonState = MyTeleOp.ButtonState.RELEASED;
  private enum GrabberState {
    OPEN, CLOSED
  }

  private GrabberState curGrabberState = GrabberState.OPEN;

  private Servo leftServo = null;
  private Servo rightServo = null;
  private final double OPEN_SERVO = 1.0;
  private final double CLOSE_SERVO = 0.35;

  public void init(HardwareMap hardwaremap) {
    leftServo  = hardwaremap.get(Servo.class, "Servo1");
    rightServo = hardwaremap.get(Servo.class, "Servo2");

    leftServo.setDirection(Servo.Direction.FORWARD);
    rightServo.setDirection(Servo.Direction.REVERSE);

    open();

  }
  public void update(double positionLeft, double positionRight, Telemetry telemetry) {
    // convery from -1.0 to 1.0 -> 0 to 1.0 range
    double servoPositionLeft  = (positionLeft  + 1.0) / 2;
    double servoPositionRight = (positionRight + 1.0) / 2;

    leftServo.setPosition(servoPositionLeft);
    rightServo.setPosition(servoPositionRight);

    telemetry.addData("grabberArm", "powerLeft %.2f powerRight %.2f posLeft %.2f posRight %.2f",
      positionLeft, positionRight, servoPositionLeft, servoPositionRight);
  }

  public void update(MyTeleOp.ButtonState newState, Telemetry telemetry){

     if (newState == MyTeleOp.ButtonState.PRESSED && currentButtonState == MyTeleOp.ButtonState.RELEASED){
       if (curGrabberState == GrabberState.OPEN){
         close();
         curGrabberState = GrabberState.CLOSED;
       } else {
         open();
         curGrabberState = GrabberState.OPEN;
       }
     }

    currentButtonState = newState;
     telemetry.addData("GrabberArm", "State %s button %s ", curGrabberState, currentButtonState);
  }

  public void open(){
    leftServo.setPosition(OPEN_SERVO);
    rightServo.setPosition(OPEN_SERVO);
  }

  public void close(){
    leftServo.setPosition(CLOSE_SERVO);
    rightServo.setPosition(CLOSE_SERVO);
  }

  public void stop(){
    open();
  }
}
