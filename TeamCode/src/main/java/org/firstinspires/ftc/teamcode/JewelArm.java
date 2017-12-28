package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class JewelArm {

  private Servo jewelServo = null;
  private final double UP_SERVO = 0.90;
  private final double DOWN_SERVO = 0.15;

  public void init(HardwareMap hardwaremap) {
    jewelServo = hardwaremap.get(Servo.class, "Servo3");
    jewelServo.setDirection(Servo.Direction.FORWARD);

    up();
  }

  public void update(double position, Telemetry telemetry) {
    // convery from -1.0 to 1.0 -> 0 to 1.0 range
    double servoPosition = (position + 1.0) / 2;

    jewelServo.setPosition(servoPosition);

    telemetry.addData("jewelArm", "power %.2f  posLeft %.2f",
      position, servoPosition);
  }

  public void up() {
    jewelServo.setPosition(UP_SERVO);
  }

  public void down() {
    jewelServo.setPosition(DOWN_SERVO);
  }

  public void stop() {
    up();
  }
}
