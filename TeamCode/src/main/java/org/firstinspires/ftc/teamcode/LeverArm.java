package org.firstinspires.ftc.teamcode;

import java.lang.Math;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LeverArm {

  private final static double LEVER_MOTOR_POWER = 0.75;
  private final static int encoderPosition[] = {0, 600, 3600, 6600, 9300};

  private DcMotor leverArmDrive = null;

  private int targetPositioIndex;
  private int targetPosition;

  public void init(HardwareMap hardwaremap) {
    leverArmDrive = hardwaremap.get(DcMotor.class, "Motor5");
    leverArmDrive.setPower(0.0);
    leverArmDrive.setDirection(DcMotorSimple.Direction.FORWARD);

    resetEncoder();
  }

  public void resetEncoder() {
    leverArmDrive.setPower(0.0);
    leverArmDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    targetPosition = 0;
    targetPositioIndex = 0;
    leverArmDrive.setTargetPosition(targetPosition);
  }

  public void raise(double raise, Telemetry telemetry) {
    leverArmDrive.setPower(raise);
    int encoderPosition = leverArmDrive.getCurrentPosition();
    telemetry.addData("leverArm", "power %.2f position %d", raise, encoderPosition);
  }

  public void setPosition(int position, Telemetry telemetry){
    if ((position >= 0) && (position <= 4)){
      targetPositioIndex = position;
      targetPosition = encoderPosition[position];
      leverArmDrive.setTargetPosition(targetPosition);
      leverArmDrive.setPower(LEVER_MOTOR_POWER);
    }

    int encoderPosition = leverArmDrive.getCurrentPosition();
    boolean motorBusy = leverArmDrive.isBusy();

    if ((Math.abs(encoderPosition - targetPosition) < 10) && (motorBusy == false)) {
      leverArmDrive.setPower(0.0);
    }

    telemetry.addData("leverArm", "motorBusy %b targetPositioIndex %d CurrentPos %d TargetPos %d",
      motorBusy, targetPositioIndex, encoderPosition, targetPosition);
  }

  public void stop() {
    leverArmDrive.setPower(0.0);
  }
}
