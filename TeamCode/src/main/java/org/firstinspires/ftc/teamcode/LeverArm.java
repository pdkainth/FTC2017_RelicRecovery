package org.firstinspires.ftc.teamcode;

import java.lang.Math;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LeverArm {
  private DcMotor leverArmDrive = null;
  private int encoderPosition[] = {0, 600, 3600, 6600, 9300};
  private int targetPositioIndex;
  private int targetPosition;

  public void init(HardwareMap hardwaremap) {
    leverArmDrive = hardwaremap.get(DcMotor.class, "Motor5");
    leverArmDrive.setPower(0.0);
    leverArmDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leverArmDrive.setDirection(DcMotorSimple.Direction.FORWARD);
    leverArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    targetPosition = 0;
    targetPositioIndex = 0;
    leverArmDrive.setTargetPosition(targetPosition);
    leverArmDrive.setPower(0.5);
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
      leverArmDrive.setPower(0.5);
    }

    int encoderPosition = leverArmDrive.getCurrentPosition();

    if (encoderPosition == targetPosition) {
      leverArmDrive.setPower(0.0);
    }

    telemetry.addData("leverArm", "targetPositioIndex %d CurrentPos %d TargetPos %d",
      targetPositioIndex, encoderPosition, targetPosition);
  }

  public void stop() {
    leverArmDrive.setPower(0.0);
  }
}
