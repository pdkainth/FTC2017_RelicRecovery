package org.firstinspires.ftc.teamcode;

import java.lang.Math;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LeverArm {

  public enum MotorOpMode {
    IDLE, MANUAL, ENCODER
  }

  private final static double LEVER_MOTOR_POWER = 0.75;
  private final static int encoderPosition[] = {0, 600, 3600, 6600, 9300};

  private DcMotor leverArmDrive = null;

  private int targetPositioIndex;
  private int targetPosition;
  private MotorOpMode opMode = MotorOpMode.IDLE;

  public void init(HardwareMap hardwaremap) {
    leverArmDrive = hardwaremap.get(DcMotor.class, "Motor5");
    leverArmDrive.setPower(0.0);
    leverArmDrive.setDirection(DcMotorSimple.Direction.FORWARD);

    resetEncoder();
  }

  public MotorOpMode getOpState() {
    return opMode;
  }

  public void resetEncoder() {
    leverArmDrive.setPower(0.0);
    leverArmDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    targetPosition = 0;
    targetPositioIndex = 0;
    leverArmDrive.setTargetPosition(targetPosition);
    opMode = MotorOpMode.IDLE;
  }

  public void raise(double raise, Telemetry telemetry) {
    opMode = MotorOpMode.MANUAL;
    raise = raise * LEVER_MOTOR_POWER;
    leverArmDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    leverArmDrive.setPower(raise);

    int encoderPosition = leverArmDrive.getCurrentPosition();
    telemetry.addData("leverArm", "pow %.2f pos %d", raise, encoderPosition);
  }

  public void setPosition(int position, Telemetry telemetry) {
    int curEncoderPosition;
    boolean newPositionValid = false;

    newPositionValid = (position >= 0) && (position <= 4);
    curEncoderPosition = leverArmDrive.getCurrentPosition();

    if (newPositionValid) {
      opMode = MotorOpMode.ENCODER;
      leverArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      targetPositioIndex = position;
      targetPosition = encoderPosition[position];
      leverArmDrive.setTargetPosition(targetPosition);
      leverArmDrive.setPower(LEVER_MOTOR_POWER);
    }

    boolean motorBusy = leverArmDrive.isBusy();
    boolean withinTargetRange = Math.abs(curEncoderPosition - targetPosition) < 10;

    if ((opMode == MotorOpMode.ENCODER) && withinTargetRange && motorBusy) {
      stop();
    }

    telemetry.addData("leverArm", "position:idx %d cur %d tgt %d busy %b",
      targetPositioIndex, curEncoderPosition, targetPosition, motorBusy);
  }

  public void stop() {
    opMode = MotorOpMode.IDLE;
    leverArmDrive.setPower(0.0);
  }
}
