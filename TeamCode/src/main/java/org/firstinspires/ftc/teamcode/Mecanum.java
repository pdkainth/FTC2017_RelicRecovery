package org.firstinspires.ftc.teamcode;

import java.lang.Math;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Mecanum {
  private DcMotor frontLeftDrive = null;
  private DcMotor frontRightDrive = null;
  private DcMotor backLeftDrive = null;
  private DcMotor backRightDrive = null;
  boolean scalingOn = true;

  private final static double MAX_HAZARD_DISTANCE = 60.0;
  private final static double MIN_HAZARD_DISTANCE = 35.0;
  private static final int ENC_TICK_PER_REV = 1460;

  public void init(HardwareMap hardwareMap) {
    frontLeftDrive = hardwareMap.get(DcMotor.class, "Motor4");
    frontRightDrive = hardwareMap.get(DcMotor.class, "Motor3");
    backLeftDrive = hardwareMap.get(DcMotor.class, "Motor2");
    backRightDrive = hardwareMap.get(DcMotor.class, "Motor1");

    frontLeftDrive.setPower(0.0);
    frontRightDrive.setPower(0.0);
    backLeftDrive.setPower(0.0);
    backRightDrive.setPower(0.0);

    frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
    backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    backRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

    resetEncoder();
    scalingOn = true;
  }

  public void setScaling(boolean flag) {
    scalingOn = flag;
  }

  public void resetEncoder() {
    backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

  public int getEncoder() {
    return backRightDrive.getCurrentPosition();
  }

  public int inchToEncCnt(double inch, boolean encReset) {
    int bbOffEncoderCnt;
    bbOffEncoderCnt = (int)((inch * ENC_TICK_PER_REV) / (4 * Math.PI));
    if (encReset == true) {
      resetEncoder();
    }
    return bbOffEncoderCnt;
  }

  public void drive(double drive, double strafe, double rotate, Telemetry telemetry,
                    MyRangeSensor distanceRange, boolean override) {

    if (override == false) {
      double newDrive;
      double curDistance = distanceRange.getDistance(telemetry);

      if ((curDistance < MAX_HAZARD_DISTANCE) && (drive > 0.0)) {
        newDrive = (curDistance - MIN_HAZARD_DISTANCE) / (MAX_HAZARD_DISTANCE - MIN_HAZARD_DISTANCE);
        drive = Math.min(drive, newDrive);
        drive = Math.max(0, drive);
      }
    }

    if (scalingOn) {
      drive = drive * 0.6;
      strafe = strafe * 0.5;
      rotate = rotate * 0.5;
    }

    // calculate the motor power
    double frontLeftPower = drive + strafe + rotate;
    double backLeftPower = drive - strafe + rotate;
    double frontRightPower = drive - strafe - rotate;
    double backRightPower = drive + strafe - rotate;

    // normalize to maximum between -1.0 and +1.0
    double maxPower =
      Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

    if (maxPower > 1.0) {
      frontLeftPower /= maxPower;
      backLeftPower /= maxPower;
      frontRightPower /= maxPower;
      backRightPower /= maxPower;
    }

    // scale down the power to avoid sudden movement
    frontLeftPower *= 0.3;
    backLeftPower *= 0.3;
    frontRightPower *= 0.3;
    backRightPower *= 0.3;

    //frontLeftPower  = Range.clip(frontLeftPower, -1.0, 1.0);
    //backLeftPower   = Range.clip(backLeftPower, -1.0, 1.0);
    //frontRightPower = Range.clip(frontRightPower, -1.0, 1.0);
    //backRightPower  = Range.clip(backRightPower, -1.0, 1.0);

    frontLeftDrive.setPower(frontLeftPower);
    frontRightDrive.setPower(frontRightPower);
    backLeftDrive.setPower(backLeftPower);
    backRightDrive.setPower(backRightPower);

    int encPosition = backRightDrive.getCurrentPosition();

    telemetry.addData("mecanum", "power FL %.2f FR %.2f BL %.2f BR %.2f pos %d",
      frontLeftPower, frontRightPower, backLeftPower, backRightPower, encPosition);
  }

  public void stop() {
    frontLeftDrive.setPower(0.0);
    frontRightDrive.setPower(0.0);
    backLeftDrive.setPower(0.0);
    backRightDrive.setPower(0.0);
  }
}

