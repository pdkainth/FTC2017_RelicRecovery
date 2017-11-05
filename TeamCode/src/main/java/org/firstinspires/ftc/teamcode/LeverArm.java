package org.firstinspires.ftc.teamcode;

import java.lang.Math;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LeverArm {
  private DcMotor leverArmDrive = null;

  public void init(HardwareMap hardwaremap) {
    leverArmDrive = hardwaremap.get(DcMotor.class, "Motor5");
    leverArmDrive.setPower(0.0);
    leverArmDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leverArmDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    leverArmDrive.setDirection(DcMotorSimple.Direction.FORWARD);
  }

  public void raise(double raise, Telemetry telemetry) {
    leverArmDrive.setPower(raise);
    int encoderPosition = leverArmDrive.getCurrentPosition();
    telemetry.addData("leverArm", "power %.2f position %d", raise, encoderPosition);
  }

  public void stop() {
    leverArmDrive.setPower(0.0);
  }
}
