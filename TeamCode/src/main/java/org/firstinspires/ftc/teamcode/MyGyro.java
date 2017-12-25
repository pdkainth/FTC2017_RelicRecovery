package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MyGyro {
  ModernRoboticsI2cGyro mrGyro;
  boolean calibrationInProgress = false;

  public void init(HardwareMap hardwaremap) {

    boolean lastResetState = false;
    boolean curResetState  = false;

    // Get a reference to a Modern Robotics gyro object.
    mrGyro = hardwaremap.get(ModernRoboticsI2cGyro.class, "GyroSensor");

    // Start calibrating the gyro. This takes a few seconds and is worth performing
    // during the initialization phase at the start of each opMode.
    mrGyro.calibrate();
    calibrationInProgress = true;
  }

  public void init_loop(Telemetry telemetry) {
    if (calibrationInProgress) {
      if (mrGyro.isCalibrating()) {
        telemetry.addData("gyro", "Gyro Calibrating. Do Not Move!");
      } else {
        calibrationInProgress = false;
      }
    }

    if (calibrationInProgress == false) {
      int integratedZ = mrGyro.getIntegratedZValue();
      telemetry.addData("gyro", "Gyro Calibrated, intZ %d", integratedZ);
    }
  }

  public void startCalibration() {
    mrGyro.calibrate();
    calibrationInProgress = true;
  }

  public boolean isCalibrated() { return (calibrationInProgress == false); }

  public void resetZaxis() {
    mrGyro.resetZAxisIntegrator();
  }

  public int getZaxis(Telemetry telemetry) {
    int integratedZ = mrGyro.getIntegratedZValue();
    int integratedZMod360 = integratedZ % 360;
    int integratedZMod180 = integratedZMod360;
    if (integratedZMod180 >= 180) { integratedZMod180 -= 360; }
    if (integratedZMod180 < -180) { integratedZMod180 += 360; }

    telemetry.addData("gyro", "zAxis, int %d mod360 %d mod180 %d ", integratedZ, integratedZMod360, integratedZMod180);
    return integratedZMod180;
  }
}
