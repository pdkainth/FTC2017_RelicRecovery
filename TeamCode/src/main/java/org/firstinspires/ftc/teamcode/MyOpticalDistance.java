package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;

public class MyOpticalDistance {

  ModernRoboticsAnalogOpticalDistanceSensor odsSensor;  // Hardware Device Object

  public void init(HardwareMap hardwaremap) {
    // get a reference to our Light Sensor object.
    odsSensor = hardwaremap.get(ModernRoboticsAnalogOpticalDistanceSensor.class, "OdsSensor");
    odsSensor.enableLed(true);
  }

  public double getRawData() {
    return odsSensor.getRawLightDetected();
  }

  public double getData() {
    return odsSensor.getLightDetected();
  }

  public void stop() {
    odsSensor.enableLed(false);
  }
}
