package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MyColorSensor {

  private ModernRoboticsI2cColorSensor colorSensor;

  public void init(HardwareMap hardwaremap, String name) {
    colorSensor = hardwaremap.get(ModernRoboticsI2cColorSensor.class, name);
    colorSensor.enableLed(true);
  }

  public int getColor(Telemetry telemetry){

    int index = colorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER);
    telemetry.addData("colorSensor", "colorIdx %d", index);
    return index;
  }

  public void stop() {
    colorSensor.enableLed(false);
  }
}

