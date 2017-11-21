package org.firstinspires.ftc.teamcode;

import java.lang.Math;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MyColorSensor {

  ModernRoboticsI2cColorSensor colorSensor;

  public void init(HardwareMap hardwaremap) {
    colorSensor = hardwaremap.get(ModernRoboticsI2cColorSensor.class, "ColorSensor");
    colorSensor.enableLed(true);
  }

  public void getColor(Telemetry telemetry){

    //int alpha = colorSensor.alpha();
    //int red = colorSensor.red();
    //int green = colorSensor.green();
    //int blue = colorSensor.blue();

    //telemetry.addData("colorSensor", "A %d R %d G %d B %d", alpha, red, green, blue);

    NormalizedRGBA result = colorSensor.getNormalizedColors();
    float alpha = result.alpha;
    float red = result.red;
    float green = result.green;
    float blue = result.blue;

    telemetry.addData("colorSensor", "A %.2f R %.2f G %.2f B %.2f", alpha, red, green, blue);

  }

  public void stop() {
    colorSensor.enableLed(false);
  }
}

