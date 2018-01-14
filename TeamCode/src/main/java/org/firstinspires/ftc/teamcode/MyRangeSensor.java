package org.firstinspires.ftc.teamcode;

import java.lang.Math;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MyRangeSensor {

  ModernRoboticsI2cRangeSensor sensorDistance;

  public void init(HardwareMap hardwaremap) {
    sensorDistance = hardwaremap.get(ModernRoboticsI2cRangeSensor.class, "RangeSensor");
  }

  public double getDistance(Telemetry telemetry){

    double distance = sensorDistance.getDistance(DistanceUnit.CM);

    telemetry.addData("distanceSensor", "range %.2f cm", distance);
    return distance;
  }

  public double getDistanceInch(Telemetry telemetry){

    double distance = sensorDistance.getDistance(DistanceUnit.INCH);
    if (distance == DistanceSensor.distanceOutOfRange) {
      distance = -1.0;
    }

    telemetry.addData("distanceSensor", "range %.2f inch", distance);
    return distance;
  }

  public void stop() {
  }
}

