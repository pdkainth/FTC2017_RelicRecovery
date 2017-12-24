package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test Jewel Arm")
public class Test_JewelArm extends OpMode {

  private JewelArm jewelArm = new JewelArm();
  private GrabberArm grabberArm = new GrabberArm();
  private MyColorSensor colorSensor = new MyColorSensor();

  @Override
  public void init() {
    jewelArm.init(hardwareMap);
    grabberArm.init(hardwareMap);
    colorSensor.init(hardwareMap, "ColorSensor1");
    telemetry.addData("Status", "Initializing");
  }

  @Override
  public void init_loop() {
  }

  @Override
  public void start() {
  }

  @Override
  public void loop() {
    if(gamepad2.dpad_up){
      jewelArm.up();
    } else if(gamepad2.dpad_down){
      jewelArm.down();
    } else {
//      jewelArm.update(gamepad2.left_stick_x, telemetry);
    }

    colorSensor.getColor(telemetry);
  }

  @Override
  public void stop() {
    colorSensor.stop();
    jewelArm.stop();
  }
}