package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

@Autonomous(name = "Auto Drive : RED STRAIGHT mode BROWN glyph")
public class AutoDrive_RED_STRAIGHT_BROWN extends OpMode {
  private ElapsedTime runtime = new ElapsedTime();
  private MyAutoDrive autoDrive = new MyAutoDrive();

  @Override
  public void init() {
    telemetry.addData("Status", "Initializing");
    autoDrive.init(hardwareMap, telemetry, MyAutoDrive.FieldMode.STRAIGHT,
      MyAutoDrive.AllianceColor.RED, MyAutoDrive.GlyphColor.BROWN);
    telemetry.addData("Status", "Initialized");
  }

  @Override
  public void init_loop() {
    autoDrive.init_loop(telemetry, gamepad1);
  }

  @Override
  public void start() {
    runtime.reset();
    autoDrive.start(telemetry);
  }

  @Override
  public void loop() {
    double timeElapsed = runtime.seconds();
    if (timeElapsed >= 30.0) {
      stop();
    }

    autoDrive.loop(telemetry);

    telemetry.addData("Status", "Run Time: " + runtime.toString());
  }

  @Override
  public void stop() {
    autoDrive.stop();
  }
}
