package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

/**
 * Created by pdkainth on 10/22/2017.
 */

@TeleOp(name = "Manual Control")
public class MyTeleOp extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();
  private Mecanum wheels = new Mecanum();
  private LeverArm arm1 = new LeverArm();

  @Override
  public void init() {
    telemetry.addData("Status", "Initializing");

    wheels.init(hardwareMap);
    arm1.init(hardwareMap);

    telemetry.addData("Status", "Initialized");
  }

  @Override
  public void init_loop() {

  }

  @Override
  public void start() {
    runtime.reset();
  }

  @Override
  public void loop() {
    double leftPower;
    double rightPower;

    double drive = -gamepad1.left_stick_y;
    double strafe = gamepad1.left_stick_x;
    double rotate = gamepad1.right_stick_x;

    wheels.drive(drive, strafe, rotate, telemetry);
    telemetry.addData("gamepad1 wheels", "drive(%.2f) strafe(%.2f) rotate (%.2f)", drive, strafe, rotate);

    double raise = -gamepad1.right_stick_y;
    telemetry.addData("gamepad1 arm1", "raise (%.2f)", raise);
    arm1.raise(raise, telemetry);

    telemetry.addData("Status", "Run Time: " + runtime.toString());
  }

  @Override
  public void stop() {
    wheels.stop();
    arm1.stop();
  }
}
