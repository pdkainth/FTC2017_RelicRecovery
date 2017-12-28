package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Test wheels")
public class Test_wheels extends OpMode {
  private ElapsedTime runtime = new ElapsedTime();
  private Mecanum wheels = new Mecanum();

  @Override
  public void init() {
    telemetry.addData("Status", "Initializing");

    wheels.init(hardwareMap);

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

    updateDrive();

    telemetry.addData("Status", "Run Time: " + runtime.toString());
  }

  @Override
  public void stop() {
    wheels.stop();
  }

  private void updateDrive() {
    double drive = -gamepad1.left_stick_y;
    double strafe = 0.0;

    if (gamepad1.dpad_left == true) {
      strafe = -1.0;
    } else if (gamepad1.dpad_right == true) {
      strafe = 1.0;
    }

    double rotate = gamepad1.right_stick_x;

    boolean override = (gamepad1.left_trigger > 0.7);
    wheels.drive(drive, strafe, rotate, telemetry, null, true);
    telemetry.addData("gamepad1 wheels", "D(%.2f) S(%.2f) R(%.2f)", drive, strafe, rotate);
  }

}
