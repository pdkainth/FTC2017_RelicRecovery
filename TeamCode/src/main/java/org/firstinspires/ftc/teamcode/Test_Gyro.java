package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Test Gyro")
public class Test_Gyro extends OpMode {
  private ElapsedTime runtime = new ElapsedTime();
  private Mecanum wheels = new Mecanum();
  private MyRangeSensor distanceRange = new MyRangeSensor();
  private MyGyro gyro = new MyGyro();

  boolean reorientInProgress = false;

  @Override
  public void init() {
    telemetry.addData("Status", "Initializing");

    wheels.init(hardwareMap);
    distanceRange.init(hardwareMap);
    gyro.init(hardwareMap);

    telemetry.addData("Status", "Initialized");
  }

  @Override
  public void init_loop() {
    gyro.init_loop(telemetry);
  }

  @Override
  public void start() {
    gyro.setZaxisOffset(30);
    runtime.reset();
  }

  @Override
  public void loop() {

    updateOrientation();

    if (reorientInProgress == false) {
      updateDrive();
    }
    updatedistanceRange();
    updateGyro();

    telemetry.addData("Status", "Run Time: %s reOrient %b", runtime.toString(), reorientInProgress);
  }

  @Override
  public void stop() {
    wheels.stop();
  }

  private void updateDrive() {
    double drive = -gamepad1.left_stick_y;
    double strafe = 0.0;

    if (gamepad1.dpad_left == true){
      strafe = -1.0;
    } else if (gamepad1.dpad_right == true){
      strafe = 1.0;
    }

    double rotate = gamepad1.right_stick_x;

    boolean override = (gamepad1.left_trigger > 0.7);
    wheels.drive(drive, strafe, rotate, telemetry, distanceRange,override);
    telemetry.addData("gamepad1 wheels drive", "D(%.2f) S(%.2f) R(%.2f)", drive, strafe, rotate);
  }

  private void updatedistanceRange() {
    distanceRange.getDistance(telemetry);
  }

  private void updateOrientation() {
    if (reorientInProgress == false) {
      if (gamepad1.x) {
        // stop manual drive
        wheels.drive(0, 0, 0, telemetry, distanceRange, true);
        wheels.setScaling(false);
        telemetry.addData("gamepad1 wheels reorient 1", "D(%.2f) S(%.2f) R(%.2f)", 0.0, 0.0, 0.0);
        reorientInProgress = true;
      }
    }

    if (reorientInProgress == true) {
      int zAxisRot = gyro.getZaxis(telemetry);

      if (Math.abs(zAxisRot) <= 1) {
        wheels.drive(0, 0, 0, telemetry, distanceRange, true);
        telemetry.addData("gamepad1 wheels", "D(%.2f) S(%.2f) R(%.2f)", 0.0, 0.0, 0.0);
        reorientInProgress = false;
        wheels.setScaling(true);
      } else {
        double rotPower = Range.scale((double)zAxisRot, -180.0, 179.0, -1.0, 1.0);
        rotPower = Range.scale(Math.abs(rotPower), 0, 1, 0.2, 1) * Math.signum(rotPower);
        wheels.drive(0, 0, rotPower, telemetry, distanceRange, true);
        telemetry.addData("gamepad1 wheels reorient 2", "D(%.2f) S(%.2f) R(%.2f)", 0.0, 0.0, rotPower);
      }
    }
  }

  private void updateGyro() {
    int zAxisRot = gyro.getZaxis(telemetry);
  }
}
