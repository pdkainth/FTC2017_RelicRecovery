package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by pdkainth on 10/22/2017.
 */

@TeleOp(name = "Manual Control")
public class MyTeleOp extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();
  private Mecanum wheels = new Mecanum();
  private LeverArm arm1 = new LeverArm();
  private GrabberArm arm2 = new GrabberArm();

  @Override
  public void init() {
    telemetry.addData("Status", "Initializing");

    wheels.init(hardwareMap);
    arm1.init(hardwareMap);
    arm2.init(hardwareMap);

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

    double drive = -gamepad1.left_stick_y;
    double strafe = gamepad1.left_stick_x;
    double rotate = gamepad1.right_stick_x;

    wheels.drive(drive, strafe, rotate, telemetry);
    telemetry.addData("gamepad1 wheels", "drive(%.2f) strafe(%.2f) rotate (%.2f)", drive, strafe, rotate);

    //double raise = -gamepad1.right_stick_y;
    //telemetry.addData("gamepad1 arm1", "raise (%.2f)", raise);
    //arm1.raise(raise, telemetry);

    boolean position0 = gamepad1.right_bumper;
    boolean position1 = gamepad1.y;
    boolean position2 = gamepad1.b;
    boolean position3 = gamepad1.a;
    boolean position4 = gamepad1.x;

    if (position0 == true) {
      arm1.setPosition(0, telemetry);
    } else if (position1 == true) {
      arm1.setPosition(1, telemetry);
    } else if (position2 == true) {
      arm1.setPosition(2, telemetry);
    } else if (position3 == true) {
      arm1.setPosition(3, telemetry);
    } else if (position4 == true) {
      arm1.setPosition(4, telemetry);
    } else {
      arm1.setPosition(-1, telemetry);
    }


    //double positionLeft  = gamepad2.left_stick_x;
    //double positionRight = gamepad2.right_stick_x;
    //arm2.update(positionLeft, positionRight, telemetry);

    if (gamepad2.right_bumper == true){
      arm2.open(telemetry);
    } else if (gamepad2.left_bumper == true) {
      arm2.close(telemetry);
    }

    telemetry.addData("Status", "Run Time: " + runtime.toString());
  }


  @Override
  public void stop() {
    wheels.stop();
    arm1.stop();
  }
}
