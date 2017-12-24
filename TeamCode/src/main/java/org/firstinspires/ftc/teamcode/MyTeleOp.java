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

  public enum ButtonState {
    PRESSED, RELEASED
  }

  private ElapsedTime runtime = new ElapsedTime();
  private Mecanum wheels = new Mecanum();
  private LeverArm arm1 = new LeverArm();
  private GrabberArm arm2 = new GrabberArm();
  private MyRangeSensor distanceRange = new MyRangeSensor();

  @Override
  public void init() {
    telemetry.addData("Status", "Initializing");

    wheels.init(hardwareMap);
    arm1.init(hardwareMap);
    arm2.init(hardwareMap);
    distanceRange.init(hardwareMap);

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
    updateArm1(); // lever ARM
    updateArm2(); // Grabber Arm
    updatedistanceRange();

    telemetry.addData("Status", "Run Time: " + runtime.toString());
  }

  @Override
  public void stop() {
    wheels.stop();
    arm1.stop();
    arm2.stop();
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
    wheels.drive(drive, strafe, rotate, telemetry, distanceRange, override);
    telemetry.addData("gamepad1 wheels", "D(%.2f) S(%.2f) R(%.2f)", drive, strafe, rotate);
  }

  private void updateArm1() {
    int position;
    boolean position0 = gamepad1.right_bumper;
    boolean position1 = gamepad1.y;
    boolean position2 = gamepad1.b;
    boolean position3 = gamepad1.a;
    boolean position4 = gamepad1.x;

    double raise = 0.0;
    if (gamepad1.dpad_up) {
      raise = 1.0;
    }
    if (gamepad1.dpad_down) {
      raise = -1.0;
    }
    boolean override = (gamepad1.right_trigger > 0.7);

    if (gamepad1.start) {
      arm1.resetEncoder();
    } else if (raise != 0) {
      arm1.raise(raise, override, telemetry);
    } else {
      if (arm1.getOpState() == LeverArm.MotorOpMode.MANUAL) {
        arm1.stop();
      }

      if (position0 == true) {
        position = 0;
      } else if (position1 == true) {
        position = 1;
      } else if (position2 == true) {
        position = 2;
      } else if (position3 == true) {
        position = 3;
      } else if (position4 == true) {
        position = 4;
      } else {
        position = -1;
      }
      arm1.setPosition(position, telemetry);
    }
  }

  private void testArm2() {
    double positionLeft = gamepad2.left_stick_x;
    double positionRight = gamepad2.right_stick_x;
    arm2.update(positionLeft, positionRight, telemetry);
  }

  private void updateArm2() {
    ButtonState buttonCond;

    if (gamepad1.left_bumper == true) {
      buttonCond = ButtonState.PRESSED;
    } else {
      buttonCond = ButtonState.RELEASED;
    }

    arm2.update(buttonCond, telemetry);
  }

  private void updatedistanceRange() {
    distanceRange.getDistance(telemetry);
  }


}
