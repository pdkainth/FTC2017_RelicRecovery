package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

public class MyAutoDrive {
  private ElapsedTime runtime = new ElapsedTime();
  public enum FieldMode {STRAIGHT, SQUARE}
  public enum GlyphColor {GRAY, BROWN}
  public enum AllianceColor {RED, BLUE}

  private boolean running = false;

  private FieldMode fieldMode;
  private GlyphColor glyphColor;
  private AllianceColor allianceColor;
  private int selectGlyphColumn = 0;

  private Mecanum wheels = new Mecanum();
  private LeverArm arm1 = new LeverArm();
  private GrabberArm arm2 = new GrabberArm();
  private MyRangeSensor distanceRange = new MyRangeSensor();

  public void init(HardwareMap hardwareMap, Telemetry telemetry, FieldMode fieldMode,
                   AllianceColor allianceColor, GlyphColor glyphColor) {
    telemetry.addData("AutoStatus", "Initializing");

    this.fieldMode = fieldMode;
    this.glyphColor = glyphColor;
    this.allianceColor = allianceColor;


    wheels.init(hardwareMap);
    arm1.init(hardwareMap);
    arm2.init(hardwareMap);
    distanceRange.init(hardwareMap);

    telemetry.addData("AutoStatus", "Initialized F %s G %s A %s",
      this.fieldMode, this.glyphColor, this.allianceColor);
  }

  public void init_loop(Telemetry telemetry, Gamepad gamepad1) {
    updateDrive(telemetry, gamepad1);
    updateArm1(telemetry, gamepad1);
    updateArm2(telemetry, gamepad1);
    updatedistanceRange(telemetry);
  }

  public void start(Telemetry telemetry) {
    runtime.reset();
    running = true;
    telemetry.addData("AutoStatus", "Started F %s G %s A %s",
      fieldMode, glyphColor, allianceColor);
 }

  public void loop(Telemetry telemetry) {
    if((runtime.seconds() > 30.0) && running){
      stop();
    }
    telemetry.addData("AutoStatus", "Loop Run Time: %s F %s G %s A %s",
      runtime.toString(), fieldMode, glyphColor, allianceColor);
  }

  public void stop() {
    wheels.stop();
    arm1.stop();
    arm2.stop();
    running = false;
  }

  private void updateDrive(Telemetry telemetry, Gamepad gamepad1) {
    double drive = -gamepad1.left_stick_y;
    double strafe = 0.0;

    if (gamepad1.dpad_left == true) {
      strafe = -1.0;
    } else if (gamepad1.dpad_right == true) {
      strafe = 1.0;
    }

    double rotate = gamepad1.right_stick_x;

    wheels.drive(drive, strafe, rotate, telemetry, distanceRange, true);
    telemetry.addData("gamepad1 wheels", "D(%.2f) S(%.2f) R(%.2f)", drive, strafe, rotate);
  }

  private void updateArm1(Telemetry telemetry, Gamepad gamepad1) {
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
    }
  }

  private void updateArm2(Telemetry telemetry, Gamepad gamepad1) {
    GrabberArm.ButtonState buttonCond;

    if (gamepad1.left_bumper == true) {
      buttonCond = GrabberArm.ButtonState.PRESSED;
    } else {
      buttonCond = GrabberArm.ButtonState.RELEASED;
    }

    arm2.update(buttonCond, telemetry);
  }

  private void updatedistanceRange(Telemetry telemetry) {
    distanceRange.getDistance(telemetry);
  }
}