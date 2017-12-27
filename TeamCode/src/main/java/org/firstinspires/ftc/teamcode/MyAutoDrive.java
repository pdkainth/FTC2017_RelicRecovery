package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

public class MyAutoDrive {
  private ElapsedTime runtime = new ElapsedTime();
  public enum FieldMode {STRAIGHT, SQUARE}
  public enum AllianceColor {UNKNOWN, RED, BLUE}

  private boolean running = false;

  private FieldMode fieldMode;
  private AllianceColor allianceColor;
  private int selectGlyphColumn = 0;

  private Mecanum wheels = new Mecanum();
  private LeverArm arm1 = new LeverArm();
  private GrabberArm arm2 = new GrabberArm();
  private MyRangeSensor distanceRange = new MyRangeSensor();
  private VuMark_Nav vumark = new VuMark_Nav();
  private JewelArm jewelArm = new JewelArm();
  private MyColorSensor colorSensor = new MyColorSensor();
  private MyGyro gyro = new MyGyro();

  public enum AutoState {
    IDLE {
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        return IDLE;
      }
    },
    START {
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        return GET_VUMARK;
      }
    },
    GET_VUMARK {
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        autoDrive.vumark.scan(telemetry);
        RelicRecoveryVuMark curVumark = autoDrive.vumark.getCurVumark();
        OpenGLMatrix curPose = autoDrive.vumark.getCurPose();
        if ((curVumark == RelicRecoveryVuMark.UNKNOWN) || (curPose == null)) {
          autoDrive.wheels.drive(0.5, 0, 0, telemetry, autoDrive.distanceRange, true);
          autoDrive.wheels.stop();
          return GET_VUMARK;
        }
        else {
          if (curVumark == RelicRecoveryVuMark.LEFT) {
            autoDrive.selectGlyphColumn = 0;
          } else if (curVumark == RelicRecoveryVuMark.CENTER) {
            autoDrive.selectGlyphColumn = 1;
          } else if (curVumark == RelicRecoveryVuMark.RIGHT) {
            autoDrive.selectGlyphColumn = 2;
          }
          Orientation orientation = autoDrive.vumark.getVumarkOrient();
          double yRotError = 0 - orientation.secondAngle;
          autoDrive.gyro.setZaxisOffset((int)Math.round(yRotError));
          return GOTO_JEWEL_TARGET;
        }
      }
    },
    GOTO_JEWEL_TARGET {
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        autoDrive.vumark.scan(telemetry);
        VectorF trans = autoDrive.vumark.getVumarkTrans();
        Orientation orientation = autoDrive.vumark.getVumarkOrient();
        double zError = -449 - trans.get(2);
        double xError = trans.get(0) - 160;
        double yRotError = 0 - orientation.secondAngle;

        boolean reachedAtTarget = (Math.abs(zError) < 5) && (Math.abs(xError) < 5) && (yRotError < 1);

        if (reachedAtTarget == false) {
          double drivePower = 0.0;
          double strafePower = 0.0;
          double rotPower = 0.0;
          if (Math.abs(xError) >= 5) {
            drivePower = xError / 100.0;
          }
          if (Math.abs(zError) >= 5) {
            strafePower = zError / 100.0;
          }
          if (Math.abs(yRotError) >= 1) {
            rotPower = yRotError / 30.0;
          }
          autoDrive.wheels.setScaling(false);
          autoDrive.wheels.drive(drivePower, strafePower, rotPower, telemetry, autoDrive.distanceRange, true);
          autoDrive.wheels.stop();
          return GOTO_JEWEL_TARGET;
        } else {
          autoDrive.gyro.resetZaxis();
          autoDrive.jewelKnockOutState = JewelKnockOutState.START;
          return KNOCK_OUT_JEWEL;
        }
      }
    },
    KNOCK_OUT_JEWEL {
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        autoDrive.jewelKnockOutState = autoDrive.jewelKnockOutState.update(autoDrive, telemetry);
        if (autoDrive.jewelKnockOutState != JewelKnockOutState.IDLE) {
          return KNOCK_OUT_JEWEL;
        } else {
          return IDLE;
        }
      }
    },
    GET_VUMARK_FOR_GLYPH_DRIVE {
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        return IDLE;
      }
    };

    public abstract AutoState update(MyAutoDrive autoDrive, Telemetry telemetry);
  } // end of AutoState enum

  public enum JewelKnockOutState {
    IDLE {
      public JewelKnockOutState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        return IDLE;
      }
    },
    START {
      public JewelKnockOutState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        autoDrive.jewelColor = AllianceColor.UNKNOWN;
        return JEWEL_ARM_DOWN;
      }
    },
    JEWEL_ARM_DOWN {
      public JewelKnockOutState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        autoDrive.jewelArm.down();
        return GET_COLOR_NUMBER;
      }
    },
    GET_COLOR_NUMBER {
      public JewelKnockOutState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        int colorIdx = autoDrive.colorSensor.getColor(telemetry);
        if (colorIdx == 0) {
          autoDrive.wheels.drive(0.5, 0, 0, telemetry, autoDrive.distanceRange, true);
          autoDrive.wheels.stop();
          return GET_COLOR_NUMBER;
        } else {
          if ((colorIdx >= 2) && (colorIdx <= 4)) {
            autoDrive.jewelColor = AllianceColor.BLUE;
          } else if ((colorIdx >= 9) && (colorIdx <= 11)) {
            autoDrive.jewelColor = AllianceColor.RED;
          }

          if (autoDrive.jewelColor == AllianceColor.UNKNOWN) {
            return GET_COLOR_NUMBER;
          } else {
            return ROTATE_FORWARD;
          }
        }
      }
    },
    ROTATE_FORWARD {
      public JewelKnockOutState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        double rotPowerDir;
        if (autoDrive.jewelColor == autoDrive.allianceColor) {
          rotPowerDir = 1.0;
        } else {
          rotPowerDir = -1.0;
        }
        autoDrive.wheels.setScaling(true);
        autoDrive.wheels.drive(0, 0, 0.5 * rotPowerDir, telemetry, autoDrive.distanceRange, true);
        autoDrive.jewelRotStartTime = autoDrive.runtime.milliseconds();
        return WAIT_ROTATE_FORWARD;
      }
    },
    WAIT_ROTATE_FORWARD {
      public JewelKnockOutState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        //double curTime = autoDrive.runtime.milliseconds();
        int rotError = autoDrive.gyro.getZaxis(telemetry);
        //if ((curTime - autoDrive.jewelRotStartTime) < 1000) {
        if (Math.abs(rotError) < 10) {
          return WAIT_ROTATE_FORWARD;
        } else {
          autoDrive.wheels.stop();
          return JEWEL_ARM_UP;
        }
      }
    },
    JEWEL_ARM_UP {
      public JewelKnockOutState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        autoDrive.jewelArm.up();
        return ROTATE_BACKWARD;
      }
    },
    ROTATE_BACKWARD {
      public JewelKnockOutState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        //double rotPowerDir;
        //if (autoDrive.jewelColor == autoDrive.allianceColor) {
        //  rotPowerDir = -1.0;
        //} else {
        //  rotPowerDir = 1.0;
        //}
        int rotError = autoDrive.gyro.getZaxis(telemetry);
        double rotPower = Range.scale((double)rotError, -180.0, 179.0, -1.0, 1.0);
        rotPower = Range.scale(Math.abs(rotPower), 0, 1, 0.2, 1) * Math.signum(rotPower);
        autoDrive.wheels.setScaling(false);
        autoDrive.wheels.drive(0, 0, rotPower, telemetry, autoDrive.distanceRange, true);
        autoDrive.jewelRotStartTime = autoDrive.runtime.milliseconds();
        return WAIT_ROTATE_BACKWARD;
      }
    },
    WAIT_ROTATE_BACKWARD {
      public JewelKnockOutState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        //double curTime = autoDrive.runtime.milliseconds();
        int rotError = autoDrive.gyro.getZaxis(telemetry);
        double rotPower = 0.0;
        //if ((curTime - autoDrive.jewelRotStartTime) < 1000) {
        if (Math.abs(rotError) > 1) {
          rotPower = Range.scale((double)rotError, -180.0, 179.0, -1.0, 1.0);
          rotPower = Range.scale(Math.abs(rotPower), 0, 1, 0.2, 1) * Math.signum(rotPower);
          autoDrive.wheels.drive(0, 0, rotPower, telemetry, autoDrive.distanceRange, true);
          return WAIT_ROTATE_BACKWARD;
        } else {
          autoDrive.wheels.stop();
          return IDLE;
        }
      }
    };

    public abstract JewelKnockOutState update(MyAutoDrive autoDrive, Telemetry telemetry);
  } // end of JewelKnockOutState enum

  private AutoState autoState;
  private JewelKnockOutState jewelKnockOutState;
  private AllianceColor jewelColor;
  double jewelRotStartTime;

  public void init(HardwareMap hardwareMap, Telemetry telemetry, FieldMode fieldMode,
                   AllianceColor allianceColor) {
    telemetry.addData("AutoStatus", "Initializing");

    this.fieldMode = fieldMode;
    this.allianceColor = allianceColor;

    wheels.init(hardwareMap);
    arm1.init(hardwareMap);
    arm2.init(hardwareMap);
    distanceRange.init(hardwareMap);
    jewelArm.init(hardwareMap);
    colorSensor.init(hardwareMap, "ColorSensor1");
    gyro.init(hardwareMap);
    vumark.init(hardwareMap);

    autoState = AutoState.IDLE;
    jewelKnockOutState = JewelKnockOutState.IDLE;
    jewelColor = AllianceColor.UNKNOWN;
    jewelRotStartTime = 0.0;

    telemetry.addData("AutoStatus", "Initialized F %s A %s",
      this.fieldMode, this.allianceColor);
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
    autoState = AutoState.START;
    vumark.start();
    telemetry.addData("AutoStatus", "Started F %s A %s",
      fieldMode, allianceColor);
 }

  public void loop(Telemetry telemetry) {
    if((runtime.seconds() > 30.0) && running){
      stop();
    } else {
      autoState = autoState.update(this, telemetry);
    }
    telemetry.addData("AutoStatus", "runtime: %s F %s A %s states auto %s jewel %s",
      runtime.toString(), fieldMode, allianceColor, autoState, jewelKnockOutState);
  }

  public void stop() {
    wheels.stop();
    arm1.stop();
    arm2.stop();
    jewelArm.stop();
    colorSensor.stop();
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