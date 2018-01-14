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

  private Mecanum wheels = new Mecanum();
  private LeverArm arm1 = new LeverArm();
  private GrabberArm arm2 = new GrabberArm();
  private MyRangeSensor distanceRange = null;
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
        // HACK - to skip jewel knock off
        //return GET_VUMARK_OFF_BALANCE_BOARD;
        return GET_VUMARK;
      }
    },
    GET_VUMARK {
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        // Plulse drive forward slowly while scanning for VU mark
        autoDrive.vumark.scan(telemetry);
        RelicRecoveryVuMark curVumark = autoDrive.vumark.getCurVumark();
        OpenGLMatrix curPose = autoDrive.vumark.getCurPose();
        if ((curVumark == RelicRecoveryVuMark.UNKNOWN) || (curPose == null)) {
          autoDrive.wheels.setScaling(true);
          autoDrive.wheels.drive(0.5, 0, 0, telemetry, autoDrive.distanceRange, true);
          autoDrive.wheels.stop();
          return GET_VUMARK;
        }
        else {
          autoDrive.selectGlyphColumn = VuMark_Nav.getVumarkIdx(curVumark);
          return GOTO_JEWEL_TARGET;
        }
      }
    },
    GOTO_JEWEL_TARGET {
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        // pulse drive towards the location to knock out jewel
        autoDrive.vumark.scan(telemetry);
        VectorF trans = autoDrive.vumark.getVumarkTrans();
        Orientation orientation = autoDrive.vumark.getVumarkOrient();
        double zError = -460 - trans.get(2);
        double xError = trans.get(0) - 160;
        double yRotError = 0 - orientation.secondAngle;

        boolean reachedAtTarget = (Math.abs(zError) < 5) && (Math.abs(xError) < 5) && (yRotError < 1);

        if (reachedAtTarget == false) {
          double drivePower = 0.0;
          double strafePower = 0.0;
          double rotPower = 0.0;
          if (Math.abs(xError) >= 5) {
            drivePower = xError / 200.0;
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
          // calibrate Gyro
          autoDrive.gyro.resetZaxis();
          autoDrive.gyro.setZaxisOffset((int)Math.round(-yRotError));

          autoDrive.jewelKnockOutState = JewelKnockOutState.START;
          return KNOCK_OUT_JEWEL;
        }
      }
    },
    KNOCK_OUT_JEWEL {
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        // knock out jeqwel
        autoDrive.jewelKnockOutState = autoDrive.jewelKnockOutState.update(autoDrive, telemetry);
        if (autoDrive.jewelKnockOutState != JewelKnockOutState.IDLE) {
          return KNOCK_OUT_JEWEL;
        } else {
          return GET_VUMARK_OFF_BALANCE_BOARD;
        }
      }
    },
    GET_VUMARK_OFF_BALANCE_BOARD {
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        // pulse drive forward while correcting rotation error to acquire VU mark again
        autoDrive.vumark.scan(telemetry);
        RelicRecoveryVuMark curVumark = autoDrive.vumark.getCurVumark();
        OpenGLMatrix curPose = autoDrive.vumark.getCurPose();
        if ((curVumark == RelicRecoveryVuMark.UNKNOWN) || (curPose == null)) {
          int rotError = autoDrive.gyro.getZaxis(telemetry);
          double rotPower = Range.scale((double)rotError, -180.0, 179.0, -1.0, 1.0);
          rotPower = Range.scale(Math.abs(rotPower), 0, 1, 0.2, 1) * Math.signum(rotPower);
          autoDrive.wheels.drive(0.5, 0, rotPower, telemetry, autoDrive.distanceRange, true);
          autoDrive.wheels.stop();
          return GET_VUMARK_OFF_BALANCE_BOARD;
        }
        else {
          autoDrive.selectGlyphColumn = VuMark_Nav.getVumarkIdx(curVumark);

          // calibrate Gyro again
          Orientation orientation = autoDrive.vumark.getVumarkOrient();
          double yRotError = 0 - orientation.secondAngle;
          autoDrive.gyro.resetZaxis();
          autoDrive.gyro.setZaxisOffset((int)Math.round(-yRotError));

          if (autoDrive.allianceColor == AllianceColor.BLUE) {
            // for Blue field, this is last time VU mark can be seen, go to final Glyph drive
            return WAIT_TIME_VUMARK_STABLE;
          } else {
            // for Red field, get off the balance board to acquire VU mark again for more accurate drive to Glyph
            autoDrive.bbOffEncoderCnt = autoDrive.wheels.inchToEncCnt(23.5 - autoDrive.vumark.getVdistance(), true);
            return GET_OFF_BALANCE_BOARD;
          }
        }
      }
    },
    GET_OFF_BALANCE_BOARD {
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        // Drive off the balance board
        if (autoDrive.glyphDrive_Move(0, 1.0, telemetry) == false) {
          return GET_OFF_BALANCE_BOARD;
        } else {
          autoDrive.wheels.stop();
          return ORIENT_FOR_VUMARK_GLYPH_DRIVE;
        }
      }
    },
    ORIENT_FOR_VUMARK_GLYPH_DRIVE {
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        // rotate towards VU mark
        if (autoDrive.glyphDrive_Orient(45, telemetry) == false) {
          return ORIENT_FOR_VUMARK_GLYPH_DRIVE;
        } else {
          autoDrive.startTime = autoDrive.runtime.milliseconds();
          return GET_VUMARK_FOR_GLYPH_DRIVE;
        }
      }
    },
    GET_VUMARK_FOR_GLYPH_DRIVE {
      // acquire the glyph again for final target position
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        autoDrive.vumark.scan(telemetry);
        RelicRecoveryVuMark curVumark = autoDrive.vumark.getCurVumark();
        OpenGLMatrix curPose = autoDrive.vumark.getCurPose();
        if ((curVumark == RelicRecoveryVuMark.UNKNOWN) || (curPose == null)) {
          return GET_VUMARK_FOR_GLYPH_DRIVE;
        } else {
          return WAIT_TIME_VUMARK_STABLE;
        }
      }
    },
    WAIT_TIME_VUMARK_STABLE {
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        if ((autoDrive.runtime.milliseconds() - autoDrive.startTime) < 1000) {
          autoDrive.vumark.scan(telemetry);
          return WAIT_TIME_VUMARK_STABLE;
        } else {
          autoDrive.vumark.scan(telemetry);

          // calculate the final glyph drive target position
          RelicRecoveryVuMark curVumark = autoDrive.vumark.getCurVumark();
          OpenGLMatrix curPose = autoDrive.vumark.getCurPose();
          if ((curVumark == RelicRecoveryVuMark.UNKNOWN) || (curPose == null)) {
            return IDLE;
          } else {
            Orientation orientation = autoDrive.vumark.getVumarkOrient();
            double yRotError = 0 - orientation.secondAngle;
            autoDrive.gyro.resetZaxis();
            autoDrive.gyro.setZaxisOffset((int) Math.round(-yRotError));

            double curPosH = autoDrive.vumark.getHdistance();
            double curPosV = autoDrive.vumark.getVdistance();
            // rotate the position to final orientation before getting target position delta
            double finalOrient;
            if (autoDrive.glyphDriveData.driveOrder[1] == GlyphDriveDir.HOR) {
              finalOrient = autoDrive.glyphDriveData.orientMoveH;
            } else {
              finalOrient = autoDrive.glyphDriveData.orientMoveV;
            }
            double curAngle = 39.0 + orientation.secondAngle;
            double finalAngle = 39.0 + finalOrient;
            curPosH = curPosH + (6.75 * (Math.sin(Math.toRadians(curAngle)) - Math.sin(Math.toRadians(finalAngle))));
            curPosV = curPosV + (6.75 * (Math.cos(Math.toRadians(finalAngle)) - Math.cos(Math.toRadians(curAngle))));

            autoDrive.glyphDriveData.targetDeltaH =
              autoDrive.glyphDriveData.targetH + autoDrive.glyphDriveData.targetOffsetH[autoDrive.selectGlyphColumn] - curPosH;
            autoDrive.glyphDriveData.driveDirH *= Math.signum(autoDrive.glyphDriveData.targetDeltaH);
            autoDrive.glyphDriveData.targetDeltaH = Math.abs(autoDrive.glyphDriveData.targetDeltaH);

            autoDrive.glyphDriveData.targetDeltaV =
              autoDrive.glyphDriveData.targetV + autoDrive.glyphDriveData.targetOffsetV[autoDrive.selectGlyphColumn] - curPosV;
            autoDrive.glyphDriveData.driveDirV *= Math.signum(autoDrive.glyphDriveData.targetDeltaV);
            autoDrive.glyphDriveData.targetDeltaV = Math.abs(autoDrive.glyphDriveData.targetDeltaV);

            autoDrive.glyphDriveTarget = GlyphDriveTarget.START;
            return GOTO_GLYPH_TARGET;
          }
        }
      }
    },
    GOTO_GLYPH_TARGET {
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        // go to Glyph target
        if (autoDrive.glyphDriveTarget != GlyphDriveTarget.IDLE) {
          autoDrive.glyphDriveTarget = autoDrive.glyphDriveTarget.update(autoDrive, telemetry);
          return GOTO_GLYPH_TARGET;
        } else {
          autoDrive.startTime = autoDrive.runtime.milliseconds();
          return DROP_GLYPH;
        }
      }
    },
    DROP_GLYPH {
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        // drop the glyph
        autoDrive.arm2.open();

        autoDrive.bbOffEncoderCnt = autoDrive.wheels.inchToEncCnt(4.0, true);
        return MOVE_BACK;
      }
    },
    MOVE_BACK {
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        // move back by 2 inches and lower lever arm to push
        if (autoDrive.glyphDrive_Move(0, -1.0, telemetry) == false) {
          return MOVE_BACK;
        } else {
          autoDrive.leverArmPosition = 0;
          autoDrive.arm1.setPosition(0, telemetry);
          autoDrive.startTime = autoDrive.runtime.milliseconds();
          return WAIT_ARM_DOWN;
        }
      }
    },
    WAIT_ARM_DOWN {
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        // wait for 3 seconds for lever arm to get down
        if ((autoDrive.runtime.milliseconds() - autoDrive.startTime) < 1000) {
          return WAIT_ARM_DOWN;
        } else {
          autoDrive.bbOffEncoderCnt = autoDrive.wheels.inchToEncCnt(6.0, true);
          autoDrive.leverArmPosition = -1;
          return PUSH_GLYPH;
        }
      }
    },
    PUSH_GLYPH {
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        // push forward by 4 inches to push glyph in
        if (autoDrive.glyphDrive_Move(0, 1.0, telemetry) == false) {
          return PUSH_GLYPH;
        } else{
          return IDLE;
        }
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
        return WAIT_ROTATE_FORWARD;
      }
    },
    WAIT_ROTATE_FORWARD {
      public JewelKnockOutState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        int rotError = autoDrive.gyro.getZaxis(telemetry);
        if (autoDrive.jewelColor == autoDrive.allianceColor) {
          rotError = -15 - rotError;
        } else {
          rotError = 15 - rotError;
        }
        if (Math.abs(rotError) > 1) {
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
        int rotError = autoDrive.gyro.getZaxis(telemetry);
        double rotPower = Range.scale((double)rotError, -180.0, 179.0, -1.0, 1.0);
        rotPower = Range.scale(Math.abs(rotPower), 0, 1, 0.2, 1) * Math.signum(rotPower);
        autoDrive.wheels.setScaling(false);
        autoDrive.wheels.drive(0, 0, rotPower, telemetry, autoDrive.distanceRange, true);
        return WAIT_ROTATE_BACKWARD;
      }
    },
    WAIT_ROTATE_BACKWARD {
      public JewelKnockOutState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        int rotError = autoDrive.gyro.getZaxis(telemetry);
        double rotPower = 0.0;
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

  public enum GlyphDriveTarget {
    IDLE {
      public GlyphDriveTarget update(MyAutoDrive autoDrive, Telemetry telemetry) {
        return IDLE;
      }
    },
    START {
      public GlyphDriveTarget update(MyAutoDrive autoDrive, Telemetry telemetry) {
        return ORIENT_MOVE_1;
      }
    },
    ORIENT_MOVE_1 {
      public GlyphDriveTarget update(MyAutoDrive autoDrive, Telemetry telemetry) {
        if (autoDrive.runGlyphOrientMove(0, telemetry) == false) {
          return ORIENT_MOVE_1;
        } else {
          return MOVE_1;
        }
      }
    },
    MOVE_1 {
      public GlyphDriveTarget update(MyAutoDrive autoDrive, Telemetry telemetry) {
        if (autoDrive.runGlyphDriveMove(0, telemetry) == false) {
          return MOVE_1;
        } else {
          return ORIENT_MOVE_2;
        }
      }
    },
    ORIENT_MOVE_2 {
      public GlyphDriveTarget update(MyAutoDrive autoDrive, Telemetry telemetry) {
        if (autoDrive.runGlyphOrientMove(1, telemetry) == false) {
          return ORIENT_MOVE_2;
        } else {
          return MOVE_2;
        }
      }
    },
    MOVE_2 {
      public GlyphDriveTarget update(MyAutoDrive autoDrive, Telemetry telemetry) {
        if (autoDrive.runGlyphDriveMove(1, telemetry) == false) {
          return MOVE_2;
        } else {
          if ((autoDrive.allianceColor == AllianceColor.BLUE) && (autoDrive.fieldMode == FieldMode.SQUARE)) {
            return SPECIAL_BLUE_SQ_ORIENT;
          } else {
            return FINAL_ORIENT;
          }
        }
      }
    },
    SPECIAL_BLUE_SQ_ORIENT {
      public GlyphDriveTarget update(MyAutoDrive autoDrive, Telemetry telemetry) {
        if (autoDrive.glyphDrive_Orient(-180, telemetry) == false) {
          return SPECIAL_BLUE_SQ_ORIENT;
        } else {
          autoDrive.bbOffEncoderCnt = autoDrive.wheels.inchToEncCnt(4.0, true);
          return SPECIAL_BLUE_SQ_DRIVE;
        }
      }
    },
    SPECIAL_BLUE_SQ_DRIVE {
      public GlyphDriveTarget update(MyAutoDrive autoDrive, Telemetry telemetry) {
        if (autoDrive.glyphDrive_Move (-180, 1.0, telemetry) == false) {
          return SPECIAL_BLUE_SQ_DRIVE;
        } else {
          return IDLE;
        }
      }
    },
    FINAL_ORIENT {
      public GlyphDriveTarget update(MyAutoDrive autoDrive, Telemetry telemetry) {
        if (autoDrive.runGlyphOrientMove(1, telemetry) == false) {
          return FINAL_ORIENT;
        } else {
          return IDLE;
        }
      }
    };

    public abstract GlyphDriveTarget update(MyAutoDrive autoDrive, Telemetry telemetry);
  }

  private enum GlyphDriveDir {HOR, VERT}

  private class GlyphDriveData {
    GlyphDriveDir[] driveOrder = new GlyphDriveDir[2];
    public double targetH;
    public double targetV;
    public int orientMoveH;
    public int orientMoveV;
    public double driveDirH;
    public double driveDirV;
    public double[] targetOffsetH = new double[3];
    public double[] targetOffsetV = new double[3];
    private double targetDeltaH;
    private double targetDeltaV;
  };

  private FieldMode fieldMode;
  private AllianceColor allianceColor;
  private int selectGlyphColumn = 0;

  private AutoState autoState;
  private JewelKnockOutState jewelKnockOutState;
  private AllianceColor jewelColor;
  private double startTime;

  private int bbOffEncoderCnt;
  private int lastEncoderCnt;
  private boolean encoderStuck;
  private double encoderStuckTimeStart;
  private boolean abortedAtEncoderStuck;

  private int leverArmPosition;

  GlyphDriveData glyphDriveData = new GlyphDriveData();
  private GlyphDriveTarget glyphDriveTarget;

  public void init(HardwareMap hardwareMap, Telemetry telemetry, FieldMode fieldMode,
                   AllianceColor allianceColor) {
    telemetry.addData("AutoStatus", "Initializing");

    this.fieldMode = fieldMode;
    this.allianceColor = allianceColor;

    wheels.init(hardwareMap);
    arm1.init(hardwareMap);
    arm2.init(hardwareMap);
    //distanceRange.init(hardwareMap);
    jewelArm.init(hardwareMap);
    colorSensor.init(hardwareMap, "ColorSensor1");
    gyro.init(hardwareMap);
    vumark.init(hardwareMap);

    autoState = AutoState.IDLE;
    jewelKnockOutState = JewelKnockOutState.IDLE;
    jewelColor = AllianceColor.UNKNOWN;

    bbOffEncoderCnt = 0;
    leverArmPosition = -1;

    if ((allianceColor == AllianceColor.RED) && (fieldMode == FieldMode.SQUARE)) {
      glyphDriveData.targetH = 24;
      glyphDriveData.targetV = 39.5;
      glyphDriveData.orientMoveH = 90;
      glyphDriveData.orientMoveV = 0;
      glyphDriveData.driveDirH = -1.0;
      glyphDriveData.driveDirV = 1.0;
      glyphDriveData.driveOrder[0] = GlyphDriveDir.HOR;
      glyphDriveData.driveOrder[1] = GlyphDriveDir.VERT;
      glyphDriveData.targetOffsetH[2] = 0.25;
      glyphDriveData.targetOffsetH[1] = glyphDriveData.targetOffsetH[2] + 7.6;
      glyphDriveData.targetOffsetH[0] = glyphDriveData.targetOffsetH[1] + 7.6;
      glyphDriveData.targetOffsetV[0] = glyphDriveData.targetOffsetV[1] = glyphDriveData.targetOffsetV[2] = -12;
    } else if ((allianceColor == AllianceColor.RED) && (fieldMode == FieldMode.STRAIGHT)) {
      glyphDriveData.targetH = 0;
      glyphDriveData.targetV = 15.5;
      glyphDriveData.orientMoveH = 90;
      glyphDriveData.orientMoveV = 0;
      glyphDriveData.driveDirH = -1.0;
      glyphDriveData.driveDirV = 1.0;
      glyphDriveData.driveOrder[0] = GlyphDriveDir.VERT;
      glyphDriveData.driveOrder[1] = GlyphDriveDir.HOR;
      glyphDriveData.targetOffsetH[0] = glyphDriveData.targetOffsetH[1] = glyphDriveData.targetOffsetH[2] = 10.5;
      glyphDriveData.targetOffsetV[2] = -3.5;
      glyphDriveData.targetOffsetV[1] = glyphDriveData.targetOffsetV[2] + 7.6;
      glyphDriveData.targetOffsetV[0] = glyphDriveData.targetOffsetV[1] + 7.6;
    } else if ((allianceColor == AllianceColor.BLUE) && (fieldMode == FieldMode.SQUARE)) {
      // fix these
      glyphDriveData.targetH = 24;
      glyphDriveData.targetV = -56.5;
      glyphDriveData.orientMoveH = -90;
      glyphDriveData.orientMoveV = 0;
      glyphDriveData.driveDirH = 1.0;
      glyphDriveData.driveDirV = 1.0;
      glyphDriveData.driveOrder[0] = GlyphDriveDir.VERT;
      glyphDriveData.driveOrder[1] = GlyphDriveDir.HOR;
      glyphDriveData.targetOffsetH[0] = 8.5;
      glyphDriveData.targetOffsetH[1] = glyphDriveData.targetOffsetH[0] + 7.6;
      glyphDriveData.targetOffsetH[2] = glyphDriveData.targetOffsetH[1] + 7.6;
      glyphDriveData.targetOffsetV[0] = glyphDriveData.targetOffsetV[1] = glyphDriveData.targetOffsetV[2] = 20;
    } else if ((allianceColor == AllianceColor.BLUE) && (fieldMode == FieldMode.STRAIGHT)) {
      glyphDriveData.targetH = 0;
      glyphDriveData.targetV = -32.5;
      glyphDriveData.orientMoveH = 90;
      glyphDriveData.orientMoveV = 0;
      glyphDriveData.driveDirH = -1.0;
      glyphDriveData.driveDirV = 1.0;
      glyphDriveData.driveOrder[0] = GlyphDriveDir.VERT;
      glyphDriveData.driveOrder[1] = GlyphDriveDir.HOR;
      glyphDriveData.targetOffsetH[0] = glyphDriveData.targetOffsetH[1] = glyphDriveData.targetOffsetH[2] = 10.5;
      glyphDriveData.targetOffsetV[0] = -11.75;
      glyphDriveData.targetOffsetV[1] = glyphDriveData.targetOffsetV[0] - 7.6;
      glyphDriveData.targetOffsetV[2] = glyphDriveData.targetOffsetV[1] - 7.6;
    }

    glyphDriveTarget = GlyphDriveTarget.IDLE;

    telemetry.addData("AutoStatus", "Initialized F %s A %s",
      this.fieldMode, this.allianceColor);
  }

  public void init_loop(Telemetry telemetry, Gamepad gamepad1) {
    updateDrive(telemetry, gamepad1);
    updateArm1(telemetry, gamepad1);
    updateArm2(telemetry, gamepad1);
    //updatedistanceRange(telemetry);
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
    if (autoState == AutoState.IDLE) {
      //  updatedistanceRange(telemetry);
      gyro.getZaxis(telemetry);
      vumark.scan(telemetry);
    }

    if (leverArmPosition != -1) {
      arm1.setPosition(leverArmPosition, telemetry);
    }

    telemetry.addData("AutoStatus", "runtime: %s F %s A %s C %d states auto %s jewel %s",
      runtime.toString(), fieldMode, allianceColor, selectGlyphColumn, autoState, jewelKnockOutState);
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
      arm1.raise(0, override, telemetry);
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

  public boolean runGlyphOrientMove(int index, Telemetry telemetry) {
    int desiredOrient;
    if (glyphDriveData.driveOrder[index] == GlyphDriveDir.HOR) {
      desiredOrient = glyphDriveData.orientMoveH;
    } else {
      desiredOrient = glyphDriveData.orientMoveV;
    }

    if (glyphDrive_Orient(desiredOrient, telemetry) == false) {
      return false;
    } else {
      double targetDelta;
      if (glyphDriveData.driveOrder[index] == GlyphDriveDir.HOR) {
        targetDelta = glyphDriveData.targetDeltaH;
      } else {
        targetDelta = glyphDriveData.targetDeltaV;
      }
      bbOffEncoderCnt = wheels.inchToEncCnt(targetDelta, true);
      return true;
    }
  }

  public boolean glyphDrive_Orient(int orientation, Telemetry telemetry) {
    int rotError = orientation - (-gyro.getZaxis(telemetry));
    if (rotError < -180) {
      rotError += 360;
    } else if (rotError > 179) {
      rotError -= 360;
    }
    double rotPower = 0.0;
    if (Math.abs(rotError) > 1) {
      rotPower = Range.scale((double)rotError, -180.0, 180.0, -1.0, 1.0);
      rotPower = Range.scale(Math.abs(rotPower), 0, 1, 0.2, 0.7) * Math.signum(rotPower);
      wheels.setScaling(false);
      wheels.drive(0, 0, rotPower, telemetry, distanceRange, true);
      return false;
    } else {
      wheels.stop();
      wheels.resetEncoder();
      return true;
    }
  }

  public boolean runGlyphDriveMove(int index, Telemetry telemetry) {
    double driveDir;
    int orient;
    if (glyphDriveData.driveOrder[index] == GlyphDriveDir.HOR) {
      driveDir = glyphDriveData.driveDirH;
      orient = glyphDriveData.orientMoveH;
    } else {
      driveDir = glyphDriveData.driveDirV;
      orient = glyphDriveData.orientMoveV;
    }

    if (glyphDrive_Move(orient, driveDir, telemetry) == false) {
      return false;
    } else {
      wheels.stop();
      wheels.resetEncoder();
      return true;
    }
  }


  private boolean glyphDrive_Move(int orientation, double driveDir, Telemetry telemetry) {
    int curEncCount = wheels.getEncoder() * (int)driveDir;
    int encoderDelta = bbOffEncoderCnt - curEncCount;

    boolean encoderStuckDetected  = false;
    abortedAtEncoderStuck = false;
    double encoderStuckDuration = 0.0;
    if (Math.abs(curEncCount) > 100) {
      if (Math.abs(lastEncoderCnt - curEncCount) < 5) {
        if (encoderStuck == false) {
          encoderStuckTimeStart = runtime.milliseconds();
          encoderStuck = true;
        } else {
          encoderStuckDuration = runtime.milliseconds() - encoderStuckTimeStart;
          if (encoderStuckDuration > 2000) {
            encoderStuckDetected = true;
            abortedAtEncoderStuck = true;
          }
        }
      } else {
        encoderStuck = false;
      }
    } else {
      encoderStuck = false;
    }
    lastEncoderCnt = curEncCount;
    if ((Math.abs(encoderDelta) > 3) && (encoderStuckDetected == false)) {
    //if (curEncCount < bbOffEncoderCnt) {
      double drivePowerRange = Math.max(bbOffEncoderCnt, 1500);
      double drivePower = Range.scale((double) encoderDelta, -drivePowerRange, drivePowerRange, -1.0, 1.0) * driveDir;
      drivePower = Range.scale(Math.abs(drivePower), 0, 1.0, 0.2, 0.4) * Math.signum(drivePower);

      double rotPower = 0.0;
      int rotError = orientation - (-gyro.getZaxis(telemetry));
      //if (Math.abs(encoderDelta) > 1500) {
        //rotPower = Range.scale((double) rotError, -180.0, 179.0, -1.0, 1.0);
        //rotPower = Range.scale(Math.abs(rotPower), 0, 1, 0.1, 0.3) * Math.signum(rotPower);
      //}

      wheels.setScaling(false);
      wheels.drive(drivePower, 0, rotPower, telemetry, distanceRange, true);

      telemetry.addData("glyphDrive", "encCnt cur %d last %d bbOffEncoderCnt %d rotError %d encoderStuck %b time %d",
        curEncCount, lastEncoderCnt, bbOffEncoderCnt, rotError, encoderStuck, (int)encoderStuckDuration);
      return false;
    } else {
      wheels.stop();
      wheels.resetEncoder();
      return true;
    }
  }
}