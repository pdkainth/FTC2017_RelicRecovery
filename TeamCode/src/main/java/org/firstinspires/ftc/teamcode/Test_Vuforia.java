package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@TeleOp(name = "Test VuMark", group = "TEST_OP_MODE")
public class Test_Vuforia extends OpMode {
  private enum JewelKnockState { IDLE, START_FORWARD, WAIT_FORWARD, START_BACKWARD, WAIT_BACKWARD }

  private ElapsedTime runtime = new ElapsedTime();
  private Mecanum wheels = new Mecanum();
  private MyRangeSensor distanceRange = new MyRangeSensor();
  private VuMark_Nav vumark = new VuMark_Nav();
  private JewelArm jewelArm = new JewelArm();
  private MyColorSensor colorSensor = new MyColorSensor();
  private MyGyro gyro = new MyGyro();
  private GrabberArm grabberArm = new GrabberArm();

  private boolean gotVuMark = false;
  private boolean gotToTarget = false;
  private boolean colorIsFound = false;

  private double rotPowerDir = 1.0;
  private JewelKnockState jewelKnockState = JewelKnockState.IDLE;
  private double jewelRotStartTime = 0.0;

  @Override
  public void init() {
    telemetry.addData("Status", "Initializing");

    wheels.init(hardwareMap);
    distanceRange.init(hardwareMap);
    vumark.init(hardwareMap);
    jewelArm.init(hardwareMap);
    colorSensor.init(hardwareMap, "ColorSensor1");
    gyro.init(hardwareMap);
    grabberArm.init(hardwareMap);

    gotVuMark = false;
    gotToTarget = false;
    colorIsFound = false;
    rotPowerDir = 1.0;
    jewelKnockState = JewelKnockState.IDLE;

    telemetry.addData("Status", "Initialized");
  }

  @Override
  public void init_loop() {
    gyro.init_loop(telemetry);
  }

  @Override
  public void start() {
    vumark.start();
    runtime.reset();
  }

  @Override
  public void loop() {
    //updateDrive();
    updatedistanceRange();
    vumark.scan(telemetry);
    if (gotVuMark == false) {
      RelicRecoveryVuMark curVumark =  vumark.getCurVumark();
      OpenGLMatrix curPose = vumark.getCurPose();
      if ((curVumark == RelicRecoveryVuMark.UNKNOWN) || (curPose == null)) {
        wheels.drive(0.5, 0, 0, telemetry, distanceRange, true);
        wheels.stop();
      } else {
        gotVuMark = true;
      }
    } else {
      if (gotToTarget == false) {
        VectorF trans = vumark.getVumarkTrans();
        Orientation orientation = vumark.getVumarkOrient();
        double zError = -449 - trans.get(2);
        double xError = trans.get(0) - 160;
        double yRotError = 0 - orientation.secondAngle;

        if ((Math.abs(zError) < 5) && (Math.abs(xError) < 5) && (yRotError < 1)) {
          gotToTarget = true;
          gyro.resetZaxis();
          jewelArm.down();
        } else {
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
          wheels.setScaling(false);
          wheels.drive(drivePower, strafePower, rotPower, telemetry, distanceRange, true);
          wheels.stop();
        }
      } else {
        if (colorIsFound == false) {
          int colorIdx = colorSensor.getColor(telemetry);
          if (colorIdx == 0) {
            wheels.drive(0.5, 0, 0, telemetry, distanceRange, true);
            wheels.stop();
          } else {
            colorIsFound = true;
            if (colorIdx == 10) {
              rotPowerDir = 1.0;
            } else {
              rotPowerDir = -1.0;
            }
            jewelKnockState = JewelKnockState.START_FORWARD;
            wheels.setScaling(true);
          }
        } else {
          if (jewelKnockState == JewelKnockState.START_FORWARD) {
            wheels.drive(0, 0, 0.5 * rotPowerDir, telemetry, distanceRange, true);
            jewelKnockState = JewelKnockState.WAIT_FORWARD;
            jewelRotStartTime = runtime.milliseconds();
          } else if (jewelKnockState == JewelKnockState.WAIT_FORWARD) {
            double curTime = runtime.milliseconds();
            if ((curTime - jewelRotStartTime) > 1000) {
              wheels.stop();
              jewelKnockState = JewelKnockState.START_BACKWARD;
            }
          } else if (jewelKnockState == JewelKnockState.START_BACKWARD) {
            jewelArm.up();
            wheels.drive(0, 0, -0.5 * rotPowerDir, telemetry, distanceRange, true);
            jewelKnockState = JewelKnockState.WAIT_BACKWARD;
            jewelRotStartTime = runtime.milliseconds();
          } else if (jewelKnockState == JewelKnockState.WAIT_BACKWARD) {
            double curTime = runtime.milliseconds();
            if ((curTime - jewelRotStartTime) > 1000) {
              wheels.stop();
              jewelKnockState = JewelKnockState.IDLE;
            }
          }
        }
      }
    }

    telemetry.addData("Status", "Run Time: " + runtime.toString());
  }

  @Override
  public void stop() {
    wheels.stop();
    jewelArm.stop();
    colorSensor.stop();
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
    telemetry.addData("gamepad1 wheels", "D(%.2f) S(%.2f) R(%.2f)", drive, strafe, rotate);
  }

  private void updatedistanceRange() {
    distanceRange.getDistance(telemetry);
  }
}
