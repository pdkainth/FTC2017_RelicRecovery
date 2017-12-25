package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class VuMark_Nav {
  //public static final String TAG = "Vuforia VuMark for FTC";
  private OpenGLMatrix lastLocation = null;
 
  /**
   * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
   * localization engine.
   */
  private VuforiaLocalizer vuforia;
  private int cameraMonitorViewId;
  private VuforiaLocalizer.Parameters parameters;
  private VuforiaTrackables relicTrackables;
  private VuforiaTrackable relicTemplate;

  private RelicRecoveryVuMark vuMark;
  private OpenGLMatrix pose;
  private VectorF translation;
  private Orientation orientation;

  public void init(HardwareMap hardwareMap) {
    cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
    parameters.vuforiaLicenseKey = "AWjQG/f/////AAAAGaR1lGJIDkdysS4tDu7sXQYDMWeiLDd9SoUJThu+KZdTtUifS+QbDu1xvPaSQTLtSRQuFNS82vbCdNnJZ9menqK5EWoKuH3N5BRZP14ZkIX71FS7Y1al/IzK+TEpILyoz3xoi2vPoiO1eHJpApOglq7sFPzQjrOu/12lHMI62JwzqRxuM55x++q0jgw/B3nP4duClSl4GenRihJpLA1ons/GHwtGHl3/M0cGgmQS/yYq6r/gpaNp4KCM9AdmyJ0Lstn85xnTek3EmxiLBWQ0WM16CB1zpXdo1oNlz/o8/9pZMTpTuz8zAjDkmL5W7xlGY/mT3bmaajDDWR/VZQOaRC7hL2Ic+UK4Hl97H2KCrLaW";
    parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
    this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
    relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
    relicTemplate = relicTrackables.get(0);
    relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
  }

  public void start() {
    relicTrackables.activate();
  }

  public void scan(Telemetry telemetry) {
    vuMark = RelicRecoveryVuMark.from(relicTemplate);
    if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
      /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
       * it is perhaps unlikely that you will actually need to act on this pose information, but
       * we illustrate it nevertheless, for completeness. */
      pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
      telemetry.addData("Pose", format(pose));

      /* We further illustrate how to decompose the pose into useful rotational and
       * translational components */
      if (pose != null) {
        translation = pose.getTranslation();
        orientation = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        // Extract the X, Y, and Z components of the offset of the target relative to the robot
        double tX = translation.get(0);
        double tY = translation.get(1);
        double tZ = translation.get(2);

        // Extract the rotational components of the target relative to the robot
        double rX = orientation.firstAngle;
        double rY = orientation.secondAngle;
        double rZ = orientation.thirdAngle;

        telemetry.addData("VuMark", "%s trans %.2f %.2f %.2f rot %.2f %.2f %.2f", vuMark,
                          tX, tY, tZ, rX, rY, rZ);
      }
      else {
        telemetry.addData("VuMark", "%s", vuMark);
      }
    } else {
      telemetry.addData("VuMark", "not visible");
    }
  } 

  public void stop() {
  }

  String format(OpenGLMatrix transformationMatrix) {
      return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
  }

  public RelicRecoveryVuMark getCurVumark() { return vuMark; }
  public OpenGLMatrix getCurPose() { return pose; }
  public VectorF getVumarkTrans() { return translation; }
  public Orientation getVumarkOrient() { return orientation; }
}
