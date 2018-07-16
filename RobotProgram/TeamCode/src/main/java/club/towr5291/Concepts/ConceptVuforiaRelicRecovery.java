package club.towr5291.Concepts;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import club.towr5291.libraries.LibraryVuforiaRelicRecovery;
import club.towr5291.libraries.robotConfig;
import club.towr5291.opmodes.OpModeMasterLinear;


/**
 * Created by ianhaden on 1/6/18.
 */
@Autonomous(name="Concept Vuforia Relic Recovery", group="Cocept")
@Disabled
public class ConceptVuforiaRelicRecovery  extends OpModeMasterLinear {


    private robotConfig ourRobotConfig;

    @Override
    public void runOpMode() throws InterruptedException {
        ourRobotConfig.setAllianceColor("Red");
        ourRobotConfig.setTeamNumber("5291");
        ourRobotConfig.setAllianceStartPosition("Left");

        LibraryVuforiaRelicRecovery test = new LibraryVuforiaRelicRecovery();
        VuforiaTrackables RelicRecovery;
        RelicRecovery = test.LibraryVuforiaRelicRecovery(hardwareMap, ourRobotConfig);

        //activate vuforia
        RelicRecovery.activate();

        //vuMark will be the position to load the glyph
        RelicRecoveryVuMark vuMark;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //the main loop.  this is where the action happens
        while (opModeIsActive()) {

            vuMark = RelicRecoveryVuMark.from(test.getRelicTemplate());
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)test.getRelicTemplate().getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                    telemetry.addLine("Location2 x=" + tX + ", y=" + tY +", z=" + tZ);
                }
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
