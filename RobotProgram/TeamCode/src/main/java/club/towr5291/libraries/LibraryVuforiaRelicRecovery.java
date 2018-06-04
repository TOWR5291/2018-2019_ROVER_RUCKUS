package club.towr5291.libraries;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.HINT;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.core.Point3;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by ianhaden on 1/6/18.
 */

public class LibraryVuforiaRelicRecovery {
    VuforiaLocalizer vuforia;
    VuforiaTrackables RelicRecovery;
    VuforiaTrackable relicTemplate;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    public VuforiaTrackables getRelicRecovery() {
        return this.RelicRecovery;
    }

    public VuforiaTrackable getRelicTemplate() {
        return this.relicTemplate;
    }

    public VuforiaLocalizer getVuforia() {
        return this.vuforia;
    }

    public List<VuforiaTrackable> getAllTrackables () {
        return this.allTrackables;
    }

    public VuforiaTrackables LibraryVuforiaRelicRecovery(HardwareMap hardwareMap, robotConfig robotConfiguration) {
        //load all the vuforia stuff

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        switch (robotConfiguration.getTeamNumber()) {
            case "5291":
                parameters.vuforiaLicenseKey = "AVATY7T/////AAAAGQJxfNYzLUgGjSx0aOEU0Q0rpcfZO2h2sY1MhUZUr+Bu6RgoUMUP/nERGmD87ybv1/lM2LBFDxcBGRHkXvxtkHel4XEUCsNHFTGWYcVkMIZqctQsIrTe13MnUvSOfQj8ig7xw3iULcwDpY+xAftW61dKTJ0IAOCxx2F0QjJWqRJBxrEUR/DfQi4LyrgnciNMXCiZ8KFyBdC63XMYkQj2joTN579+2u5f8aSCe8jkAFnBLcB1slyaU9lhnlTEMcFjwrLBcWoYIFAZluvFT0LpqZRlS1/XYf45QBSJztFKHIsj1rbCgotAE36novnAQBs74ewnWsJifokJGOYWdFJveWzn3GE9OEH23Y5l7kFDu4wc";
                break;
            case "11230":
                parameters.vuforiaLicenseKey = "Not Provided";
                break;
            case "11231":
                parameters.vuforiaLicenseKey = "Aai2GEX/////AAAAGaIIK9GK/E5ZsiRZ/jrJzdg7wYZCIFQ7uzKqQrMx/0Hh212zumzIy4raGwDY6Mf6jABMShH2etZC/BcjIowIHeAG5ShG5lvZIZEplTO+1zK1nFSiGFTPV59iGVqH8KjLbQdgUbsCBqp4f3tI8BWYqAS27wYIPfTK697SuxdQnpEZAOhHpgz+S2VoShgGr+EElzYMBFEaj6kdA/Lq5OwQp31JPet7NWYph6nN+TNHJAxnQBkthYmQg687WlRZhYrvNJepnoEwsDO3NSyeGlFquwuQwgdoGjzq2qn527I9tvM/XVZt7KR1KyWCn3PIS/LFvADSuyoQ2lsiOFtM9C+KCuNWiqQmj7dPPlpvVeUycoDH";
                break;
        }

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);                                          //enables RGB565 format for the image
        this.vuforia.setFrameQueueCapacity(5);                                                           //tells VuforiaLocalizer to only store one frame at a time
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 1);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        this.RelicRecovery = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        this.relicTemplate = this.RelicRecovery.get(0);
        this.relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        //List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(this.RelicRecovery);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        /**
         * We use units of mm here because that's the recommended units of measurement for the
         * size values specified in the XML for the ImageTarget trackables in data sets. E.g.:
         * You don't *have to* use mm here, but the units here and the units used in the XML
         * target configuration files *must* correspond for the math to work out correctly.
         */
        float mmPerInch = 25.4f;
        float mmBotWidth = 18 * mmPerInch;            // ... or whatever is right for your robot
        float FTCFieldWidth = (12 * 12 - 3);
        float mmFTCFieldWidth = FTCFieldWidth * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

        /**
         * In order for localization to work, we need to tell the system where each target we
         * wish to use for navigation resides on the field, and we need to specify where on the robot
         * the phone resides. These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * For the most part, you don't need to understand the details of the math of how transformation
         * matrices work inside (as fascinating as that is, truly). Just remember these key points:
         * <ol>
         *
         *     <li>You can put two transformations together to produce a third that combines the effect of
         *     both of them. If, for example, you have a rotation transform R and a translation transform T,
         *     then the combined transformation matrix RT which does the rotation first and then the translation
         *     is given by {@code RT = T.multiplied(R)}. That is, the transforms are multiplied in the
         *     <em>reverse</em> of the chronological order in which they applied.</li>
         *
         *     <li>A common way to create useful transforms is to use methods in the {@link OpenGLMatrix}
         *     class and the Orientation class. See, for example, {@link OpenGLMatrix#translation(float,
         *     float, float)}, {@link OpenGLMatrix#rotation(AngleUnit, float, float, float, float)}, and
         *     {@link Orientation#getRotationMatrix(AxesReference, AxesOrder, AngleUnit, float, float, float)}.
         *     Related methods in {@link OpenGLMatrix}, such as {@link OpenGLMatrix#rotated(AngleUnit,
         *     float, float, float, float)}, are syntactic shorthands for creating a new transform and
         *     then immediately multiplying the receiver by it, which can be convenient at times.</li>
         *
         *     <li>If you want to break open the black box of a transformation matrix to understand
         *     what it's doing inside, use {@link MatrixF#getTranslation()} to fetch how much the
         *     transform will move you in x, y, and z, and use {@link Orientation#getOrientation(MatrixF,
         *     AxesReference, AxesOrder, AngleUnit)} to determine the rotational motion that the transform
         *     will impart. See {@link #format(OpenGLMatrix)} below for an example.</li>
         *
         * </ol>
         *
         *
         * Initially the target is conceptually lying at the origin of the field's coordinate system
         * (the center of the field), facing up.
         *
         * In this configuration, the target's coordinate system aligns with that of the field.
         *
         * In a real situation we'd also account for the vertical (Z) offset of the target,
         * but for simplicity, we ignore that here; for a real robot, you'll want to fix that.
         *
         * To place the Wheels Target on the Red Audience wall:
         * - First we rotate it 90 around the field's X axis to flip it upright
         * - Then we rotate it  90 around the field's Z access to face it away from the audience.
         * - Finally, we translate it back along the X axis towards the red audience wall.
         *
         */

        // Target Positions
        // position is approximately - (32inch, -70.5inch)
        Point3 Red1Coord = new Point3(32, -FTCFieldWidth / 2, 0);
        Point3 Red1Angle = new Point3(-90, 0, 0);
        Point3 Red1PhoneCoord = new Point3(0, 0, 200);
        Point3 Red1PhoneAngle = new Point3(-90, 0, 90);

        Point3 Red2Coord = new Point3(-44.5, -FTCFieldWidth / 2, 0);
        Point3 Red2Angle = new Point3(-90, 0, 0);
        Point3 Red2PhoneCoord = new Point3(0, 0, 200);
        Point3 Red2PhoneAngle = new Point3(-90, 0, 90);

        Point3 Blue1Coord = new Point3(15.25, FTCFieldWidth / 2, 0);
        Point3 Blue1Angle = new Point3(90, 0, 0);
        Point3 Blue1PhoneCoord = new Point3(0, 0, 200);
        Point3 Blue1PhoneAngle = new Point3(-90, 0, -90);

        Point3 Blue2Coord = new Point3(-56, FTCFieldWidth / 2, 0);
        Point3 Blue2Angle = new Point3(90, 0, 0);
        Point3 Blue2PhoneCoord = new Point3(0, 0, 200);
        Point3 Blue2PhoneAngle = new Point3(-90, 0, -90);

        Point3 targetPoint = new Point3(0, 0, 0);
        Point3 targetAngle = new Point3(0, 0, 0);
        Point3 phonePoint = new Point3(0, 0, 200);
        Point3 phoneAngle = new Point3(-90, 0, 0);

        if ((robotConfiguration.getAllianceColor().equalsIgnoreCase("red")) && (robotConfiguration.getAllianceStartPosition().equalsIgnoreCase("left"))) {
            targetPoint = Red1Coord;
            targetAngle = Red1Angle;
            phonePoint = Red1PhoneCoord;
            phoneAngle = Red1PhoneAngle;
        } else if ((robotConfiguration.getAllianceColor().equalsIgnoreCase("red")) && (robotConfiguration.getAllianceStartPosition().equalsIgnoreCase("right"))) {
            targetPoint = Red2Coord;
            targetAngle = Red2Angle;
            phonePoint = Red2PhoneCoord;
            phoneAngle = Red2PhoneAngle;
        } else if ((robotConfiguration.getAllianceColor().equalsIgnoreCase("blue")) && (robotConfiguration.getAllianceStartPosition().equalsIgnoreCase("left"))) {
            targetPoint = Blue1Coord;
            targetAngle = Blue1Angle;
            phonePoint = Blue1PhoneCoord;
            phoneAngle = Blue1PhoneAngle;
        } else if ((robotConfiguration.getAllianceColor().equalsIgnoreCase("blue")) && (robotConfiguration.getAllianceStartPosition().equalsIgnoreCase("right"))) {
            targetPoint = Blue2Coord;
            targetAngle = Blue2Angle;
            phonePoint = Blue2PhoneCoord;
            phoneAngle = Blue2PhoneAngle;
        }

        OpenGLMatrix relicVuMarkTemplateLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(mmPerInch * (float) targetPoint.x, mmPerInch * (float) targetPoint.y, mmPerInch * (float) targetPoint.z)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, (float) targetAngle.x, (float) targetAngle.y, (float) targetAngle.z));

        this.relicTemplate.setLocation(relicVuMarkTemplateLocationOnField);

        /**
         * Create a transformation matrix describing where the phone is on the robot. Here, we
         * put the phone on the right hand side of the robot with the screen facing in (see our
         * choice of BACK camera above) and in landscape mode. Starting from alignment between the
         * robot's and phone's axes, this is a rotation of -90deg along the Y axis.
         *
         * When determining whether a rotation is positive or negative, consider yourself as looking
         * down the (positive) axis of rotation from the positive towards the origin. Positive rotations
         * are then CCW, and negative rotations CW. An example: consider looking down the positive Z
         * axis towards the origin. A positive rotation about Z (ie: a rotation parallel to the the X-Y
         * plane) is then CCW, as one would normally expect from the usual classic 2D geometry.
         */
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation((mmBotWidth / 2) * (float) phonePoint.x, (mmBotWidth / 2) * (float) phonePoint.x, (mmBotWidth / 2) * (float) phonePoint.x)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, (float) phoneAngle.x, (float) phoneAngle.y, (float) phoneAngle.z));

        /**
         * Let the trackable listeners we care about know where the phone is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */
        ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        /**
         * A brief tutorial: here's how all the math is going to work:
         *
         * C = phoneLocationOnRobot  maps   phone coords -> robot coords
         * P = tracker.getPose()     maps   image target coords -> phone coords
         * L = redTargetLocationOnField maps   image target coords -> field coords
         *
         * So
         *
         * C.inverted()              maps   robot coords -> phone coords
         * P.inverted()              maps   phone coords -> imageTarget coords
         *
         * Putting that all together,
         *
         * L x P.inverted() x C.inverted() maps robot coords to field coords.
         *
         * @see VuforiaTrackableDefaultListener#getRobotLocation()
         */

        return RelicRecovery;
    }


}
