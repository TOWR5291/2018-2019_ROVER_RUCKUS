package club.towr5291.Concepts;

import android.content.SharedPreferences;
import android.graphics.Bitmap;
import android.hardware.Camera;
import android.os.Environment;
import android.preference.PreferenceManager;
import android.util.Log;
import android.widget.ImageView;
import android.widget.TextView;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraCalibration;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec2F;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
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

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgcodecs.Imgcodecs; // imread, imwrite, etc
import org.opencv.imgproc.Moments;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;

import club.towr5291.functions.BeaconAnalysisOCVPlayground;
import club.towr5291.functions.Constants;
import club.towr5291.functions.BeaconAnalysisOCV;
import club.towr5291.functions.BeaconAnalysisOCV2;
import club.towr5291.functions.BeaconAnalysisOCVAnalyse;
import club.towr5291.functions.FileLogger;
import club.towr5291.R;
import club.towr5291.functions.JewelAnalysisOCV;
import club.towr5291.opmodes.OpModeMasterLinear;
import hallib.HalDashboard;


import static org.opencv.imgproc.Imgproc.contourArea;


/**
 * Created by ianhaden on 4/10/2016.
 */

@Autonomous(name="Concept Vuforia Grab Image", group="5291Test")
public class ConceptVuforiaOpGrabImage extends OpModeMasterLinear
{
    //set up TAG for logging prefic, this info will appear first in every log statemend
    private static final String TAG = "AutoDriveTeam5291";

    OpenGLMatrix lastLocation = null;
    private double robotX;
    private double robotY;
    private double robotZ;
    private double robotBearing;

    //set up the variables for the logger
    private String startDate;
    private ElapsedTime runtime = new ElapsedTime();
    private FileLogger fileLogger;
    public int debug;

    //The autonomous menu settings from the sharepreferences
    private SharedPreferences sharedPreferences;
    private String teamNumber;
    private String allianceColor;
    private String allianceStartPosition;
    private String allianceParkPosition;
    private int delay;
    private String numBeacons;
    private String robotConfig;

    private static final int TARGET_WIDTH = 254;
    private static final int TARGET_HEIGHT = 184;


    private static HalDashboard dashboard = null;
    public static HalDashboard getDashboard()
    {
        return dashboard;
    }


    @Override
    public void runOpMode() throws InterruptedException {

        dashboard = HalDashboard.createInstance(telemetry);

        FtcRobotControllerActivity act = (FtcRobotControllerActivity)(hardwareMap.appContext);

        dashboard.setTextView((TextView)act.findViewById(R.id.textOpMode));

        dashboard.clearDisplay();
        dashboard.displayPrintf(0, "Starting Menu System");

        dashboard.displayPrintf(0, 200, "Text: ", "*** Robot Data ***");
        //start the logging

        //create logging based on initial settings, sharepreferences will adjust levels
        fileLogger = new FileLogger(runtime,1,true);
        fileLogger.write("Time,SysMS,Thread,Event,Desc");
        fileLogger.writeEvent(TAG, "Log Started");
        runtime.reset();

        //init openCV
        initOpenCv();
        dashboard.displayPrintf(1, "initRobot OpenCV!");

        if (debug >= 3) {fileLogger.writeEvent(TAG, "OpenCV Started");}

        //load menu settings and setup robot and debug level
        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        teamNumber = sharedPreferences.getString("club.towr5291.Autonomous.TeamNumber", "0000");
        allianceColor = sharedPreferences.getString("club.towr5291.Autonomous.Color", "Red");
        allianceStartPosition = sharedPreferences.getString("club.towr5291.Autonomous.StartPosition", "Left");
        delay = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Delay", "0"));
        robotConfig = sharedPreferences.getString("club.towr5291.Autonomous.RobotConfig", "TileRunnerMecanum2x40");
        debug = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", "1"));

        dashboard.displayPrintf(2, "robotConfigTeam # " + teamNumber);
        dashboard.displayPrintf(3, "Alliance          " + allianceColor);
        dashboard.displayPrintf(4, "Start Pos         " + allianceStartPosition);
        dashboard.displayPrintf(5, "Start Del         " + delay);
        dashboard.displayPrintf(6, "Robot             " + robotConfig);
        dashboard.displayPrintf(7, "Debug             " + debug);

        fileLogger.setDebugLevel(debug);

        JewelAnalysisOCV JewelColour = new JewelAnalysisOCV();

        //load all the vuforia stuff

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        switch (teamNumber) {
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

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);                                          //enables RGB565 format for the image
        vuforia.setFrameQueueCapacity(5);                                                           //tells VuforiaLocalizer to only store one frame at a time
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS,1);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables RelicRecovery = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = RelicRecovery.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(RelicRecovery);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        /**
         * We use units of mm here because that's the recommended units of measurement for the
         * size values specified in the XML for the ImageTarget trackables in data sets. E.g.:
         * You don't *have to* use mm here, but the units here and the units used in the XML
         * target configuration files *must* correspond for the math to work out correctly.
         */
        float mmPerInch        = 25.4f;
        float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
        float FTCFieldWidth    = (12*12 - 3);
        float mmFTCFieldWidth  = FTCFieldWidth * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

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
        Point3 Red1Coord        = new Point3 (32, -FTCFieldWidth/2, 0);
        Point3 Red1Angle        = new Point3 (-90, 0, 0);
        Point3 Red1PhoneCoord   = new Point3 (0,0,200);
        Point3 Red1PhoneAngle   = new Point3 (-90,0,90);

        Point3 Red2Coord        = new Point3 (-44.5, -FTCFieldWidth/2, 0);
        Point3 Red2Angle        = new Point3 (-90, 0, 0);
        Point3 Red2PhoneCoord   = new Point3 (0,0,200);
        Point3 Red2PhoneAngle   = new Point3 (-90,0,90);

        Point3 Blue1Coord       = new Point3 (15.25, FTCFieldWidth/2, 0);
        Point3 Blue1Angle       = new Point3 (90, 0, 0);
        Point3 Blue1PhoneCoord  = new Point3 (0,0,200);
        Point3 Blue1PhoneAngle  = new Point3 (-90,0,-90);

        Point3 Blue2Coord       = new Point3 (-56, FTCFieldWidth/2, 0);
        Point3 Blue2Angle       = new Point3 (90, 0, 0);
        Point3 Blue2PhoneCoord  = new Point3 (0,0,200);
        Point3 Blue2PhoneAngle  = new Point3 (-90,0,-90);

        Point3 targetPoint      = new Point3 (0,0,0);
        Point3 targetAngle      = new Point3 (0,0,0);
        Point3 phonePoint       = new Point3 (0,0,200);
        Point3 phoneAngle       = new Point3 (-90,0,0);

        if ((allianceColor == "Red") && (allianceStartPosition == "Left")) {
            targetPoint = Red1Coord;
            targetAngle = Red1Angle;
            phonePoint = Red1PhoneCoord;
            phoneAngle = Red1PhoneAngle;
        } else if ((allianceColor == "Red") && (allianceStartPosition == "Right")) {
            targetPoint = Red2Coord;
            targetAngle = Red2Angle;
            phonePoint = Red2PhoneCoord;
            phoneAngle = Red2PhoneAngle;
        } else if ((allianceColor == "Blue") && (allianceStartPosition == "Left")) {
            targetPoint = Blue1Coord;
            targetAngle = Blue1Angle;
            phonePoint = Blue1PhoneCoord;
            phoneAngle = Blue1PhoneAngle;
        } else if ((allianceColor == "Blue") && (allianceStartPosition == "Right")) {
            targetPoint = Blue2Coord;
            targetAngle = Blue2Angle;
            phonePoint = Blue2PhoneCoord;
            phoneAngle = Blue2PhoneAngle;
        }

        OpenGLMatrix relicVuMarkTemplateLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(mmPerInch * (float)targetPoint.x, mmPerInch * (float)targetPoint.y, mmPerInch * (float)targetPoint.z)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, (float)targetAngle.x, (float)targetAngle.y, (float)targetAngle.z));

        relicTemplate.setLocation(relicVuMarkTemplateLocationOnField);

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
                .translation((mmBotWidth/2) * (float)phonePoint.x, (mmBotWidth/2) * (float)phonePoint.x, (mmBotWidth/2) * (float)phonePoint.x)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, (float)phoneAngle.x, (float)phoneAngle.y, (float)phoneAngle.z));

        /**
         * Let the trackable listeners we care about know where the phone is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */
        ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

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
        //activate vuforia
        RelicRecovery.activate();

        //set up variable for our capturedimage
        Image rgb = null;

        dashboard.displayPrintf(1, "initRobot VUFORIA Loaded");

        waitForStart();

        int loop = 0;
        Constants.ObjectColours Colour = Constants.ObjectColours.UNKNOWN;
        //try {
        //    Camera camera;
        //    Camera.Parameters cameraparams;
        //    camera = Camera.open();
        //    cameraparams = camera.getParameters();
        //    cameraparams = camera.getParameters();
        //    cameraparams.setFlashMode(Camera.Parameters.FLASH_MODE_TORCH);
        //    camera.setParameters(cameraparams);
        //} catch (RuntimeException e) {
        //    Log.e("Camera Error Open", e.getMessage());
        //}
        while (opModeIsActive()) {

            for (VuforiaTrackable jewel : RelicRecovery) {

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) jewel.getListener()).getRawPose();

                if (pose != null) {

                    Matrix34F rawPose = new Matrix34F();
                    float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
                    rawPose.setData(poseData);

                    Vec2F upperLeft  = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(-TARGET_WIDTH / 2,  TARGET_HEIGHT / 2, 0));
                    Vec2F upperRight = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F( TARGET_WIDTH / 2,  TARGET_HEIGHT / 2, 0));
                    Vec2F lowerRight = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F( TARGET_WIDTH / 2, -TARGET_HEIGHT / 2, 0));
                    Vec2F lowerLeft  = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(-TARGET_WIDTH / 2, -TARGET_HEIGHT / 2, 0));

                    double dblMidPointTopx = (upperRight.getData()[0] + upperLeft.getData()[0]) / 2;
                    double dblMidPointTopy = (upperRight.getData()[1] + upperLeft.getData()[1]) / 2;
                    double dblMidPointBotx = (lowerRight.getData()[0] + lowerLeft.getData()[0]) / 2;
                    double dblMidPointBoty = (lowerRight.getData()[1] + lowerLeft.getData()[1]) / 2;

                    double width  = Math.sqrt((Math.pow((upperRight.getData()[1] - upperLeft.getData()[1]),2)) + (Math.pow((upperRight.getData()[0] - upperLeft.getData()[0]),2)));
                    double height = Math.sqrt((Math.pow((dblMidPointTopy - dblMidPointBoty),2)) + (Math.pow((dblMidPointTopx - dblMidPointBotx),2)));

                    //width is equal to 254 mm, so width of beacon is 220mm, height of beacon is 150mm
                    //pixels per mm width, using a known size of the target
                    double dblWidthPixelsPermm = width / TARGET_WIDTH;
                    double dblHeightPixelsPermm = height / TARGET_HEIGHT;

                    if (debug >= 1)
                    {
                        try {
                            fileLogger.writeEvent("Vuforia", "upperLeft 0 "  + upperLeft.getData()[0]);
                            fileLogger.writeEvent("Vuforia", "upperLeft 1 "  + upperLeft.getData()[1]);

                            fileLogger.writeEvent("Vuforia", "upperRight 0 "  + upperRight.getData()[0]);
                            fileLogger.writeEvent("Vuforia", "upperRight 1 "  + upperRight.getData()[1]);

                            fileLogger.writeEvent("Vuforia", "lowerLeft 0 "  + lowerLeft.getData()[0]);
                            fileLogger.writeEvent("Vuforia", "lowerLeft 1 "  + lowerLeft.getData()[1]);

                            fileLogger.writeEvent("Vuforia", "lowerRight 0 "  + lowerRight.getData()[0]);
                            fileLogger.writeEvent("Vuforia", "lowerRight 1 "  + lowerRight.getData()[1]);

                            fileLogger.writeEvent("Vuforia", "dblMidPointTopx "  + dblMidPointTopx);
                            fileLogger.writeEvent("Vuforia", "dblMidPointTopy "  + dblMidPointTopy);
                            fileLogger.writeEvent("Vuforia", "dblMidPointBotx "  + dblMidPointBotx);
                            fileLogger.writeEvent("Vuforia", "dblMidPointBoty "  + dblMidPointBoty);

                            fileLogger.writeEvent("Vuforia", "width in pixels "  + width);
                            fileLogger.writeEvent("Vuforia", "height in pixels "  + height);
                        }
                            catch (Exception e) {
                                //fileLogger.writeEvent("Vuforia", "OOOPS" );
                        }
                    }
                }
            }

            VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take(); //takes the frame at the head of the queue

            long numImages = frame.getNumImages();

            for (int i = 0; i < numImages; i++) {
                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                    rgb = frame.getImage(i);
                    break;
                }
            }

            /*rgb is now the Image object that we’ve used in the video*/
            Log.d("OPENCV", "Height " + rgb.getHeight() + " Width " + rgb.getWidth());

            Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(rgb.getPixels());
            Mat tmp = new Mat(rgb.getWidth(), rgb.getHeight(), CvType.CV_8UC4);
            Utils.bitmapToMat(bm, tmp);

            frame.close();
            Log.d("fl", "Debug Level Before" + fileLogger.getDebugLevel() );
            Colour = JewelColour.JewelAnalysisOCV(fileLogger, tmp, loop);
            loop++;

            for (VuforiaTrackable trackable : allTrackables) {
                /**
                 * getUpdatedRobotLocation() will return null if no new information is available since
                 * the last time that call was made, or if the trackable is not currently visible.
                 * getRobotLocation() will return null if the trackable is not currently visible.
                 */
                //telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;

                }
            }
            /**
             * Provide feedback as to where the robot was last located (if we know).
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (lastLocation != null) {
                // Then you can extract the positions and angles using the getTranslation and getOrientation methods.
                VectorF trans = lastLocation.getTranslation();
                Orientation rot = Orientation.getOrientation(lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                // Robot position is defined by the standard Matrix translation (x and y)
                robotX = trans.get(0);
                robotY = trans.get(1);
                robotZ = trans.get(2);

                // Robot bearing (in Cartesian system) position is defined by the standard Matrix z rotation
                robotBearing = rot.thirdAngle;
                if (robotBearing < 0)
                {
                    robotBearing = 360 + robotBearing;
                }

                dashboard.displayCenterPrintf(1,200, "*** Vision Data***");
                dashboard.displayPrintf(2, "Pos X " + robotX);
                dashboard.displayPrintf(3, "Pos Z " + robotY);
                dashboard.displayPrintf(4, "Pos Z " + robotZ);
                dashboard.displayPrintf(5, "Pos B " + robotBearing);
                dashboard.displayPrintf(6, "Pos - " + format(lastLocation));
                dashboard.displayCenterPrintf(7,200, vuMark.toString());

            } else {
                dashboard.displayCenterPrintf(1,200,"*** UNKNOWN***");
            }

            switch (Colour) {
                case OBJECT_BLUE:
                    dashboard.displayPrintf(9, "Colour Blue");
                    break;
                case OBJECT_RED:
                    dashboard.displayPrintf(9, "Colour Red");
                    break;
                case OBJECT_BLUE_RED:
                    dashboard.displayPrintf(9, "Colour Blue Red");
                    break;
                case OBJECT_RED_BLUE:
                    dashboard.displayPrintf(9, "Colour Red Blue");
                    break;
                case OBJECT_BLUE_LEFT:
                    dashboard.displayPrintf(9, "Colour Blue XXXX");
                    break;
                case OBJECT_RED_LEFT:
                    dashboard.displayPrintf(9, "Colour Red XXXX");
                    break;
                case OBJECT_BLUE_RIGHT:
                    dashboard.displayPrintf(9, "Colour XXXX Blue");
                    break;
                case OBJECT_RED_RIGHT:
                    dashboard.displayPrintf(9, "Colour XXXX Red");
                    break;
                case UNKNOWN:
                    dashboard.displayPrintf(9, "Colour Unknown");
                    break;
            }
        }

        //stop the log
        if (fileLogger != null)
        {
            fileLogger.writeEvent("stop()","Stopped");
            fileLogger.close();
            fileLogger = null;
        }

    }

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

}
