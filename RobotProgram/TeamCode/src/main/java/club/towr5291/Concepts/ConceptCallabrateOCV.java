package club.towr5291.Concepts;

import android.content.SharedPreferences;
import android.graphics.Bitmap;
import android.preference.PreferenceManager;
import android.util.Log;
import android.widget.TextView;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
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
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import club.towr5291.R;
import club.towr5291.functions.Constants;
import club.towr5291.functions.FileLogger;
import club.towr5291.functions.RoverRuckusOCV;
import club.towr5291.libraries.TOWRDashBoard;
import club.towr5291.opmodes.OpModeMasterLinear;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

/**
 * Created by ianhaden on 4/10/2016.
 */

@TeleOp(name="Concept OCV Calibrate", group="5291Test")
//@Disabled
public class ConceptCallabrateOCV extends OpModeMasterLinear
{
    //set up TAG for logging prefic, this info will appear first in every log statemend
    private static final String TAG = "AutoDriveTeam5291";

    OpenGLMatrix lastLocation = null;
    private double robotX;
    private double robotY;
    private double robotZ;
    private double robotBearing;

    VuforiaTrackable blueRover, redFootPrint, frontCraters, backSpace;

    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor


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
    private String RobotConfigBase;

    private static final int TARGET_WIDTH = 254;
    private static final int TARGET_HEIGHT = 184;


    private static TOWRDashBoard dashboard = null;
    public static TOWRDashBoard getDashboard()
    {
        return dashboard;
    }
    private Gamepad game1;

    @Override
    public void runOpMode() throws InterruptedException {
        game1 = gamepad1;

        boolean calibrate = false;

        dashboard = TOWRDashBoard.createInstance(telemetry);

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
        RobotConfigBase = sharedPreferences.getString("club.towr5291.Autonomous.RobotConfigBase", "TileRunnerMecanum2x40");
        debug = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", "1"));

        dashboard.displayPrintf(2, "robotConfigTeam # " + teamNumber);
        dashboard.displayPrintf(3, "Alliance          " + allianceColor);
        dashboard.displayPrintf(4, "Start Pos         " + allianceStartPosition);
        dashboard.displayPrintf(5, "Start Del         " + delay);
        dashboard.displayPrintf(6, "Robot             " + RobotConfigBase);
        dashboard.displayPrintf(7, "Debug             " + debug);
        dashboard.displayPrintf(8, "Press A to Calibrate");

        fileLogger.setDebugLevel(debug);

        RoverRuckusOCV elementColour = new RoverRuckusOCV();

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

        VuforiaTrackables roverTrackables = vuforia.loadTrackablesFromAsset("RoverRuckus");
        blueRover = roverTrackables.get(0);
        blueRover.setName("BlueRover");
        redFootPrint = roverTrackables.get(1);
        redFootPrint.setName("RedFootprint");
        frontCraters = roverTrackables.get(2);
        frontCraters.setName("FrontCraters");
        backSpace = roverTrackables.get(3);
        backSpace.setName("BackSpace");

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        roverTrackables.activate();

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> trackables = new ArrayList<VuforiaTrackable>();
        trackables.addAll(roverTrackables);

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootPrint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        final int CAMERA_FORWARD_DISPLACEMENT  = 0;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot;

        phoneLocationOnRobot = OpenGLMatrix
                 .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                 .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        -90, 0, 0));

        //set up variable for our capturedimage
        Image rgb = null;

        dashboard.displayPrintf(1, "initRobot VUFORIA Loaded");

        waitForStart();

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
        for (VuforiaTrackable trackable : roverTrackables) {

            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) trackable.getListener()).getRawPose();

            if (pose != null) {

                Matrix34F rawPose = new Matrix34F();
                float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
                rawPose.setData(poseData);

                Vec2F upperLeft = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(-TARGET_WIDTH / 2, TARGET_HEIGHT / 2, 0));
                Vec2F upperRight = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(TARGET_WIDTH / 2, TARGET_HEIGHT / 2, 0));
                Vec2F lowerRight = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(TARGET_WIDTH / 2, -TARGET_HEIGHT / 2, 0));
                Vec2F lowerLeft = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(-TARGET_WIDTH / 2, -TARGET_HEIGHT / 2, 0));

                double dblMidPointTopx = (upperRight.getData()[0] + upperLeft.getData()[0]) / 2;
                double dblMidPointTopy = (upperRight.getData()[1] + upperLeft.getData()[1]) / 2;
                double dblMidPointBotx = (lowerRight.getData()[0] + lowerLeft.getData()[0]) / 2;
                double dblMidPointBoty = (lowerRight.getData()[1] + lowerLeft.getData()[1]) / 2;

                double width = Math.sqrt((Math.pow((upperRight.getData()[1] - upperLeft.getData()[1]), 2)) + (Math.pow((upperRight.getData()[0] - upperLeft.getData()[0]), 2)));
                double height = Math.sqrt((Math.pow((dblMidPointTopy - dblMidPointBoty), 2)) + (Math.pow((dblMidPointTopx - dblMidPointBotx), 2)));

                //width is equal to 254 mm, so width of beacon is 220mm, height of beacon is 150mm
                //pixels per mm width, using a known size of the target
                double dblWidthPixelsPermm = width / TARGET_WIDTH;
                double dblHeightPixelsPermm = height / TARGET_HEIGHT;


                try {
                    fileLogger.writeEvent(debug, "Vuforia", "upperLeft 0 " + upperLeft.getData()[0]);
                    fileLogger.writeEvent(debug, "Vuforia", "upperLeft 1 " + upperLeft.getData()[1]);

                    fileLogger.writeEvent(debug, "Vuforia", "upperRight 0 " + upperRight.getData()[0]);
                    fileLogger.writeEvent(debug, "Vuforia", "upperRight 1 " + upperRight.getData()[1]);

                    fileLogger.writeEvent(debug, "Vuforia", "lowerLeft 0 " + lowerLeft.getData()[0]);
                    fileLogger.writeEvent(debug, "Vuforia", "lowerLeft 1 " + lowerLeft.getData()[1]);

                    fileLogger.writeEvent(debug, "Vuforia", "lowerRight 0 " + lowerRight.getData()[0]);
                    fileLogger.writeEvent(debug, "Vuforia", "lowerRight 1 " + lowerRight.getData()[1]);

                    fileLogger.writeEvent(debug, "Vuforia", "dblMidPointTopx " + dblMidPointTopx);
                    fileLogger.writeEvent(debug, "Vuforia", "dblMidPointTopy " + dblMidPointTopy);
                    fileLogger.writeEvent(debug, "Vuforia", "dblMidPointBotx " + dblMidPointBotx);
                    fileLogger.writeEvent(debug, "Vuforia", "dblMidPointBoty " + dblMidPointBoty);

                    fileLogger.writeEvent(debug, "Vuforia", "width in pixels " + width);
                    fileLogger.writeEvent(debug, "Vuforia", "height in pixels " + height);
                } catch (Exception e) {
                    //fileLogger.writeEvent(1,"Vuforia", "OOOPS" );
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
        fileLogger.writeEvent(debug,"fl", "Debug Level Before" + fileLogger.getDebugLevel());

        if (game1.a)
            calibrate = true;

        dashboard.displayPrintf(2, "Calibatrate " + calibrate);
        Colour = elementColour.RoverRuckusOCV(fileLogger, dashboard, tmp, 0, true, 7, calibrate);

        for (VuforiaTrackable trackable : trackables) {
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
            if (robotBearing < 0) robotBearing = 360 + robotBearing;

            dashboard.displayCenterPrintf(1,200, "*** Vision Data***");
            dashboard.displayPrintf(2, "Pos X " + robotX);
            dashboard.displayPrintf(3, "Pos Z " + robotY);
            dashboard.displayPrintf(4, "Pos Z " + robotZ);
            dashboard.displayPrintf(5, "Pos B " + robotBearing);
            dashboard.displayPrintf(6, "Pos - " + format(lastLocation));

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