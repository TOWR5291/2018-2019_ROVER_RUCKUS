
package club.towr5291.Concepts;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import android.widget.TextView;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigationWebcam;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.android.OpenCVLoader;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import club.towr5291.R;
import club.towr5291.functions.FileLogger;
import club.towr5291.libraries.robotConfig;
import club.towr5291.libraries.robotConfigSettings;
import club.towr5291.opmodes.OpModeMasterLinear;
import club.towr5291.robotconfig.HardwareDriveMotors;

import club.towr5291.libraries.TOWRDashBoard;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

@TeleOp(group = "Concepts", name = "Vuforia Demo2")
//@Disabled
public class ConceptPTCDemo2  extends OpModeMasterLinear {

    private static final float      mmPerInch        = 25.4f;
    private static final float      mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float      mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor
    private static final double     COUNTS_PER_MOTOR_REV    = 537.6;
    private static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double     HEADING_THRESHOLD       = 3 ;      // As tight as we can make it with an integer gyro
    private static final double     P_TURN_COEFF            = 0.007;     // Larger is more responsive, but also less stable
    private static final double     P_DRIVE_COEFF           = 0.05;     // Larger is more responsive, but also less stable

    private static double gyroDriveError = 0;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    WebcamName webcamName;
    VuforiaTrackable blueRover, redFootPrint, frontCraters, backSpace;
    FileLogger fileLogger;
    SharedPreferences sharedPreferences;
    robotConfig ourRobotConfig;
    ElapsedTime runtime = new ElapsedTime();
    boolean targetVisible;
    VuforiaTrackable targetFound;
    float robotXmm, robotYmm, robotZmm;
    float robotXin, robotYin, robotZin;
    float rotationXDegree, rotationYDegree, rotationZDegree;
    float rotationXRadian, rotationYRadian, rotationZRadian;
    boolean usingWebCam = false;
    private HardwareDriveMotors robot;
    private BNO055IMU imu;

    private static TOWRDashBoard dashboard = null;

    public static TOWRDashBoard getDashboard() {
        return dashboard;
    }

    double turnTo = 0;
    double turnToWithGyro = 0;

    double adafruitIMUHeading;
    int calculateState = 2; //0 = init, 1 = in process, 2 = finished
    int moveState = 2;  //0 = init, 1 = in process, 2 = finished
    int turnState = 2;  //0 = init, 1 = in process, 2 = finished
    int requiredMoveAngle;
    boolean done;
    int step = 0;
    double gyroOffset;
    boolean gyroOffsetSet = false;
    int     newLeftTarget1 = 0;
    int     newLeftTarget2;
    int     newRightTarget1;
    int     newRightTarget2;
    int     moveCounts = 0;
    double  max;
    double  error;
    double  steer;
    double  leftSpeed;
    double  rightSpeed;
    double  driveAngle;
    double  requiredMoveDistance;

    @Override
    public void runOpMode() throws InterruptedException {

        dashboard = TOWRDashBoard.createInstance(telemetry);
        dashboard = TOWRDashBoard.getInstance();

        FtcRobotControllerActivity activity = (FtcRobotControllerActivity) hardwareMap.appContext;

        dashboard.setTextView((TextView) activity.findViewById(R.id.textOpMode));
        dashboard.displayPrintf(0, 200, "Text: ", "*** Robot Data ***");
        //start the logging

        ourRobotConfig = new robotConfig();

        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);

        ourRobotConfig.setAllianceColor(sharedPreferences.getString("club.towr5291.Autonomous.Color", "Red"));// Using a Function to Store The Robot Specification
        ourRobotConfig.setTeamNumber(sharedPreferences.getString("club.towr5291.Autonomous.TeamNumber", "0000"));
        ourRobotConfig.setAllianceStartPosition(sharedPreferences.getString("club.towr5291.Autonomous.Position", "Left"));
        ourRobotConfig.setDelay(Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Delay", "0")));
        ourRobotConfig.setRobotConfigBase(sharedPreferences.getString("club.towr5291.Autonomous.RobotConfig", "TileRunner-2x40"));

        ourRobotConfig.initConfig();

        //create logging based on initial settings, sharepreferences will adjust levels
        fileLogger = new FileLogger(runtime, Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", "1")), true);// initializing FileLogger
        fileLogger.open();
        fileLogger.write("Time,SysMS,Thread,Event,Desc");
        fileLogger.writeEvent("OpModeStart", "Log Started");
        runtime.reset();
        dashboard.displayPrintf(1, "FileLogger: Started");

        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();
        parametersIMU.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parametersIMU.loggingEnabled = true;
        parametersIMU.loggingTag = "IMU";
        parametersIMU.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parametersIMU);

        dashboard.displayPrintf(1, "IMU Inited");

        robot = new HardwareDriveMotors();
        robot.init(fileLogger, hardwareMap, robotConfigSettings.robotConfigChoice.valueOf(ourRobotConfig.getRobotConfigBase()));// Starting robot Hardware map
        robot.logEncoderCounts(fileLogger);// Logging The Encoder Counts
        robot.allMotorsStop();
        robot.setHardwareDriveDirections(robotConfigSettings.robotConfigChoice.valueOf(ourRobotConfig.getRobotConfigBase()));

        dashboard.displayPrintf(2, "Hardware Loaded");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AW9Mt0r/////AAAAGZTQpkbS80ZAmcBkWIDLcy6CS9VhLDFoyKS2MgMoVfFc4dJZnfKCp8KkOAJoW9SBWFImTgCniDMEbhB4Pk1R+q1R5iWbeE9m6JhgwNT1kZOmpJohh59A8H8yEqkl0v7gL4LgRLVWH/GrOw+RVxCNrP2kYNyr9mAoCGxoU8pKZQ2XUPDclGl5xzt0y4yTElsZL+92X6tJ7uEOoqhyvoviORCWU3oPDUQX9ki7ZedBC0IWXZWu38Uw/XuIJNGvDw3YfX7zs1z6rfTvAh85jns5l2PZ4QQtknCCBd4ynRFkG+d0PIYKAdPDZ47gJ4jm5scg9cp3Am2lElbPQOuv0SnYLyqD2DPsnclhBQz3PVt3smr0";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables roverTrackables = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        blueRover = roverTrackables.get(0);
        blueRover.setName("BlueRover");
        redFootPrint = roverTrackables.get(1);
        redFootPrint.setName("RedFootprint");
        frontCraters = roverTrackables.get(2);
        frontCraters.setName("FrontCraters");
        backSpace = roverTrackables.get(3);
        backSpace.setName("BackSpace");

        roverTrackables.activate();

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> trackables = new ArrayList<VuforiaTrackable>();
        trackables.addAll(roverTrackables);

        /**
         * In order for localization to work, we need to tell the system where each ` is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * This Rover Ruckus sample places a specific target in the middle of each perimeter wall.
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        /**
         * To place the BlueRover target in the middle of the blue perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Then, we translate it along the Y axis to the blue perimeter wall.
         */
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        /**
         * To place the RedFootprint target in the middle of the red perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative Y axis to the red perimeter wall.
         */
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootPrint.setLocation(redFootprintLocationOnField);

        /**
         * To place the FrontCraters target in the middle of the front perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative X axis to the front perimeter wall.
         */
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        /**
         * To place the BackSpace target in the middle of the back perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the X axis to the back perimeter wall.
         */
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        /**
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
         * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
         *
         * If using the rear (High Res) camera:
         * We need to rotate the camera around it's long axis to bring the rear camera forward.
         * This requires a negative 90 degree rotation on the Y axis
         *
         * If using the Front (Low Res) camera
         * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
         * This requires a Positive 90 degree rotation on the Y axis
         *
         * Next, translate the camera lens to where it is on the robot.
         * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
         */

        final int CAMERA_FORWARD_DISPLACEMENT = 220;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 150;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot;
        if (usingWebCam) {
            phoneLocationOnRobot = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                            0, 90, 90));
        } else {
            phoneLocationOnRobot = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                             90, 0, 0));
        }

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : trackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }
        OpenGLMatrix robotLocationTransform = null;
        dashboard.displayPrintf(3, " Vuforia Complete");

        targetVisible = false;
        dashboard.displayPrintf(0, "Press Play to start");
        waitForStart();

        dashboard.clearDisplay();
        dashboard.displayPrintf(0, "Running");

        while (opModeIsActive()) {
            while (!targetVisible) {
                dashboard.displayPrintf(1, "Looking For Target");

                // check all the trackable target to see which one (if any) is visible.
                for (VuforiaTrackable trackable : trackables) {

                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {

                        dashboard.displayPrintf(1, "Looking For Target %s", trackable.getName());
                        //telemetry.addData("Visible Target", trackable.getName());
                        targetVisible = true;
                        targetFound = trackable;

                        // getUpdatedRobotLocation() will return null if no new information is available since
                        // the last time that call was made, or if the trackable is not currently visible.
                        robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            lastLocation = robotLocationTransform;
                        }
                        break;
                    }
                }
                if (robotLocationTransform != null) {

                    VectorF translation = lastLocation.getTranslation();

                    Orientation rotationDegree = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                    //Orientation rotationRadian = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, RADIANS);

                    //get camera location position x,y,z position from target in mm
                    robotXmm = translation.get(0);
                    robotYmm = translation.get(1);
                    robotZmm = translation.get(2);

                    //get camera location position x,y,z position from target in inches
                    robotXin = translation.get(0) / mmPerInch;
                    robotYin = translation.get(1) / mmPerInch;
                    robotZin = translation.get(2) / mmPerInch;

                    //get camera location angle in degrees
                    rotationXDegree = rotationDegree.firstAngle;
                    rotationYDegree = rotationDegree.secondAngle;
                    rotationZDegree = rotationDegree.thirdAngle;

                    //get angle location angle in radians
                    //rotationYRadian = rotationRadian.secondAngle;
                    //rotationXRadian = rotationRadian.firstAngle;
                    //rotationZRadian = rotationRadian.thirdAngle;

                    rotationZDegree = rotationZDegree + 180;

                    dashboard.displayPrintf(2, "Pos (mm) {X, Y, Z} = %.1f, %.1f, %.1f",
                                robotXmm, robotYmm, robotZmm);
                    dashboard.displayPrintf(3, "Rot (deg) {Roll (X), Pitch (Y), Heading (Z)} = %.0f, %.0f, %.0f", rotationXDegree, rotationYDegree, rotationZDegree);

                    fileLogger.writeEvent(2, "OpMode()", "Pos (mm) {X, Y, Z} = " + robotXmm + ", " + robotYmm + ", " + robotYmm);
                    fileLogger.writeEvent(2, "OpMode()", "Rot (deg) {Roll (X), Pitch (Y), Heading (Z)} = " + rotationXDegree + ", " + rotationYDegree + ", " + rotationZDegree);

                    //resetImuOffset(rotationZDegree);
                    dashboard.displayPrintf(4, "Current Gyro Reading " + getAdafruitHeading());
                    fileLogger.writeEvent(2, "OpMode() GyroError " + gyroOffset);
                    fileLogger.writeEvent(2, "OpMode() Gyro " + getAdafruitHeading());

                    if (!gyroOffsetSet)
                        gyroOffset = rotationZDegree;
                    gyroOffsetSet = true;
                    fileLogger.writeEvent(2, "OpMode() Gyro Corrected " + (getAdafruitHeading() ));
                    dashboard.displayPrintf(5, "Current Gyro Corrected " + (getAdafruitHeading() ));
                    calculateState = 1;
                    VuforiaCalculate();
                    fileLogger.writeEvent(2, "OpMode() Calculated Offset " + requiredMoveAngle);
                    dashboard.displayPrintf(6, "Move Angle " + requiredMoveAngle);
                }
            }

            adafruitIMUHeading = getAdafruitHeading();

            // turn robot to face 0,0 (center of field
            //determine if we are in the right location (1 tile size 600mm x 600mm
            switch (step) {
                case 0:
                    if (calculateState == 2)
                        calculateState = 0;
                    break;
                case 1:
                    if (turnState == 2) {
                        turnState = 0;
                        driveAngle = getAdafruitHeading().intValue();
                    }
                    break;
                case 2:
                    if (moveState == 2)
                        moveState = 0;
                    break;
                case 3:
                    done = true;
                    break;
                default:
                    break;
            }

            VuforiaCalculate();
            gyroTurn(.5, requiredMoveAngle);
            gyroDrive(.5 , -(requiredMoveDistance - 12), driveAngle);

            if (done) {
                break;
            }
        }

        //stop the log
        if (fileLogger != null) {
            fileLogger.writeEvent("OpModeStop", "Stopped");
            fileLogger.close();
            fileLogger = null;
        }

    }

    private void VuforiaCalculate ()
    {
        switch (calculateState) {
            case 0: {
                calculateState = 1;
                fileLogger.writeEvent(2,"VuforiaMove()", "Initialised");
                dashboard.displayPrintf(5, "Vuforia Calculate Init");
            }
            break;
            case 1:
            {
                dashboard.displayPrintf(5, "Vuforia Calculate Running");
                fileLogger.writeEvent(2,"VuforiaMove()", "Running" );

                int currentX = (int)robotXmm;
                int currentY = (int)robotYmm;
                int intLocalisedRobotBearing = (int)rotationZDegree;
                double requiredMoveX = -(currentX);
                double requiredMoveY = -(currentY);

                requiredMoveDistance = ((Math.sqrt(requiredMoveX * requiredMoveX + requiredMoveY * requiredMoveY)) / 25.4) - 2;
                double requiredMoveAngletemp1 = requiredMoveX/requiredMoveY;
                double requiredMoveAngletemp2 = Math.atan(requiredMoveAngletemp1);
                double requiredMoveAngletemp3 = Math.toDegrees(requiredMoveAngletemp2);
                requiredMoveAngle = (int)Math.abs(requiredMoveAngletemp3);

                fileLogger.writeEvent(2,"VuforiaMove()", "Temp Values requiredMoveAngletemp1 " + requiredMoveAngletemp1 + " requiredMoveAngletemp2 " + requiredMoveAngletemp2 + " requiredMoveAngletemp3 " + requiredMoveAngletemp3);
                fileLogger.writeEvent(2,"VuforiaMove()", "Temp Values currentX " + currentX + " currentY " + currentY);
                fileLogger.writeEvent(2,"VuforiaMove()", "Localised, determining angles....requiredMoveX " + requiredMoveX + " requiredMoveY " + requiredMoveY);
                fileLogger.writeEvent(2,"VuforiaMove()", "Localised, determining angles....requiredMoveDistance " + requiredMoveDistance + " requiredMoveAngle " + requiredMoveAngle);

                switch (targetFound.getName()) {
                    case "BlueRover":
                        if ((((int) 0) < currentY) && ((int) 0 > currentX)) {
                            requiredMoveAngle = ((180 + 6*requiredMoveAngle) + getAdafruitHeading().intValue());
                            fileLogger.writeEvent(2,"BlueRover 1 ");
                        } else {
                            requiredMoveAngle = (180 - 4*requiredMoveAngle) + getAdafruitHeading().intValue();
                            fileLogger.writeEvent(2,"BlueRover 2 ");
                        }
                    break;
                    case "FrontCraters":
                        if ((((int) 0) < currentY) && ((int) 0 > currentX)) {
                            //requiredMoveAngle = (180 - 2*requiredMoveAngle + getAdafruitHeading().intValue());
                            requiredMoveAngle = 180 - 3 * (270 - getAdafruitHeading().intValue());
                            fileLogger.writeEvent(2,"Front Craters 1 ");
                        } else {
                            requiredMoveAngle = 180 + 2 * (270 - getAdafruitHeading().intValue());
                            fileLogger.writeEvent(2,"Front Craters 2 ");
                        }
                    break;
                    case "RedFootprint":
                        if ((((int) 0) > currentY) && ((int) 0 > currentX)) {
                            requiredMoveAngle = (180 - 4*requiredMoveAngle) + getAdafruitHeading().intValue();
                            fileLogger.writeEvent(2,"RedFootPrint 1 ");
                        } else {
                            requiredMoveAngle = (180 + 3*requiredMoveAngle) + getAdafruitHeading().intValue();
                            fileLogger.writeEvent(2,"RedFootPrint 2 ");
                        }
                    break;
                    case "BackSpace":
                        if ((((int) 0) > currentY) && ((int) 0 < currentX)) {
                            requiredMoveAngle = (180 + 6*requiredMoveAngle + getAdafruitHeading().intValue());
                            fileLogger.writeEvent(2,"BackSpace 1 ");
                        } else {
                            requiredMoveAngle = (180 - 4*requiredMoveAngle + getAdafruitHeading().intValue());
                            fileLogger.writeEvent(2,"BackSpace 2 ");
                        }
                    break;
                }
                //requiredMoveAngle = (int)angleToHeading(requiredMoveAngle);
                /*
                if ((((int) 0) > currentY) && ((int) 0 > currentX)) {
                    requiredMoveAngle = requiredMoveAngle;
                } else if ((((int) 0) > currentY) && ((int) 0 < currentX)) {
                    requiredMoveAngle = requiredMoveAngle;
                } else if ((((int) 0) < currentY) && ((int) 0 > currentX)) {
                    requiredMoveAngle = 270 + requiredMoveAngle;
                } else if ((((int) 0) < currentY) && ((int) 0 < currentX)) {
                    requiredMoveAngle = 270 + requiredMoveAngle;
                }
                */
                calculateState = 2;
                step += 1;
            }
            break;
        }
    }

    private void gyroDrive ( double speed, double distance, double angle) {

        switch (moveState) {
            case 0: {
                robot.setHardwareDriveResetEncoders();
                robot.setHardwareDriveRunUsingEncoders();
                // Determine new target position, and pass to motor controller
                moveCounts = (int) (distance * COUNTS_PER_INCH);
                fileLogger.writeEvent(2, "COUNTS_PER_INCH " + COUNTS_PER_INCH);
                fileLogger.writeEvent(2, "distance " + distance);
                newLeftTarget1 = robot.baseMotor1.getCurrentPosition() + moveCounts;
                newLeftTarget2 = robot.baseMotor2.getCurrentPosition() + moveCounts;
                newRightTarget1 = robot.baseMotor3.getCurrentPosition() + moveCounts;
                newRightTarget2 = robot.baseMotor4.getCurrentPosition() + moveCounts;

                // Set Target and Turn On RUN_TO_POSITION
                robot.baseMotor1.setTargetPosition(newLeftTarget1);
                robot.baseMotor2.setTargetPosition(newLeftTarget2);
                robot.baseMotor3.setTargetPosition(newRightTarget1);
                robot.baseMotor4.setTargetPosition(newRightTarget2);

                robot.setHardwareDriveRunToPosition();

                // start motion.
                speed = Range.clip(Math.abs(speed), 0.0, 1.0);
                robot.setHardwareDriveLeftMotorPower(speed);
                robot.setHardwareDriveRightMotorPower(speed);
                moveState = 1;
                fileLogger.writeEvent(2, "gyroDrive", "Initialised");
                fileLogger.writeEvent(2, "newLeftTarget1 " + newLeftTarget1);
                fileLogger.writeEvent(2, "newLeftTarget2 " + newLeftTarget2);
                fileLogger.writeEvent(2, "newRightTarget1 " + newRightTarget1);
                fileLogger.writeEvent(2, "newRightTarget2 " + newRightTarget2);
                dashboard.displayPrintf(5, "GyroDrive Initialized");
            }
            break;
            case 1: {
                fileLogger.writeEvent(2,"GyroDrive()", "Running");

                // keep looping while we are still active, and BOTH motors are running.
                if ((robot.baseMotor1.isBusy() && robot.baseMotor2.isBusy() && robot.baseMotor3.isBusy() && robot.baseMotor4.isBusy())) {
                    fileLogger.writeEvent(2,"GyroDrive()", "Motors Running");
                    // adjust relative speed based on heading error.
                    //error = getError(angle);
                    //steer = getSteer(error, P_DRIVE_COEFF);

                    // if driving in reverse, the motor correction also needs to be reversed
                    //if (distance < 0)
                    //    steer *= -1.0;

                    leftSpeed = speed;// - steer;
                    rightSpeed = speed;//+ steer;

                    // Normalize speeds if either one exceeds +/- 1.0;
                    max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                    if (max > 1.0) {
                        leftSpeed /= max;
                        rightSpeed /= max;
                    }

                    robot.setHardwareDriveLeftMotorPower(leftSpeed);
                    robot.setHardwareDriveRightMotorPower(rightSpeed);

                    // Display drive status for the driver.
                    telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                    telemetry.addData("Target", "%7d:%7d", newLeftTarget1, newRightTarget1);
                    telemetry.addData("Place", "%7d:%7d", robot.baseMotor1.getCurrentPosition(), robot.baseMotor3.getCurrentPosition());
                    telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                    telemetry.update();
                } else {
                    fileLogger.writeEvent(2,"GyroDrive()", "Motors Not Running");
                    // Stop all motion;
                    robot.setHardwareDriveLeftMotorPower(0);
                    robot.setHardwareDriveRightMotorPower(0);

                    // Turn off RUN_TO_POSITION
                    robot.setHardwareDriveRunUsingEncoders();

                    moveState = 2;
                    step += 1;

                }
            }
        }
    }


    public void gyroTurn (double speed, double angle) {
        switch (turnState) {
            case 0: {
                turnState = 1;
                fileLogger.writeEvent(2, "gyroTurn()", "Initialised");
                dashboard.displayPrintf(5, "GyroTurn Initialized");
            }
            break;
            case 1: {
                dashboard.displayPrintf(5, "GyroTurn Running");
                fileLogger.writeEvent(2,"gyroTurn()", "Running");
                fileLogger.writeEvent(2,"gyroTurn()", "Speed " + speed + " Angle " +angle);
                fileLogger.writeEvent(2,"gyroTurn()", "Gyro " +  getAdafruitHeading());
                // keep looping while we are still active, and not on heading.
                if (onHeading(speed, angle, P_TURN_COEFF)) {
                    turnState = 2;
                    step += 1;
                    dashboard.displayPrintf(4, "GyroTurn Finished");
                    fileLogger.writeEvent(2,"gyroTurn()", "Complete");
                }
            }
            break;
        }
    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error;
        double   steer;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = -getError2(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }  else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.setHardwareDriveLeftMotorPower(leftSpeed);
        robot.setHardwareDriveRightMotorPower(rightSpeed);

        // Display it for the driver
        dashboard.displayPrintf(7, "Target " + angle);
        dashboard.displayPrintf(8, "Err/St " + error + " " + steer);
        dashboard.displayPrintf(9, "leftSpeed, rightSpeed" + leftSpeed + " " + rightSpeed);

        return onTarget;
    }

    private double getError(double targetAngle) {

        double robotError;

        //Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getAdafruitHeading();
        fileLogger.writeEvent(2,"gyroTurn() ERROR 1 " + robotError);
        if (robotError > 180)
            robotError -= 360;
        else if (robotError <= -180)
            robotError += 360;
        fileLogger.writeEvent(2,"gyroTurn() ERROR 2 " + robotError);
        return robotError;
    }

    private double getError2(double targetAngle) {

        double robotError;
        if (targetAngle > 360)
            targetAngle = targetAngle - 360;
        //Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // calculate error in -179 to +180 range  (
        //if (targetAngle < 0 )
        //    robotError = 180 + targetAngle - getAdafruitHeading() ;
        //else
        robotError = getAdafruitHeading() - targetAngle;
        if (robotError > 180)
            robotError -= 360;
        fileLogger.writeEvent(2,"gyroTurn() ERROR2 " + robotError);
        return robotError;
    }


    private double TurnToHeading (float input){
        while (input > 180)  input -= 360;
        while (input <= -180) input += 360;
        return input;
    }
    private double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    //for adafruit IMU
    private Double formatAngle(AngleUnit angleUnit, double angle) {
        return AngleUnit.DEGREES.fromUnit(angleUnit, angle);
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    private void resetImuOffset (double currentAngle) {
        gyroOffset = 0;
        gyroOffset = currentAngle - getAdafruitHeading();
        if (gyroOffset < 0)
            gyroOffset = gyroOffset + 360;
        else if (gyroOffset > 360)
            gyroOffset = gyroOffset - 360;
        else
            gyroOffset = gyroOffset;
    }

    private Double getAdafruitHeading ()
    {
        Orientation angles;
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return angleToHeading(formatAngle(angles.angleUnit, angles.firstAngle + gyroOffset));
    }

    //for adafruit IMU as it returns z angle only
    private double angleToHeading(double z) {
        double angle = z ;
        if (angle < 0)
            return angle + 360;
        else if (angle > 360)
            return angle - 360;
        else
            return angle;
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

}


