package club.towr5291.Concepts;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigationWebcam;
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

import club.towr5291.functions.FileLogger;
import club.towr5291.libraries.LibraryMotorType;
import club.towr5291.libraries.robotConfig;
import club.towr5291.libraries.robotConfigSettings;
import club.towr5291.robotconfig.HardwareDriveMotors;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

@TeleOp(group = "Vufroia", name = "Vuforia Demo")
@Disabled
public class ConceptFor_PTC_OutReach2018 extends LinearOpMode {

    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor
    private static final double     COUNTS_PER_MOTOR_REV    = 537.6;
    private static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    private static final double     P_TURN_COEFF            = 0.06;     // Larger is more responsive, but also less stable
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
    float targetXmm, targetYmm, targetZmm;
    float targetXin, targetYin, targetZin;
    float rotationXDegree, rotationYDegree, rotationZDegree;
    float rotationXRadian, rotationYRadian, rotationZRadian;
    boolean usingWebCam = false;
    private HardwareDriveMotors robot;
    private BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();
        parametersIMU.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parametersIMU.loggingEnabled      = true;
        parametersIMU.loggingTag          = "IMU";
        parametersIMU.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parametersIMU);

        ourRobotConfig = new robotConfig();

        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);


        ourRobotConfig.setAllianceColor(sharedPreferences.getString("club.towr5291.Autonomous.Color", "Red"));// Using a Function to Store The Robot Specification
        ourRobotConfig.setTeamNumber(sharedPreferences.getString("club.towr5291.Autonomous.TeamNumber", "0000"));
        ourRobotConfig.setAllianceStartPosition(sharedPreferences.getString("club.towr5291.Autonomous.Position", "Left"));
        ourRobotConfig.setDelay(Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Delay", "0")));
        ourRobotConfig.setRobotMotorType(sharedPreferences.getString("club.towr5291.Autonomous.RobotMotorChoice", "ANDY40SPUR"));
        ourRobotConfig.setRobotConfigBase(sharedPreferences.getString("club.towr5291.Autonomous.RobotConfigBase", "TileRunner2x40"));

        fileLogger = new FileLogger(runtime, Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", "1")), true);// initializing FileLogger
        fileLogger.open();// Opening FileLogger
        fileLogger.writeEvent("Start", "Log Started");// First Line Add To Log

        robot = new HardwareDriveMotors();
        robot.init(fileLogger, hardwareMap, robotConfigSettings.robotConfigChoice.valueOf(ourRobotConfig.getRobotConfigBase()), LibraryMotorType.MotorTypes.valueOf(ourRobotConfig.getRobotMotorType()));// Starting robot Hardware map
        robot.logEncoderCounts(fileLogger);// Logging The Encoder Counts
        robot.allMotorsStop();
        robot.setHardwareDriveDirections(robotConfigSettings.robotConfigChoice.valueOf(ourRobotConfig.getRobotConfigBase()), LibraryMotorType.MotorTypes.valueOf(ourRobotConfig.getRobotMotorType()));

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

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        roverTrackables.activate();

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> trackables = new ArrayList<VuforiaTrackable>();
        trackables.addAll(roverTrackables);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
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
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
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

        final int CAMERA_FORWARD_DISPLACEMENT  = 0;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot;
        if (usingWebCam) {
            phoneLocationOnRobot = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                            0, 90, 90));
        } else{
                phoneLocationOnRobot = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                            CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));
        }

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : trackables) {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }
        //Orientation gyro   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        while (opModeIsActive() && !targetVisible) {
            // check all the trackable target to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : trackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;
                    targetFound = trackable;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }
            //gyro   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //gyroTurn(.2, gyro.firstAngle + 20);
            //Thread.sleep(1000);
        }

        VectorF translation = lastLocation.getTranslation();
        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
         translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

        targetXmm = translation.get(0); targetXin = translation.get(0) / mmPerInch;
        targetYmm = translation.get(1); targetYin = translation.get(1) / mmPerInch;
        targetZmm = translation.get(2); targetZin = translation.get(2) / mmPerInch;

        // express the rotation of the robot in degrees.
        Orientation rotationDegree = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
        Orientation rotationRadian = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, RADIANS);
        telemetry.addData("Rot (deg)", "{Roll (X), Pitch (Y), Heading (Z)} = %.0f, %.0f, %.0f", rotationDegree.firstAngle, rotationDegree.secondAngle, rotationDegree.thirdAngle);

        rotationXDegree = rotationDegree.firstAngle; rotationXRadian = rotationRadian.firstAngle;
        rotationYDegree = rotationDegree.secondAngle; rotationYRadian = rotationRadian.secondAngle;
        rotationZDegree = rotationDegree.thirdAngle; rotationZRadian = rotationRadian.thirdAngle;

        double turnTo = 0;
        double turnToWithGyro = 0;
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        switch (targetFound.getName()){
            case "BlueRover":
                if (rotationZDegree > 0 && rotationZDegree < 90){
                    turnTo = 90 - rotationZDegree;
                    turnToWithGyro = turnTo + angles.firstAngle;
                    gyroTurn(.1, turnToWithGyro);
                    //angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    //gyroDrive(.1, targetYin, angles.firstAngle);
                    //Thread.sleep(3000);
                    //angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    //gyroTurn(.1, 90 + angles.firstAngle);
                    //Thread.sleep(3000);
                    //angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    //gyroDrive(.1, -targetXin, angles.firstAngle);
                }

                if (rotationZDegree > 90 && rotationZDegree < 180){
                    turnTo = 180 - rotationZDegree;
                    turnToWithGyro = turnTo + angles.firstAngle;
                    gyroTurn(.1, turnToWithGyro + 90);
                    //angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    //gyroDrive(.1, -targetYin, angles.firstAngle);
                    //Thread.sleep(3000);
                    //angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    //gyroTurn(.1, 90 + angles.firstAngle);
                    //Thread.sleep(3000);
                    //angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    //gyroDrive(.1, targetXin, angles.firstAngle);
                }
                break;
            case "RedFootprint":
                if (rotationZDegree < 0 && rotationZDegree > -90){
                    turnTo = 90 - rotationZDegree;
                    turnToWithGyro = turnTo + angles.firstAngle;
                    gyroTurn(.1, turnToWithGyro + 180);
                    angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gyroDrive(.1, -targetYin, angles.firstAngle);
                    Thread.sleep(3000);
                    angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gyroTurn(.1, 90 + angles.firstAngle);
                    Thread.sleep(3000);
                    angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gyroDrive(.1, targetXin, angles.firstAngle);
                }

                if (rotationZDegree < -90 && rotationZDegree > -180){
                    turnTo = 180 - rotationZDegree;
                    turnToWithGyro = turnTo + angles.firstAngle;
                    gyroTurn(.1, turnToWithGyro + 90);
                    angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gyroDrive(.1, -targetYin, angles.firstAngle);
                    Thread.sleep(3000);
                    angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gyroTurn(.1, 90 + angles.firstAngle);
                    Thread.sleep(3000);
                    angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gyroDrive(.1, targetXin, angles.firstAngle);
                }
                break;
            case "FrontCraters":
                if (rotationZDegree > 90 && rotationZDegree < 180){
                    turnTo = 90 - rotationZDegree;
                    turnToWithGyro = turnTo + angles.firstAngle;
                    gyroTurn(.1, turnToWithGyro + 90);
                    angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gyroDrive(.1, -targetXin, angles.firstAngle);
                    Thread.sleep(3000);
                    angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gyroTurn(.1, 90 + angles.firstAngle);
                    Thread.sleep(3000);
                    angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gyroDrive(.1, -targetYin, angles.firstAngle);
                }

                if (rotationZDegree < -90 && rotationZDegree > -180){
                    turnTo = 180 - rotationZDegree;
                    turnToWithGyro = turnTo + angles.firstAngle;
                    gyroTurn(.1, turnToWithGyro + 180);
                    angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gyroDrive(.1, targetXin, angles.firstAngle);
                    Thread.sleep(3000);
                    angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gyroTurn(.1, 90 + angles.firstAngle);
                    Thread.sleep(3000);
                    angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gyroDrive(.1, targetYin, angles.firstAngle);
                }
                break;
            case "BackSpace":
                if (rotationZDegree > 0 && rotationZDegree > -90){
                    turnTo = 90 - rotationZDegree;
                    turnToWithGyro = turnTo + angles.firstAngle;
                    gyroTurn(.1, turnToWithGyro + 180);
                    angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gyroDrive(.1, -targetYin, angles.firstAngle);
                    Thread.sleep(3000);
                    angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gyroTurn(.1, 90 + angles.firstAngle);
                    Thread.sleep(3000);
                    angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gyroDrive(.1, targetXin, angles.firstAngle);
                }

                if (rotationZDegree < -90 && rotationZDegree > -180){
                    turnTo = 180 - rotationZDegree;
                    turnToWithGyro = turnTo + angles.firstAngle;
                    gyroTurn(.1, turnToWithGyro + 90);
                    angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gyroDrive(.1, -targetYin, angles.firstAngle);
                    Thread.sleep(3000);
                    angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gyroTurn(.1, 90 + angles.firstAngle);
                    Thread.sleep(3000);
                    angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gyroDrive(.1, targetXin, angles.firstAngle);
                }
                break;
        }


        telemetry.update();
        while(opModeIsActive()){
            telemetry.addData("Target", targetFound.getName());
            telemetry.addData("move count baseMotor1", robot.baseMotor1.getCurrentPosition());
            telemetry.addData("rotationXDegree", rotationXDegree);
            telemetry.addData("rotationYDegree", rotationYDegree);
            telemetry.addData("rotationZDegree", rotationZDegree);
            telemetry.addData("targetYin", targetYin);
            telemetry.addData("targetXin", targetXin);
            telemetry.addData("targetZin", targetZin);
            telemetry.addData("turnTo", turnTo);
            telemetry.addData("turnToWithGyro", turnToWithGyro);
            telemetry.update();
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    private int gyroDrive ( double speed, double distance, double angle) {
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

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
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

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.baseMotor1.isBusy() && robot.baseMotor2.isBusy() && robot.baseMotor3.isBusy() && robot.baseMotor4.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.setHardwareDriveLeftMotorPower(leftSpeed);
                robot.setHardwareDriveRightMotorPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget1,  newRightTarget1);
                telemetry.addData("Place", "%7d:%7d", robot.baseMotor1.getCurrentPosition(), robot.baseMotor3.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.setHardwareDriveLeftMotorPower(0);
            robot.setHardwareDriveRightMotorPower(0);

            // Turn off RUN_TO_POSITION
            robot.setHardwareDriveRunUsingEncoders();
        }
        return newLeftTarget1;
    }

    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.setHardwareDriveLeftMotorPower(0);
        robot.setHardwareDriveRightMotorPower(0);
    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error;
        double   steer;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.setHardwareDriveLeftMotorPower(leftSpeed);
        robot.setHardwareDriveRightMotorPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    private double getError(double targetAngle) {

        double robotError;


        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // calculate error in -179 to +180 range  (
        robotError = targetAngle - Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
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

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
