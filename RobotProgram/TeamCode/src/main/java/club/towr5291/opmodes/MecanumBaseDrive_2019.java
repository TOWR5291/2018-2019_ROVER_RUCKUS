package club.towr5291.opmodes;

/**
 * Created by kids on 9/30/2017 at 9:55 AM.
 */

import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import android.widget.TextView;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import club.towr5291.R;
import club.towr5291.functions.FileLogger;
import club.towr5291.functions.TOWR5291PID;
import club.towr5291.libraries.robotConfigSettings;
import club.towr5291.robotconfig.HardwareDriveMotors;
import hallib.HalDashboard;

//import club.towr5291.robotconfig.HardwareArmMotorsRR;

@TeleOp(name="IAN Mecanum Test 2019", group="OpModes5291")
//@Disabled
public class MecanumBaseDrive_2019 extends OpModeMasterLinear
{

    private class MyThread implements Runnable {
        @Override
        public void run() {

        }
    }

    //set up TAG for logging prefic, this info will appear first in every log statemend
    private static final String TAG = "MecanumBaseDrive2019";

    //motors
    // Declare OpMode members.
    private HardwareDriveMotors robotDrive  = new HardwareDriveMotors();   // Use a Pushbot's hardware

    //mode selection stuff
    public int mode = 0;

    //general variables
    public float speed = 0;
    public float turn = 0;
    public float intendedTurn = 0;
    public float strafe = 0;

    //all modes variables
    public double dblLeftMotor1;
    public double dblLeftMotor2;
    public double dblRightMotor1;
    public double dblRightMotor2;

    //gyro assisted and field-centric driving variables
    public int quadrant = 1;
    public double radius = 0;
    public double heading = 0;
    public float ajustedHeading = 0;
    public float revHeading = 0;

    //The autonomous menu settings from the sharepreferences
    private SharedPreferences sharedPreferences;
    private String teamNumber;
    private String allianceColor;
    private String allianceStartPosition;
    private int delay;
    private String robotConfig;
    private ElapsedTime runtime = new ElapsedTime();

    //adafruit IMU
    // The IMU sensor object
    private BNO055IMU imu;

    //set up the variables for file logger and what level of debug we will log info at
    private FileLogger fileLogger;
    private int debug = 3;

    private static HalDashboard dashboard = null;

    public static HalDashboard getDashboard()
    {
        return dashboard;
    }

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    @Override
    public void runOpMode() throws InterruptedException
    {

        dashboard = HalDashboard.createInstance(telemetry);

        FtcRobotControllerActivity act = (FtcRobotControllerActivity)(hardwareMap.appContext);

        dashboard.setTextView((TextView)act.findViewById(R.id.textOpMode));
        dashboard.displayPrintf(0, "Starting Menu System");

        //load menu settings and setup robot and debug level
        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        teamNumber = sharedPreferences.getString("club.towr5291.Autonomous.TeamNumber", "0000");
        allianceColor = sharedPreferences.getString("club.towr5291.Autonomous.Color", "Red");
        allianceStartPosition = sharedPreferences.getString("club.towr5291.Autonomous.StartPosition", "Left");
        delay = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Delay", "0"));
        robotConfig = sharedPreferences.getString("club.towr5291.Autonomous.RobotConfig", "TileRunnerMecanum2x40");
        debug = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", "1"));

        //create logging based on initial settings, sharepreferences will adjust levels
        fileLogger = new FileLogger(runtime, debug,true);
        fileLogger.writeEvent("START", "-------------------------------------------------------------------------");
        fileLogger.write("Time,SysMS,Thread,Event,Desc");
        fileLogger.writeEvent(TAG, "Loading sharePreferences");
        runtime.reset();

        dashboard.displayPrintf(1, "FileLogger - " + runtime.toString());
        dashboard.displayPrintf(2, "FileLogger - " + fileLogger.getFilename());

        dashboard.displayPrintf(3, "Team          - " + teamNumber);
        dashboard.displayPrintf(4, "alliance      - " + allianceColor);
        dashboard.displayPrintf(5, "StartPosition - " + allianceStartPosition);
        dashboard.displayPrintf(6, "Delay         - " + delay);
        dashboard.displayPrintf(7, "Config        - " + robotConfig);
        dashboard.displayPrintf(8, "Debug         - " + debug);

        //adjust debug level based on shared preferences
        fileLogger.writeEvent(1,TAG, "Loaded sharePreferences");
        fileLogger.writeEvent(1,TAG, "Loading baseHardware");

        robotDrive.init(fileLogger, hardwareMap, robotConfigSettings.robotConfigChoice.valueOf(robotConfig));

        fileLogger.writeEvent(1,TAG, "Loaded baseHardware");
        fileLogger.writeEvent(1,TAG, "Setting setHardwareDriveRunWithoutEncoders");

        robotDrive.setHardwareDriveRunWithoutEncoders();
        fileLogger.writeEvent(1,TAG, "Set setHardwareDriveRunWithoutEncoders");

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parametersAdafruitImu = new BNO055IMU.Parameters();
        parametersAdafruitImu.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersAdafruitImu.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersAdafruitImu.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parametersAdafruitImu.loggingEnabled = true;
        parametersAdafruitImu.loggingTag = "IMU";
        parametersAdafruitImu.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parametersAdafruitImu);
        fileLogger.writeEvent(1,TAG, "IMU Initialised");

        //angle drift error PID control
        TOWR5291PID driftRotateAngle = new TOWR5291PID(runtime,0,0,4.5,0,0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 100);

        double correction = 0;
        double lastposition = getAdafruitHeading();

        dashboard.clearDisplay();

        while (opModeIsActive()) {
            if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
                correction = driftRotateAngle.PIDCorrection(runtime,Math.sin(getAdafruitHeading() * (Math.PI / 180.0)), lastposition);
            } else {
                correction = 0;
                lastposition = Math.sin(getAdafruitHeading() * (Math.PI / 180.0));
            }
            if (gamepad1.a)
                robotDrive.setMaxOutput(0.5);
            if (gamepad1.b)
                robotDrive.setMaxOutput(1);

            robotDrive.mecanumDrive_Cartesian(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x - correction, getAdafruitHeading());
            dashboard.displayPrintf(1, "IMU Heading - " + getAdafruitHeading());
            dashboard.displayPrintf(2, "Left Power  - " + gamepad1.left_stick_y);
            dashboard.displayPrintf(3, "Right Power - " + gamepad1.right_stick_y);
            dashboard.displayPrintf(4, "Error       - " + correction);
        }
        stopMode();
    }

    public void stopMode()
    {
        //stop the log
        if (fileLogger != null) {
            fileLogger.writeEvent("STOP", "-------------------------------------------------------------------------");
            fileLogger.writeEvent(TAG, "Stopped");
            fileLogger.close();
            fileLogger = null;
        }
    }   //stopMode

    public static double scaleRange(double value, double lowSrcRange, double highSrcRange, double lowDstRange, double highDstRange)
    {
        return lowDstRange + (value - lowSrcRange)*(highDstRange - lowDstRange)/(highSrcRange - lowSrcRange);
    }   //scaleRange

    public double determineHeading(float x) {
        return Math.asin(x/radius);
    }

    public float determineAjustedHeading() {
        // ajustedHeading = heading from driver - robot heading relative to driver
        ajustedHeading = (float) (heading - revHeading);
        return ajustedHeading;
    }

    public float determineSpeed(float angle, float distance) {
        speed = (float) (distance/(-Math.sin(angle)));
        return speed;
}

    public float determineStrafe(float angle, float distance) {
        strafe = (float) (distance/(-Math.sin(angle)));
        return strafe;
    }

    private Double getAdafruitHeading ()
    {
        Orientation angles;
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return angleToHeading(formatAngle(angles.angleUnit, angles.firstAngle));
    }

    //for adafruit IMU
    private Double formatAngle(AngleUnit angleUnit, double angle) {
        return AngleUnit.DEGREES.fromUnit(angleUnit, angle);
    }

    //for adafruit IMU as it returns z angle only
    private double angleToHeading(double z) {
        double angle = -z;
        if (angle < 0)
            return angle + 360;
        else if (angle > 360)
            return angle - 360;
        else
            return angle;
    }

    enum handed {

        LEFT("Left"),
        RIGHT("Right");
        private final String value;

        handed(String value) {
            this.value = value;
        }

        public String toString() {
            return value;
        }
    }


}

