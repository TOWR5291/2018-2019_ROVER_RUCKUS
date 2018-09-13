package club.towr5291.opmodes;

import android.content.SharedPreferences;
import android.os.Build;
import android.preference.PreferenceManager;
import android.widget.TextView;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import club.towr5291.R;
import club.towr5291.functions.FileLogger;
import club.towr5291.functions.TOWR5291PID;
import club.towr5291.functions.TOWR5291Tick;
import club.towr5291.functions.TOWR5291Toggle;
import club.towr5291.libraries.TOWRDashBoard;
import club.towr5291.libraries.robotConfig;
import club.towr5291.libraries.robotConfigSettings;
import club.towr5291.robotconfig.HardwareDriveMotors;

/*
    made by Wyatt Ashley on 8/2/2018
 */
@TeleOp(name = "Base Drive 2019", group = "Base drive")
public class BaseDrive_2019 extends OpMode{

    private HardwareDriveMotors Robot = new HardwareDriveMotors();

    //Settings from the sharepreferences
    private SharedPreferences sharedPreferences;
    double correction = 0;
    double lastposition;

    private FileLogger fileLogger;
    final String TAG = "Concept Logging";
    private ElapsedTime runtime = new ElapsedTime();
    private club.towr5291.libraries.robotConfig ourRobotConfig;
    private TOWR5291Tick robotTick = new TOWR5291Tick();
    private TOWR5291Tick switchRobotController = new TOWR5291Tick();
    private TOWR5291Toggle lightToggle;
    private Gamepad FIRST_GAMEPAD = gamepad1;
    private Gamepad SECOND_GAMEPAD = gamepad2;

    private enum LEDState {
        STATE_ERROR,
        STATE_TEAM,
        STATE_MOVING,
        STATE_BEACON,
        STATE_SUCCESS,
        STATE_SHOOTING,
        STATE_FORKS,
        STATE_LIFT,
        STATE_FINISHED,
        STATE_CELEBRATION
    }

    private boolean BlueRedState = false;
    DeviceInterfaceModule dim;                  // Device Object
    final int GREEN1_LED_CHANNEL = 0;
    final int RED1_LED_CHANNEL = 1;
    final int BLUE1_LED_CHANNEL = 2;
    final int GREEN2_LED_CHANNEL = 3;
    final int RED2_LED_CHANNEL = 4;
    final int BLUE2_LED_CHANNEL = 5;
    final boolean LedOn = false;
    private ElapsedTime mStateTime = new ElapsedTime();

    private TOWR5291PID driftRotateAngle;
    private BNO055IMU imu;

    private static TOWRDashBoard dashboard = null;

    public static TOWRDashBoard getDashboard()
    {
        return dashboard;
    }

    @Override
    public void init() {
        dashboard = TOWRDashBoard.createInstance(telemetry);

        FtcRobotControllerActivity act = (FtcRobotControllerActivity)(hardwareMap.appContext);

        dashboard.setTextView((TextView)act.findViewById(R.id.textOpMode));
        dashboard.displayPrintf(0, "Starting Menu System");


        BNO055IMU.Parameters parametersAdafruitImu = new BNO055IMU.Parameters();
        parametersAdafruitImu.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersAdafruitImu.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersAdafruitImu.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parametersAdafruitImu.loggingEnabled = true;
        parametersAdafruitImu.loggingTag = "IMU";
        parametersAdafruitImu.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parametersAdafruitImu);

        lightToggle = new TOWR5291Toggle(SECOND_GAMEPAD.left_stick_button);
        ourRobotConfig = new robotConfig();

        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);

        ourRobotConfig.setAllianceColor(sharedPreferences.getString("club.towr5291.Autonomous.Color", "Red"));// Using a Function to Store The Robot Specification
        ourRobotConfig.setTeamNumber(sharedPreferences.getString("club.towr5291.Autonomous.TeamNumber", "0000"));
        ourRobotConfig.setAllianceStartPosition(sharedPreferences.getString("club.towr5291.Autonomous.Position", "Left"));
        ourRobotConfig.setDelay(Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Delay", "0")));
        ourRobotConfig.setRobotConfigBase(sharedPreferences.getString("club.towr5291.Autonomous.RobotConfig", "TileRunner-2x40"));

        fileLogger = new FileLogger(runtime, Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", "1")), true);// initializing FileLogger
        fileLogger.open();// Opening FileLogger
        fileLogger.writeEvent(TAG, "Log Started");// First Line Add To Log

        // get a reference to a Modern Robotics DIM, and IO channels.
        dim = hardwareMap.get(DeviceInterfaceModule.class, "dim");   //  Use generic form of device mapping
        dim.setDigitalChannelMode(GREEN1_LED_CHANNEL, DigitalChannelController.Mode.OUTPUT); // Set the direction of each channel
        dim.setDigitalChannelMode(RED1_LED_CHANNEL, DigitalChannelController.Mode.OUTPUT); // Set the direction of each channel
        dim.setDigitalChannelMode(BLUE1_LED_CHANNEL, DigitalChannelController.Mode.OUTPUT); // Set the direction of each channel
        dim.setDigitalChannelMode(GREEN2_LED_CHANNEL, DigitalChannelController.Mode.OUTPUT); // Set the direction of each channel
        dim.setDigitalChannelMode(RED2_LED_CHANNEL, DigitalChannelController.Mode.OUTPUT); // Set the direction of each channel
        dim.setDigitalChannelMode(BLUE2_LED_CHANNEL, DigitalChannelController.Mode.OUTPUT); // Set the direction of each channel

        LedState(LedOn, LedOn, LedOn, LedOn, LedOn, LedOn);// Setting Leds To Wight

        Robot.init(fileLogger, hardwareMap, robotConfigSettings.robotConfigChoice.valueOf(ourRobotConfig.getRobotConfigBase()));// Starting robot Hardware map

        Robot.logEncoderCounts(fileLogger);// Logging The Encoder Counts
        Robot.allMotorsStop();

        robotTick.setRollOver(true);
        robotTick.setTickMax(1);
        robotTick.setTickMin(0.1);
        robotTick.setTickIncrement(0.1);

        switchRobotController.setRollOver(true);
        switchRobotController.setTickMin(1);
        switchRobotController.setTickIncrement(1);

        lightToggle.setDebounce(250);

        driftRotateAngle = new TOWR5291PID(runtime,0,0,4.5,0,0);

        // All The Specification of the robot and controller
        fileLogger.writeEvent("Alliance Color", ourRobotConfig.getAllianceColor());
        fileLogger.writeEvent("Alliance Start Position", ourRobotConfig.getAllianceStartPosition());
        fileLogger.writeEvent("Delay", String.valueOf(ourRobotConfig.getDelay()));
        fileLogger.writeEvent("Robot Base Config", ourRobotConfig.getRobotConfigBase());
        fileLogger.writeEvent("Team Number", ourRobotConfig.getTeamNumber());
        fileLogger.writeEvent("Robot Controller Max Tick", String.valueOf(switchRobotController.getTickMax()));
        fileLogger.writeEvent("Robot Controller Min Tick", String.valueOf(switchRobotController.getTickMin()));
    }

    @Override
    public void start(){
        fileLogger.writeEvent("Starting Loop");
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        lastposition = getAdafruitHeading();
    }


    @Override
    public void loop() {
        setControllerMaxTick(robotConfigSettings.robotConfigChoice.valueOf(ourRobotConfig.getRobotConfigBase()));

        if (BlueRedState && mStateTime.milliseconds() < 350) LedState(false, false, true, false, true, false);
        if (BlueRedState && mStateTime.milliseconds() >= 350) LedState(false, true, false, false, false, true);

        robotTick.incrementTick(FIRST_GAMEPAD.dpad_up);
        robotTick.decrementTick(FIRST_GAMEPAD.dpad_down);

        switchRobotController.incrementTick(FIRST_GAMEPAD.start);

        dashboard.displayPrintf(0, "Current Tick" + robotTick.getTickCurrValue());

        switch ((int) switchRobotController.getTickCurrValue()){
            case 1:
                /*
                 * Case 1 is the controller type POV
                 * POV uses both joy sticks to drive
                 * The left joy stick is for forward and back
                 * The right joy sstick is for turning left and right
                 */
                dashboard.displayPrintf(1, "Controller POV");
                fileLogger.writeEvent("Controller Mode", "POV");
                Robot.setHardwareDriveLeftMotorPower(Range.clip(-FIRST_GAMEPAD.left_stick_y + FIRST_GAMEPAD.right_stick_x, -1.0, 1.0) * robotTick.getTickCurrValue());
                Robot.setHardwareDriveRightMotorPower(Range.clip(-FIRST_GAMEPAD.left_stick_y - FIRST_GAMEPAD.right_stick_x, -1.0, 1.0) * robotTick.getTickCurrValue());
                break;

            case 2:
                /*
                 * Case 2 is the controller type Tank drive
                 * Tank uses both joy sticks to drive
                 * The left joy stick is for the left wheel speed
                 * The right joy stick is for the right wheel speed
                 */
                dashboard.displayPrintf(1, "Controller Tank");
                fileLogger.writeEvent("Controller Mode", "Tank");
                Robot.setHardwareDriveLeftMotorPower(-FIRST_GAMEPAD.left_stick_y * robotTick.getTickCurrValue());
                Robot.setHardwareDriveRightMotorPower(-FIRST_GAMEPAD.right_stick_y * robotTick.getTickCurrValue());
                break;

            case 3:
                /*
                 * Does NOT HAVE VARIABLE SPEED YET
                 * Mecanum Drive is for Mecanum bases ONLY
                 * If You Need help ask some one to explain it to you
                 */
                dashboard.displayPrintf(1, "Controller Mecanum Drive");
                fileLogger.writeEvent("Controller Mode", "Mecanum Drive");
                if (FIRST_GAMEPAD.left_stick_x != 0 || FIRST_GAMEPAD.left_stick_y != 0) {
                    correction = driftRotateAngle.PIDCorrection(runtime,Math.sin(getAdafruitHeading() * (Math.PI / 180.0)), lastposition);
                } else {
                    correction = 0;
                    lastposition = Math.sin(getAdafruitHeading() * (Math.PI / 180.0));
                }

                Robot.mecanumDrive_Cartesian(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x - correction, getAdafruitHeading());
                break;
        }
    }

    private Double getAdafruitHeading () {
        Orientation angles;
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return angleToHeading(formatAngle(angles.angleUnit, angles.firstAngle));
    }
    private Double formatAngle(AngleUnit angleUnit, double angle) {
        fileLogger.writeEvent(4,"Formating Angle For Gyro");
        return AngleUnit.DEGREES.fromUnit(angleUnit, angle);
    }
    private double angleToHeading(double z) {
        double angle = -z;
        if (angle < 0)
            return angle + 360;
        else if (angle > 360)
            return angle - 360;
        else
            return angle;
    }

    private void LedState (boolean greenLED1, boolean redLED1, boolean blueLED1, boolean greenLED2, boolean redLED2, boolean blueLED2) {
        dim.setDigitalChannelState(RED1_LED_CHANNEL, redLED1);//R
        dim.setDigitalChannelState(GREEN1_LED_CHANNEL, greenLED1);//G
        dim.setDigitalChannelState(BLUE1_LED_CHANNEL, blueLED1);//B

        dim.setDigitalChannelState(RED2_LED_CHANNEL, redLED2);//R
        dim.setDigitalChannelState(GREEN2_LED_CHANNEL, greenLED2);//G
        dim.setDigitalChannelState(BLUE2_LED_CHANNEL, blueLED2);//B
    }

    private void setControllerMaxTick(robotConfigSettings.robotConfigChoice choice){
        switch (choice){
            case TileRunner2x20: switchRobotController.setTickMax(2); break;
            case TileRunner2x40: switchRobotController.setTickMax(2); break;
            case TileRunner2x60: switchRobotController.setTickMax(2); break;
            case TileRunnerOrbital2x20: switchRobotController.setTickMax(2); break;
            case TileRunnerOrbital2x40: switchRobotController.setTickMax(2); break;
            case TileRunnerOrbital2x60: switchRobotController.setTickMax(2); break;
            case TileRunnerMecanumOrbital2x20: switchRobotController.setTickMax(3); break;
            case TankTread2x40Custom: switchRobotController.setTickMax(2); break;
            case TileRunnerMecanum2x20: switchRobotController.setTickMax(3); break;
            case TileRunnerMecanum2x40: switchRobotController.setTickMax(3); break;
            case TileRunnerMecanum2x60: switchRobotController.setTickMax(3); break;
        }
        fileLogger.writeEvent(4,"Setting Controller Max Tick");
    }

    private boolean isMecanum (robotConfigSettings.robotConfigChoice choice){
        switch (choice){
            case TileRunner2x20: return false;
            case TileRunner2x40: return false;
            case TileRunner2x60: return false;
            case TileRunnerOrbital2x20: return false;
            case TileRunnerOrbital2x40: return false;
            case TileRunnerOrbital2x60: return false;
            case TileRunnerMecanumOrbital2x20: return true;
            case TileRunnerMecanum2x20: return true;
            case TileRunnerMecanum2x40: return true;
            case TileRunnerMecanum2x60: return true;
            case Custom_11231_2016: return false;
            case TankTread2x40Custom: return false;
            default: return false;
        }
    }
}
