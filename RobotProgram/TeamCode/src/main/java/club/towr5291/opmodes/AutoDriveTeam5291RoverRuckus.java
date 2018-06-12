/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package club.towr5291.opmodes;

import android.content.SharedPreferences;
import android.graphics.Bitmap;
import android.preference.PreferenceManager;
import android.util.Log;
import android.widget.TextView;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.util.HashMap;

import club.towr5291.R;
import club.towr5291.functions.Constants;
import club.towr5291.functions.FileLogger;
import club.towr5291.functions.JewelAnalysisOCV;
import club.towr5291.functions.ReadStepFileRoverRuckus;
import club.towr5291.libraries.LibraryStateSegAutoRoverRuckus;
import club.towr5291.libraries.robotConfig;
import club.towr5291.libraries.TOWR5291LEDControl;
import club.towr5291.libraries.LibraryVuforiaRelicRecovery;
import club.towr5291.libraries.robotConfigSettings;
import club.towr5291.libraries.TOWR5291Utils;
import club.towr5291.robotconfig.HardwareDriveMotors;
import hallib.HalDashboard;


/*
TOWR 5291 Autonomous
Copyright (c) 2016 TOWR5291
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Written by Ian Haden October 2016
2017-02-15 - Ian Haden - Changed Auton Menu
2017-02-25 - Ian Haden - Added load steps from a file
2017-03-11 - Ian Haden - Added Adafruit IMU
2017-03-11 - Ian Haden - Cleaned up code
2017-03-11 - Ian Haden - Made the load steps a class
2017-03-19 - Ian Haden - Updated Beacon Viewing Area (Crop whole picture to just beacon)
*/
@Autonomous(name="5291 Autonomous Drive Rover Ruckus", group="5291")
public class AutoDriveTeam5291RoverRuckus extends OpModeMasterLinear {

    private OpMode onStop = this;
    private OpModeManagerImpl opModeManager;
    private String TeleOpMode = "Base Drive";

    final int LABEL_WIDTH = 200;

    //The autonomous menu settings from the sharepreferences
    private SharedPreferences sharedPreferences;
    private robotConfig ourRobotConfig;

    private ElapsedTime runtime = new ElapsedTime();

    //set up the variables for file logger and what level of debug we will log info at
    public FileLogger fileLogger;
    private int debug = 3;

    //set up range sensor variables
    //set up range sensor1
    private byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable
    private I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    private static final int RANGE1_REG_START = 0x04; //Register to start reading
    private static final int RANGE1_READ_LENGTH = 2; //Number of byte to read
    private I2cDevice RANGE1;
    private I2cDeviceSynch RANGE1Reader;
    private double mdblRangeSensor1;

    //set up rangesensor 2
    private byte[] range2Cache; //The read will return an array of bytes. They are stored in this variable
    private I2cAddr RANGE2ADDRESS = new I2cAddr(0x18); //Default I2C address for MR Range (7-bit)
    private static final int RANGE2_REG_START = 0x04; //Register to start reading
    private static final int RANGE2_READ_LENGTH = 2; //Number of byte to read
    private I2cDevice RANGE2;
    private I2cDeviceSynch RANGE2Reader;
    private double mdblRangeSensor2;

    //adafruit IMU
    // The IMU sensor object
    private BNO055IMU imu;
    // State used for updating telemetry
    private boolean useAdafruitIMU = false;

    private double mdblTurnAbsoluteGyro;
    private double mdblGyrozAccumulated;
    private int mintStableCount;
    private String mstrWiggleDir;
    private double mdblPowerBoost;
    private int mintPowerBoostCount;

    //vuforia localisation variables
    private OpenGLMatrix lastLocation = null;
    private double localisedRobotX;
    private double localisedRobotY;
    private double localisedRobotBearing;
    private boolean localiseRobotPos;
    private static final int TARGET_WIDTH = 254;
    private static final int TARGET_HEIGHT = 184;

    //define each state for the step.  Each step should go through some of the states below
    // set up the variables for the state engine
    private int mintCurrentStep = 1;                                            // Current Step in State Machine.
    private Constants.stepState mintCurrentStateStep;                           // Current State Machine State.
    private Constants.stepState mintCurrentStateDrive;                          // Current State of Drive.
    private Constants.stepState mintCurrentStateDriveHeading;                   // Current State of Drive Heading.
    private Constants.stepState mintCurrentStateTankTurn;                       // Current State of Tank Turn.
    private Constants.stepState mintCurrentStatePivotTurn;                      // Current State of Pivot Turn.
    private Constants.stepState mintCurrentStateRadiusTurn;                     // Current State of Radius Turn.
    private Constants.stepState mintCurrentStateVuforiaLocalise5291;            // Current State of Vuforia Localisation
    private Constants.stepState mintCurStVuforiaMove5291;                       // Current State of Vuforia Move
    private Constants.stepState mintCurStVuforiaTurn5291;                       // Current State of Vuforia Turn
    private Constants.stepState mintCurrentStateGyroTurnEncoder5291;            // Current State of the Turn function that take the Gyro as an initial heading
    private Constants.stepState mintCurrentStateEyes5291;                       // Current State of the Eyelids
    private Constants.stepState mintCurrentStateTankTurnGyroHeading;            // Current State of Tank Turn using Gyro
    private Constants.stepState mintCurrentStateMecanumStrafe;                  // Current State of mecanum strafe
    private Constants.stepState mintCurrentStepDelay;                           // Current State of Delay (robot doing nothing)
    //private ArrayList<LibraryStateTrack> mValueSteps    = new ArrayList<>();  // Current State of the Step
    private HashMap<String, Integer> mintActiveSteps = new HashMap<>();
    private HashMap<String, Integer> mintActiveStepsCopy = new HashMap<>();

    //motors
    private HardwareDriveMotors robotDrive = new HardwareDriveMotors();   // Use a Pushbot's hardware
    private HardwareDriveMotors armDrive = new HardwareDriveMotors();   // Use a Pushbot's hardware

    //variable for the state engine, declared here so they are accessible throughout the entire opmode with having to pass them through each function
    private double mdblStepTimeout;                          //Timeout value ofthe step, the step will abort if the timeout is reached
    private String mstrRobotCommand;                         //The command the robot will execute, such as move forward, turn right etc
    private double mdblStepDistanceX;                        //used when decoding the step, this will indicate how far the robot is to move in inches
    private double mdblStepDistanceY;                        //used when decoding the step, this will indicate how far the robot is to move in inches
    private double mdblStepDistanceDir;                      //used when decoding the step, this will indicate what direction the robot should move
    private double mdblStepSpeed;                            //When a move command is executed this is the speed the motors will run at
    private boolean mblnUseGyro;                             //used to determine if step should use the Gyro to maintain direction
    private boolean mblnParallel;                            //used to determine if next step will run in parallel - at same time
    private boolean mblnRobotLastPos;                        //used to determine if next step will run from end of last step or from encoder position
    private double mdblRobotParm1;                           //First Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mdblRobotParm2;                           //Second Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mdblRobotParm3;                           //Third Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mdblRobotParm4;                           //Fourth Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mdblRobotParm5;                           //Fifth Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mdblRobotParm6;                           //Sixth Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not

    private boolean mblnReadyToCapture = false;              //Ready to get the camera for capturing images
    private int mintStartPositionLeft1;                      //Left Motor 1  - start position of the robot in inches, starts from 0 to the end
    private int mintStartPositionLeft2;                      //Left Motor 2  - start position of the robot in inches, starts from 0 to the end
    private int mintStartPositionRight1;                     //Right Motor 1 - start position of the robot in inches, starts from 0 to the end
    private int mintStartPositionRight2;                     //Right Motor 2 - start position of the robot in inches, starts from 0 to the end
    private int mintStepLeftTarget1;                         //Left Motor 1   - encoder target position
    private int mintStepLeftTarget2;                         //Left Motor 2   - encoder target position
    private int mintStepRightTarget1;                        //Right Motor 1  - encoder target position
    private int mintStepRightTarget2;                        //Right Motor 2  - encoder target position
    private int mintStartPositionMain;                       //Main Lift Motor - start Position
    private int mintStartPositionTop;                        //Main Lift Motor - start Position
    private int mintTargetPositionMain;                      //Main Lift Motor - start Position
    private int mintTargetPositionTop;                       //Main Lift Motor - start Position
    private double dblStepSpeedTempLeft;
    private double dblStepSpeedTempRight;
    private double mdblStepTurnL;                            //used when decoding the step, this will indicate if the robot is turning left
    private double mdblStepTurnR;                            //used when decoding the step, this will indicate if the robot is turning right
    private double mdblRobotTurnAngle;                       //used to determine angle the robot will turn
    private int mintLastEncoderDestinationLeft1;             //used to store the encoder destination from current Step
    private int mintLastEncoderDestinationLeft2;             //used to store the encoder destination from current Step
    private int mintLastEncoderDestinationRight1;            //used to store the encoder destination from current Step
    private int mintLastEncoderDestinationRight2;            //used to store the encoder destination from current Step
    private boolean mblnNextStepLastPos;                     //used to detect using encoders or previous calc'd position
    private int mintStepDelay;                               //used when decoding the step, this will indicate how long the delay is on ms.
    private boolean mblnDisableVisionProcessing = false;     //used when moving to disable vision to allow faster speed reading encoders.
    private int mintStepRetries = 0;                         //used to count retries on a step
    private ElapsedTime mStateTime = new ElapsedTime();     // Time into current state, used for the timeout
    private int mintStepNumber;
    private int mintGlyphPosition = 0;                       // column the glyph needs to go in, 1 - Left, 2 - Center, 3 - Right
    private boolean flipit = false;
    private int quadrant;


    //hashmap for the steps to be stored in.  A Hashmap is like a fancy array
    //private HashMap<String, LibraryStateSegAutoRoverRuckus> autonomousSteps = new HashMap<String, LibraryStateSegAutoRoverRuckus>();
    private HashMap<String, String> powerTable = new HashMap<String, String>();
    private ReadStepFileRoverRuckus autonomousStepsFile = new ReadStepFileRoverRuckus();

    //OpenCV Stuff
    private JewelAnalysisOCV JewelColour = new JewelAnalysisOCV();

    private int mintCaptureLoop = 0;
    private int mintNumberColourTries = 0;
    private Constants.ObjectColours mColour;

    //servos
    private Servo servoGlyphGripTopLeft;
    private Servo servoGlyphGripBotLeft;
    private Servo servoGlyphGripTopRight;
    private Servo servoGlyphGripBotRight;
    private Servo servoJewelLeft;
    private Servo servoJewelRight;
    private Servo servoRelicFront;
    private Servo servoRelicWrist1;
    private Servo servoRelicWrist2;
    private Servo servoRelicGrip;

    // the servos are on the servo controller
    private final static double SERVOLIFTLEFTTOP_MIN_RANGE      = 0;
    private final static double SERVOLIFTLEFTTOP_MAX_RANGE      = 180;
    private final static double SERVOLIFTLEFTTOP_HOME           = 165; //90
    private final static double SERVOLIFTLEFTTOP_GLYPH_START    = 125;  //need to work this out
    private final static double SERVOLIFTLEFTTOP_GLYPH_RELEASE  = 60;
    private final static double SERVOLIFTLEFTTOP_GLYPH_GRAB     = 30;

    private final static double SERVOLIFTRIGHTTOP_MIN_RANGE     = 0;
    private final static double SERVOLIFTRIGHTTOP_MAX_RANGE     = 180;
    private final static double SERVOLIFTRIGHTTOP_HOME          = 165;
    private final static double SERVOLIFTRIGHTTOP_GLYPH_START   = 125;  //need to work this out
    private final static double SERVOLIFTRIGHTTOP_GLYPH_RELEASE = 60;
    private final static double SERVOLIFTRIGHTTOP_GLYPH_GRAB    = 30;

    private final static double SERVOLIFTLEFTBOT_MIN_RANGE      = 0;
    private final static double SERVOLIFTLEFTBOT_MAX_RANGE      = 180;
    private final static double SERVOLIFTLEFTBOT_HOME           = 165;
    private final static double SERVOLIFTLEFTBOT_GLYPH_START    = 125;  //need to work this out
    private final static double SERVOLIFTLEFTBOT_GLYPH_RELEASE  = 60;
    private final static double SERVOLIFTLEFTBOT_GLYPH_GRAB     = 30;

    private final static double SERVOLIFTRIGHTBOT_MIN_RANGE     = 0;
    private final static double SERVOLIFTRIGHTBOT_MAX_RANGE     = 180;
    private final static double SERVOLIFTRIGHTBOT_HOME          = 165;
    private final static double SERVOLIFTRIGHTBOT_GLYPH_START   = 125;  //need to work this out
    private final static double SERVOLIFTRIGHTBOT_GLYPH_RELEASE = 60;
    private final static double SERVOLIFTRIGHTBOT_GLYPH_GRAB    = 30;

    private final static double SERVOJEWELLEFT_MIN_RANGE        = 0;
    private final static double SERVOJEWELLEFT_MAX_RANGE        = 180;
    private final static double SERVOJEWELLEFT_HOME             = 143;
    private final static double SERVOJEWELRIGHT_MIN_RANGE       = 4;
    private final static double SERVOJEWELRIGHT_MAX_RANGE       = 180;
    private final static double SERVOJEWELRIGHT_HOME            = 149;

    private final static double SERVORELICFRONT_MIN_RANGE       = 0;
    private final static double SERVORELICFRONT_MAX_RANGE       = 180;
    private final static double SERVORELICFRONT_HOME            = 0;
    private final static double SERVORELICWRIST_MIN_RANGE       = 0;
    private final static double SERVORELICWRIST_MAX_RANGE       = 180;
    private final static double SERVORELICWRIST_HOME            = 180;
    private final static double SERVORELICGRIP_MIN_RANGE        = 0;
    private final static double SERVORELICGRIP_MAX_RANGE        = 180;
    private final static double SERVORELICGRIP_HOME             = 90;   //Closed is position 90

    //LED Strips
    private TOWR5291LEDControl LEDs;
    private Constants.LEDState mint5291LEDStatus;                                                   // Flash the LED based on the status

    //Limit Switches
    DigitalChannel limitswitch1;  // Hardware Device Object
    DigitalChannel limitswitch2;  // Hardware Device Object

    private static HalDashboard dashboard = null;

    public static HalDashboard getDashboard() {
        return dashboard;
    }

    //each robot speeds up and slows down at different rates
    //helps reduce over runs and
    //table for the tilerunner from AndyMark.  These values are for the twin 20 motors which makes the robot fast
    private void loadPowerTableTileRunner() {
        powerTable.put(String.valueOf(0.5), ".1");
        powerTable.put(String.valueOf(1), ".2");
        powerTable.put(String.valueOf(2), ".3");
        powerTable.put(String.valueOf(4), ".4");
        powerTable.put(String.valueOf(6), ".5");
        powerTable.put(String.valueOf(8), ".6");
        powerTable.put(String.valueOf(10), ".7");
        powerTable.put(String.valueOf(12), ".8");
    }

    //table for the custom tanktread robot.  These values are for the twin 40 motors
    private void loadPowerTableTankTread() {
        powerTable.put(String.valueOf(0.5), ".3");
        powerTable.put(String.valueOf(1), ".3");
        powerTable.put(String.valueOf(2), ".4");
        powerTable.put(String.valueOf(4), ".5");
        powerTable.put(String.valueOf(6), ".5");
        powerTable.put(String.valueOf(8), ".6");
        powerTable.put(String.valueOf(10), ".6");
        powerTable.put(String.valueOf(12), ".6");
        powerTable.put(String.valueOf(15), ".8");
    }

    @Override
    public void runOpMode() throws InterruptedException {

        dashboard = HalDashboard.createInstance(telemetry);
        dashboard = HalDashboard.getInstance();

        FtcRobotControllerActivity activity = (FtcRobotControllerActivity) hardwareMap.appContext;

        dashboard.setTextView((TextView) activity.findViewById(R.id.textOpMode));
        dashboard.displayPrintf(0, LABEL_WIDTH, "Text: ", "*** Robot Data ***");
        //start the logging

        //create logging based on initial settings, sharepreferences will adjust levels
        fileLogger = new FileLogger(runtime, 1, true);
        fileLogger.open();
        fileLogger.write("Time,SysMS,Thread,Event,Desc");
        fileLogger.setEventTag("runOpMode()");
        fileLogger.writeEvent("Log Started");
        runtime.reset();
        dashboard.displayPrintf(1, "FileLogger: Started");

        //init openCV
        initOpenCv();
        dashboard.displayPrintf(1, "initRobot OpenCV!");
        fileLogger.writeEvent(3, "OpenCV Started");

        //load menu settings and setup robot and debug level
        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        ourRobotConfig = new robotConfig();
        ourRobotConfig.setTeamNumber(sharedPreferences.getString("club.towr5291.Autonomous.TeamNumber", "0000"));
        ourRobotConfig.setAllianceColor(sharedPreferences.getString("club.towr5291.Autonomous.Color", "Red"));
        ourRobotConfig.setAllianceStartPosition(sharedPreferences.getString("club.towr5291.Autonomous.StartPosition", "Left"));
        ourRobotConfig.setDelay(Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Delay", "0")));
        ourRobotConfig.setRobotConfig(sharedPreferences.getString("club.towr5291.Autonomous.RobotConfig", "TileRunnerMecanum2x40"));
        debug = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", "1"));

        //now we have loaded the config from sharedpreferences we can setup the robot
        ourRobotConfig.initConfig();

        //adjust debug level based on saved settings
        fileLogger.setDebugLevel(debug);

        fileLogger.writeEvent(1, "robotConfigTeam #  " + ourRobotConfig.getTeamNumber());
        fileLogger.writeEvent(1, "Alliance Colour    " + ourRobotConfig.getAllianceColor());
        fileLogger.writeEvent(1, "Alliance Start Pos " + ourRobotConfig.getAllianceStartPosition());
        fileLogger.writeEvent(1, "Alliance Delay     " + ourRobotConfig.getDelay());
        fileLogger.writeEvent(1, "Robot Config       " + ourRobotConfig.getRobotConfig());
        fileLogger.writeEvent(3, "Configuring Robot Parameters - Finished");
        fileLogger.writeEvent(3, "Loading Autonomous Steps - Start");

        dashboard.displayPrintf(1, "initRobot Loading Steps " + ourRobotConfig.getAllianceColor() + " Team " + ourRobotConfig.getTeamNumber());

        dashboard.displayPrintf(3, "robotConfigTeam # " + ourRobotConfig.getTeamNumber());
        dashboard.displayPrintf(4, "Alliance          " + ourRobotConfig.getAllianceColor());
        dashboard.displayPrintf(5, "Start Pos         " + ourRobotConfig.getAllianceStartPosition());
        dashboard.displayPrintf(6, "Start Del         " + ourRobotConfig.getDelay());
        dashboard.displayPrintf(7, "Robot             " + ourRobotConfig.getRobotConfig());
        dashboard.displayPrintf(7, "Debug Level       " + debug);
        dashboard.displayPrintf(1, "initRobot SharePreferences!");

        // Set up the LEDS
        LEDs = new TOWR5291LEDControl(hardwareMap, "lg", "lr", "lb", "rg", "rr", "rb");
        LEDs.setLEDControlDemoMode(false);
        LEDs.setLEDColour(Constants.LEDColours.LED_MAGENTA);

        dashboard.displayPrintf(1, "initRobot LED Initiated!");

        // get a reference to our digitalTouch object.
        limitswitch1 = hardwareMap.get(DigitalChannel.class, "limittop");
        limitswitch2 = hardwareMap.get(DigitalChannel.class, "limitbot");
        // set the digital channel to input.
        limitswitch1.setMode(DigitalChannel.Mode.INPUT);
        limitswitch2.setMode(DigitalChannel.Mode.INPUT);

        dashboard.displayPrintf(1, "initRobot Limit Switch Initiated!");
        //load the sequence based on alliance colour and team

        autonomousStepsFile.ReadStepFile(ourRobotConfig);

        //need to load initial step of a delay based on user input
        autonomousStepsFile.insertSteps(ourRobotConfig.getDelay() + 1, "DELAY" ,0,0,0,0, false,false, false,  (ourRobotConfig.getDelay() * 1000), 0, 0, 0, 0, 0,1);

        dashboard.displayPrintf(1, "initRobot STEPS LOADED");

        fileLogger.writeEvent(3, "Loading Autonomous Steps - Finished");
        fileLogger.writeEvent(3, "Configuring Adafruit IMU - Start");

        dashboard.displayPrintf(1, "initRobot IMU Loading");

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

        dashboard.displayPrintf(1, "initRobot IMU Configured");

        fileLogger.writeEvent(3,"Configuring Adafruit IMU - Finished");
        fileLogger.writeEvent(3,"Configuring Motors Base - Start");

        dashboard.displayPrintf(1, "initRobot BaseDrive Loading");

        robotDrive.init(fileLogger, hardwareMap, robotConfigSettings.robotConfigChoice.valueOf(ourRobotConfig.getRobotConfig()));
        robotDrive.setHardwareDriveResetEncoders();
        robotDrive.setHardwareDriveRunUsingEncoders();

        dashboard.displayPrintf(1, "initRobot BaseDrive Loaded");

        fileLogger.writeEvent(3,"Configuring Motors Base - Finish");
        fileLogger.writeEvent(3,"Configuring Motors Arms - Start");

        armDrive.init(fileLogger, hardwareMap, robotConfigSettings.robotConfigChoice.valueOf(ourRobotConfig.getRobotConfig()), "lifttop", "liftbot", null, null);
        armDrive.setHardwareDriveResetEncoders();
        armDrive.setHardwareDriveRunUsingEncoders();

        fileLogger.writeEvent(3,"Configuring Motors Lifts - Finish");
        fileLogger.writeEvent(3, "Configuring Range Sensors - Start");

        dashboard.displayPrintf(1, "initRobot Range Sensors Loading");

        RANGE1 = hardwareMap.i2cDevice.get("range1");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();
        RANGE2 = hardwareMap.i2cDevice.get("range2");
        RANGE2Reader = new I2cDeviceSynchImpl(RANGE2, RANGE2ADDRESS, false);
        RANGE2Reader.engage();

        dashboard.displayPrintf(1, "initRobot Range Sensors Loaded");

        fileLogger.writeEvent(3,"Configuring Range Sensors - Finish");
        fileLogger.writeEvent(3,"Resetting State Engine - Start");

        initDefaultStates();

        mint5291LEDStatus = Constants.LEDState.STATE_TEAM;
        mblnNextStepLastPos = false;

        fileLogger.writeEvent(3,"Resetting State Engine - Finish");
        fileLogger.writeEvent(3,"Configuring Vuforia - Start");

        dashboard.displayPrintf(1, "initRobot VUFORIA Loading");

        //load all the vuforia stuff
        LibraryVuforiaRelicRecovery RelicRecoveryVuforia = new LibraryVuforiaRelicRecovery();
        VuforiaTrackables RelicRecoveryTrackables;
        RelicRecoveryTrackables = RelicRecoveryVuforia.LibraryVuforiaRelicRecovery(hardwareMap, ourRobotConfig);

        //activate vuforia
        RelicRecoveryTrackables.activate();

        //vuMark will be the position to load the glyph
        RelicRecoveryVuMark vuMark;

        //set up variable for our capturedimage
        Image rgb = null;

        dashboard.displayPrintf(1, "initRobot VUFORIA Loaded");
        fileLogger.writeEvent(3,"Configuring Servos - Start");

        //config the servos
        servoGlyphGripTopLeft = hardwareMap.servo.get("griptopleft");
        servoGlyphGripBotLeft = hardwareMap.servo.get("gripbotleft");
        servoGlyphGripTopLeft.setDirection(Servo.Direction.REVERSE);
        servoGlyphGripBotLeft.setDirection(Servo.Direction.REVERSE);
        servoGlyphGripTopRight = hardwareMap.servo.get("griptopright");
        servoGlyphGripBotRight = hardwareMap.servo.get("gripbotright");
        servoJewelLeft = hardwareMap.servo.get("jewelleft");
        servoJewelRight = hardwareMap.servo.get("jewelright");
        servoRelicFront = hardwareMap.servo.get("relicfront");
        servoRelicWrist1 = hardwareMap.servo.get("relicwrist1");
        servoRelicWrist2 = hardwareMap.servo.get("relicwrist2");
        servoRelicWrist2.setDirection(Servo.Direction.REVERSE);
        servoRelicGrip = hardwareMap.servo.get("relicgrip");

        //lock the jewel arms home
        sendServosHome(servoGlyphGripTopLeft, servoGlyphGripBotLeft, servoGlyphGripTopRight, servoGlyphGripBotRight, servoJewelLeft, servoJewelRight);
        fileLogger.writeEvent(3,"Configuring Servos - Finish");
        //lock the relic servos in fetal position
        moveServo(servoRelicFront,SERVORELICFRONT_HOME,SERVORELICFRONT_MIN_RANGE,SERVORELICFRONT_MAX_RANGE);
        moveServo(servoRelicWrist1,SERVORELICWRIST_HOME,SERVORELICWRIST_MIN_RANGE,SERVORELICWRIST_MAX_RANGE);
        moveServo(servoRelicWrist2,SERVORELICWRIST_HOME,SERVORELICWRIST_MIN_RANGE,SERVORELICWRIST_MAX_RANGE);
        moveServo(servoRelicGrip,SERVORELICGRIP_HOME,SERVORELICGRIP_MIN_RANGE,SERVORELICGRIP_MAX_RANGE);

        dashboard.displayPrintf(1, "initRobot Servos Loaded");

        // get a reference to our digitalTouch object.
        limitswitch1 = hardwareMap.get(DigitalChannel.class, "limittop");
        limitswitch2 = hardwareMap.get(DigitalChannel.class, "limitbot");

        // set the digital channel to input.
        limitswitch1.setMode(DigitalChannel.Mode.INPUT);
        limitswitch2.setMode(DigitalChannel.Mode.INPUT);

        fileLogger.writeEvent(1,"Set Limit Switches");

        dashboard.displayPrintf(1, "initRobot Limit Switches Configured");

        Mat tmp = new Mat();

        fileLogger.writeEvent(3,"Configuring Vuforia - Finished");
        fileLogger.writeEvent(3,"Configuring Robot Parameters - Start");
        dashboard.displayPrintf(1, "Init - Complete, Wait for Start");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        fileLogger.setEventTag("opModeIsActive()");
        dashboard.clearDisplay();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //the main loop.  this is where the action happens
        while (opModeIsActive()) {
            if (!mblnDisableVisionProcessing) {
                //start capturing frames for analysis
                if (mblnReadyToCapture) {
                    //vuMark will be the position to load the glyph
                    vuMark = RelicRecoveryVuMark.from(RelicRecoveryVuforia.getRelicTemplate());

                    tmp = new Mat();
                    try {
                        VuforiaLocalizer.CloseableFrame frame = RelicRecoveryVuforia.getVuforia().getFrameQueue().take(); //takes the frame at the head of the queue
                        long numImages = frame.getNumImages();

                        for (int i = 0; i < numImages; i++) {
                            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                                rgb = frame.getImage(i);
                                break;
                            }
                        }

                        /*rgb is now the Image object that we’ve used in the video*/
                        Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
                        bm.copyPixelsFromBuffer(rgb.getPixels());

                        //put the image into a MAT for OpenCV
                        tmp = new Mat(rgb.getWidth(), rgb.getHeight(), CvType.CV_8UC4);
                        Utils.bitmapToMat(bm, tmp);
                        //close the frame, prevents memory leaks and crashing
                        frame.close();
                    } catch (InterruptedException e) {
                        dashboard.displayPrintf(1, "VUFORIA --- ERROR ERROR ERROR");
                        dashboard.displayPrintf(2, "VUFORIA --- ERROR ERROR ERROR");
                        LEDs.LEDControlUpdate(Constants.LEDState.STATE_ERROR);
                        fileLogger.writeEvent(3,"Init Colour Returned " + mColour + " Column " + vuMark.toString());
                    }

                    if (ourRobotConfig.getAllianceColor().equals("Blue")) {
                        flipit = false;
                        quadrant = 3;
                    }
                    else {
                        flipit = false;
                        quadrant = 3;
                    }

                    mColour = JewelColour.JewelAnalysisOCV(fileLogger, tmp, mintCaptureLoop, flipit, quadrant, false);
                    fileLogger.writeEvent(3,"Colour Returned " + mColour + " Column " + vuMark.toString());
                    dashboard.displayPrintf(2, "Jewel Colour-" + mColour + " Column-" + vuMark.toString());
                    switch (vuMark) {
                        case LEFT:
                            mintGlyphPosition = 1;
                            break;
                        case CENTER:
                            mintGlyphPosition = 2;
                            break;
                        case RIGHT:
                            mintGlyphPosition = 3;
                            break;
                        default:
                            mintGlyphPosition = 0;
                            break;
                    }
                    mintCaptureLoop++;
                }

                //use vuforia to get location information
                for (VuforiaTrackable trackable : RelicRecoveryVuforia.getAllTrackables()) {
                    /**
                     * getUpdatedRobotLocation() will return null if no new information is available since
                     * the last time that call was made, or if the trackable is not currently visible.
                     * getRobotLocation() will return null if the trackable is not currently visible.
                     */
                    dashboard.displayPrintf(1, LABEL_WIDTH, trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                }
                /**
                 * Provide feedback as to where the robot was last located (if we know).
                 */
                if (lastLocation != null) {
                    // Then you can extract the positions and angles using the getTranslation and getOrientation methods.
                    VectorF trans = lastLocation.getTranslation();
                    Orientation rot = Orientation.getOrientation(lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                    // Robot position is defined by the standard Matrix translation (x and y)
                    localisedRobotX = trans.get(0);
                    localisedRobotY = trans.get(1);
                    // Robot bearing (in Cartesian system) position is defined by the standard Matrix z rotation
                    localisedRobotBearing = rot.thirdAngle;
                    if (localisedRobotBearing < 0) {
                        localisedRobotBearing = 360 + localisedRobotBearing;
                    }
                    dashboard.displayPrintf(3, "Pos X " + localisedRobotX);
                    dashboard.displayPrintf(4, "Pos Y ", localisedRobotY);
                    dashboard.displayPrintf(5, "Bear  ", localisedRobotBearing);
                    dashboard.displayPrintf(6, "Pos   ", format(lastLocation));
                    localiseRobotPos = true;
                } else {
                    localiseRobotPos = false;
                    dashboard.displayPrintf(3, "Pos   ", "Unknown");
                }
            }

            switch (mintCurrentStateStep) {
                case STATE_INIT:
                    fileLogger.writeEvent(1,"mintCurrentStateStep:- " + mintCurrentStateStep + " mintCurrentStateStep " + mintCurrentStateStep);
                    fileLogger.writeEvent(1,"About to check if step exists " + mintCurrentStep);

                    // get step from hashmap, send it to the initStep for decoding
                    if (autonomousStepsFile.activeSteps().containsKey(String.valueOf(mintCurrentStep))) {
                        fileLogger.writeEvent(1,"Step Exists TRUE " + mintCurrentStep + " about to get the values from the step");
                        initStep();
                    } else {
                        mintCurrentStateStep = Constants.stepState.STATE_FINISHED;
                    }
                    break;
                case STATE_START:

                    break;
                case STATE_RUNNING:

                    //load all the parallel steps so they can be evaluated for completeness
                    loadParallelSteps();

                    //Process all the parallel steps
                    for (String stKey : mintActiveStepsCopy.keySet()) {
                        fileLogger.writeEvent(1, "STATE_RUNNING", "Looping through Parallel steps, found " + stKey);
                        mintStepNumber = mintActiveStepsCopy.get(stKey);
                        loadActiveStep(mintStepNumber);
                        fileLogger.writeEvent(1, "STATE_RUNNING", "About to run " + mstrRobotCommand.substring(0, 3));
                        processSteps(mstrRobotCommand.substring(0, 3));
                    }

                    //Check the status of all the steps if all the states are complete we can move to the next state
                    if (checkAllStatesComplete()) {
                        mintCurrentStateStep = Constants.stepState.STATE_COMPLETE;
                    }

                    //make sure we load the current step to determine if parallel, if the steps are run out of order and a previous step was parallel
                    //things get all messed up and a step that isn't parallel can be assumed to be parallel
                    loadActiveStep(mintCurrentStep);
                    if (mblnParallel) {
                        // mark this step as complete and do next one, the current step should continue to run.  Not all steps are compatible with being run in parallel
                        // like drive steps, turns etc
                        // Drive forward and shoot
                        // Drive forward and detect beacon
                        // are examples of when parallel steps should be run
                        // errors will occur if other combinations are run
                        // only go to next step if current step equals the one being processed for parallelism.
                        for (String stKey : mintActiveStepsCopy.keySet()) {
                            mintStepNumber = mintActiveStepsCopy.get(stKey);
                            if (mintCurrentStep == mintStepNumber)
                                mintCurrentStateStep = Constants.stepState.STATE_COMPLETE;
                        }
                    }
                    break;
                case STATE_PAUSE:
                    break;
                case STATE_COMPLETE:
                    fileLogger.writeEvent(1,"Step Complete - Current Step:- " + mintCurrentStep);
                    //  Transition to a new state and next step.
                    mintCurrentStep++;
                    mintCurrentStateStep = Constants.stepState.STATE_INIT;
                    break;
                case STATE_TIMEOUT:
                    robotDrive.setHardwareDrivePower(0);
                    //  Transition to a new state.
                    mintCurrentStateStep = Constants.stepState.STATE_FINISHED;
                    break;
                case STATE_ERROR:
                    LEDs.LEDControlUpdate(Constants.LEDState.STATE_ERROR);
                    dashboard.displayPrintf(2, "STATE", "ERROR WAITING TO FINISH " + mintCurrentStep);
                    break;
                case STATE_FINISHED:
                    LEDs.LEDControlUpdate(Constants.LEDState.STATE_FINISHED);
                    robotDrive.setHardwareDrivePower(0);
                    //stop the logging
                    if (fileLogger != null) {
                        fileLogger.writeEvent(1, "Step FINISHED - FINISHED");
                        fileLogger.writeEvent(1, "Stopped");
                        Log.d("END:-", "FileLogger Stopped");
                        fileLogger.close();
                        fileLogger = null;
                    }
                    //deactivate vuforia
                    RelicRecoveryTrackables.deactivate();
                    dashboard.displayPrintf(1, "STATE", "FINISHED " + mintCurrentStep);
                    break;

            }

            //Update the LEDs based on the system changes
            mint5291LEDStatus = LEDs.LEDControlUpdate(mint5291LEDStatus);

        }
        if (fileLogger != null) {
            fileLogger.writeEvent(1, "FINISHED AUTON - TIMED OUT");
            Log.d("END:-", "FINISHED AUTON - TIMED OUT - logger stopped");
            fileLogger.close();
            fileLogger = null;
        }

        //switch opmode to teleop
        opModeManager = (OpModeManagerImpl) onStop.internalOpModeServices;
        opModeManager.initActiveOpMode(TeleOpMode);
        //opmode not active anymore
    }

    private void loadActiveStep(int step) {
        fileLogger.setEventTag("loadActiveStep()");
        LibraryStateSegAutoRoverRuckus mStateSegAuto = autonomousStepsFile.activeSteps().get(String.valueOf(step));
        fileLogger.writeEvent(1,"Got the values for step " + step + " about to decode");
        mdblStepTimeout = mStateSegAuto.getmRobotTimeOut();
        mstrRobotCommand = mStateSegAuto.getmRobotCommand();
        mdblStepDistanceX = mStateSegAuto.getmRobotDistanceX();
        mdblStepDistanceY = mStateSegAuto.getmRobotDistanceY();
        mdblStepDistanceDir = mStateSegAuto.getmRobotDirection();
        mdblStepSpeed = mStateSegAuto.mRobotSpeed();
        mblnUseGyro = mStateSegAuto.ismGyroDrive();
        mblnParallel = mStateSegAuto.getmRobotParallel();
        mblnRobotLastPos = mStateSegAuto.getmRobotLastPos();
        mdblRobotParm1 = mStateSegAuto.mRobotParm1();
        mdblRobotParm2 = mStateSegAuto.mRobotParm2();
        mdblRobotParm3 = mStateSegAuto.mRobotParm3();
        mdblRobotParm4 = mStateSegAuto.mRobotParm4();
        mdblRobotParm5 = mStateSegAuto.mRobotParm5();
        mdblRobotParm6 = mStateSegAuto.mRobotParm6();
    }

    private void loadParallelSteps() {
        fileLogger.setEventTag("loadParallelSteps()");
        mintActiveStepsCopy.clear();
        for (String stKey : mintActiveSteps.keySet()) {
            fileLogger.writeEvent(2,"Loading Active Parallel Step " + stKey);
            mintActiveStepsCopy.put(stKey, mintActiveSteps.get(stKey));
        }
    }

    private void deleteParallelStep() {
        fileLogger.setEventTag("deleteParallelStep()");
        for (String stKey : mintActiveStepsCopy.keySet()) {
            int tempStep = mintActiveStepsCopy.get(stKey);
            if (mintStepNumber == tempStep) {
                fileLogger.writeEvent(2,"Removing Parallel Step " + tempStep);
                if (mintActiveSteps.containsKey(stKey))
                    mintActiveSteps.remove(stKey);
            }
        }
    }

    private void processSteps(String stepName) {
        fileLogger.setEventTag("processSteps()");

        switch (stepName) {
            case "DELAY":
                DelayStep();
                break;
            case "LTE":
            case "RTE":
                TankTurnStep();
                break;
            case "GTH":
                TankTurnGyroHeading();
                break;
            case "GTE":  // Special Function, 5291 Move forward until line is found
                TankTurnGyroHeadingEncoder();
                break;
            case "LPE":
            case "RPE":
                PivotTurnStep();
                break;
            case "RRE":  // Right turn with a Radius in Parm 1
            case "LRE":  // Left turn with a Radius in Parm 1
                RadiusTurnStep();
                break;
            case "FWE":  // Drive forward a distance in inches and power setting
                DriveStepHeading();
                break;
            case "VFL":  // Position the robot using vuforia parameters ready fro AStar  RObot should postion pointing to Red wall and Blue wall where targets are located
                VuforiaLocalise();
                break;
            case "VME":  // Move the robot using localisation from the targets
                VuforiaMove();
                break;
            case "VTE":  // Turn the Robot using information from Vuforia and Pythag
                VuforiaTurn();
                break;
        }
    }

    //--------------------------------------------------------------------------
    //  Initialise the state.
    //--------------------------------------------------------------------------
    //private void initStep (LibraryStateSegAuto mStateSegAuto) {
    private void initStep() {
        fileLogger.setEventTag("initStep()");
        fileLogger.writeEvent(3,"Starting to Decode Step " + mintCurrentStep);

        if (!(mintActiveSteps.containsValue(mintCurrentStep))) {
            mintActiveSteps.put(String.valueOf(mintCurrentStep), mintCurrentStep);
            fileLogger.writeEvent(3,"Put step into hashmap mintActiveSteps " + mintCurrentStep);
        }

        loadActiveStep(mintCurrentStep);
        mintCurrentStateStep = Constants.stepState.STATE_RUNNING;
        // Reset the state time, and then change to next state.
        mStateTime.reset();

        switch (mstrRobotCommand.substring(0, 3)) {
            case "DELAY":
                mintCurrentStepDelay = Constants.stepState.STATE_INIT;
                break;
            case "GTH":
                mintCurrentStateTankTurnGyroHeading = Constants.stepState.STATE_INIT;
                break;
            case "MST":
                mintCurrentStateMecanumStrafe = Constants.stepState.STATE_INIT;
                break;
            case "LTE":
                mintCurrentStateTankTurn = Constants.stepState.STATE_INIT;
                break;
            case "RTE":
                mintCurrentStateTankTurn = Constants.stepState.STATE_INIT;
                break;
            case "LPE":
                mintCurrentStatePivotTurn = Constants.stepState.STATE_INIT;
                break;
            case "RPE":
                mintCurrentStatePivotTurn = Constants.stepState.STATE_INIT;
                break;
            case "LRE":  // Left turn with a Radius in Parm 1
                mintCurrentStateRadiusTurn = Constants.stepState.STATE_INIT;
                break;
            case "RRE":  // Right turn with a Radius in Parm 1
                mintCurrentStateRadiusTurn = Constants.stepState.STATE_INIT;
                break;
            case "FWE":  // Drive forward a distance in inches and power setting
                mintCurrentStateDriveHeading = Constants.stepState.STATE_INIT;
                break;
            case "VFL":  // Position the robot using vuforia parameters ready fro AStar  RObot should postion pointing to Red wall and Blue wall where targets are located
                mintCurrentStateVuforiaLocalise5291 = Constants.stepState.STATE_INIT;
                break;
            case "VME":  // Move the robot using localisation from the targets
                mintCurStVuforiaMove5291 = Constants.stepState.STATE_INIT;
                break;
            case "VTE":  // Turn the Robot using information from Vuforia and Pythag
                mintCurStVuforiaTurn5291 = Constants.stepState.STATE_INIT;
                break;
            case "GTE":  // Special Function, 5291 Move forward until line is found
                mintCurrentStateGyroTurnEncoder5291 = Constants.stepState.STATE_INIT;
                break;
            case "EYE":  // Special Function, 5291 Move forward until line is found
                mintCurrentStateEyes5291 = Constants.stepState.STATE_INIT;
                break;
            case "FNC":  //  Run a special Function with Parms

                break;
        }

        fileLogger.writeEvent(2,"Current Step          :- " + mintCurrentStep);
        fileLogger.writeEvent(2,"mdblStepTimeout       :- " + mdblStepTimeout);
        fileLogger.writeEvent(2,"mstrRobotCommand      :- " + mstrRobotCommand);
        fileLogger.writeEvent(2,"mdblStepDistanceX     :- " + mdblStepDistanceX);
        fileLogger.writeEvent(2,"mdblStepDistanceY     :- " + mdblStepDistanceY);
        fileLogger.writeEvent(2,"mdblStepDistanceDir   :- " + mdblStepDistanceDir);
        fileLogger.writeEvent(2,"mdblStepSpeed         :- " + mdblStepSpeed);
        fileLogger.writeEvent(2,"mblnUseGyro           :- " + mblnUseGyro);
        fileLogger.writeEvent(2,"mblnParallel          :- " + mblnParallel);
        fileLogger.writeEvent(2,"mblnRobotLastPos      :- " + mblnRobotLastPos);
        fileLogger.writeEvent(2,"mdblRobotParm1        :- " + mdblRobotParm1);
        fileLogger.writeEvent(2,"mdblRobotParm2        :- " + mdblRobotParm2);
        fileLogger.writeEvent(2,"mdblRobotParm3        :- " + mdblRobotParm3);
        fileLogger.writeEvent(2,"mdblRobotParm4        :- " + mdblRobotParm4);
        fileLogger.writeEvent(2,"mdblRobotParm5        :- " + mdblRobotParm5);
        fileLogger.writeEvent(2,"mdblRobotParm6        :- " + mdblRobotParm6);
    }

    private void DriveStepHeading() {
        fileLogger.setEventTag("DriveStepHeading()");

        double dblDistanceToEndLeft1;
        double dblDistanceToEndLeft2;
        double dblDistanceToEndRight1;
        double dblDistanceToEndRight2;
        double dblDistanceToEnd;
        double dblDistanceFromStartLeft1;
        double dblDistanceFromStartLeft2;
        double dblDistanceFromStartRight1;
        double dblDistanceFromStartRight2;
        double dblDistanceFromStart;
        int intLeft1MotorEncoderPosition;
        int intLeft2MotorEncoderPosition;
        int intRight1MotorEncoderPosition;
        int intRight2MotorEncoderPosition;
        double dblMaxSpeed;
        double dblError;
        double dblSteer;
        double dblLeftSpeed;
        double dblRightSpeed;

        switch (mintCurrentStateDriveHeading) {
            case STATE_INIT:

                // set motor controller to mode
                robotDrive.setHardwareDriveRunUsingEncoders();
                mblnDisableVisionProcessing = true;  //disable vision processing
                mdblStepDistanceX = Double.parseDouble(mstrRobotCommand.substring(3));
                fileLogger.writeEvent(2,"mdblStepDistanceX   :- " + mdblStepDistanceX);
                // Determine new target position

                if (mblnNextStepLastPos) {
                    mintStartPositionLeft1 = mintLastEncoderDestinationLeft1;
                    mintStartPositionLeft2 = mintLastEncoderDestinationLeft2;
                    mintStartPositionRight1 = mintLastEncoderDestinationRight1;
                    mintStartPositionRight2 = mintLastEncoderDestinationRight2;
                } else {
                    mintStartPositionLeft1 = robotDrive.baseMotor1.getCurrentPosition();
                    mintStartPositionLeft2 = robotDrive.baseMotor2.getCurrentPosition();
                    mintStartPositionRight1 = robotDrive.baseMotor3.getCurrentPosition();
                    mintStartPositionRight2 = robotDrive.baseMotor4.getCurrentPosition();
                }
                mblnNextStepLastPos = false;

                mintStepLeftTarget1 = mintStartPositionLeft1 + (int) (mdblStepDistanceX * ourRobotConfig.getCOUNTS_PER_INCH());
                mintStepLeftTarget2 = mintStartPositionLeft2 + (int) (mdblStepDistanceX * ourRobotConfig.getCOUNTS_PER_INCH());
                mintStepRightTarget1 = mintStartPositionRight1 + (int) (mdblStepDistanceX * ourRobotConfig.getCOUNTS_PER_INCH());
                mintStepRightTarget2 = mintStartPositionRight2 + (int) (mdblStepDistanceX * ourRobotConfig.getCOUNTS_PER_INCH());

                //store the encoder positions so next step can calculate destination
                mintLastEncoderDestinationLeft1 = mintStepLeftTarget1;
                mintLastEncoderDestinationLeft2 = mintStepLeftTarget2;
                mintLastEncoderDestinationRight1 = mintStepRightTarget1;
                mintLastEncoderDestinationRight2 = mintStepRightTarget2;

                // pass target position to motor controller
                robotDrive.baseMotor1.setTargetPosition(mintStepLeftTarget1);
                robotDrive.baseMotor2.setTargetPosition(mintStepLeftTarget2);
                robotDrive.baseMotor3.setTargetPosition(mintStepRightTarget1);
                robotDrive.baseMotor4.setTargetPosition(mintStepRightTarget2);

                fileLogger.writeEvent(2,"mStepLeftTarget1 :- " + mintStepLeftTarget1 + " mStepLeftTarget2 :- " + mintStepLeftTarget2);
                fileLogger.writeEvent(2,"mStepRightTarget1:- " + mintStepRightTarget1 + " mStepRightTarget2:- " + mintStepRightTarget2);

                if (!(robotDrive.baseMotor1.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION))) {
                    // set motor controller to mode, Turn On RUN_TO_POSITION
                    robotDrive.setHardwareDriveRunToPosition();
                }

                mintCurrentStateDriveHeading = Constants.stepState.STATE_RUNNING;
                robotDrive.setHardwareDrivePower(Math.abs(mdblStepSpeed));

                dblStepSpeedTempRight = mdblStepSpeed;
                dblStepSpeedTempLeft = mdblStepSpeed;
                break;
            case STATE_RUNNING:
                // pass target position to motor controller
                robotDrive.baseMotor1.setTargetPosition(mintStepLeftTarget1);
                robotDrive.baseMotor2.setTargetPosition(mintStepLeftTarget2);
                robotDrive.baseMotor3.setTargetPosition(mintStepRightTarget1);
                robotDrive.baseMotor4.setTargetPosition(mintStepRightTarget2);

                robotDrive.setHardwareDrivePower(Math.abs(mdblStepSpeed));

                intLeft1MotorEncoderPosition = robotDrive.baseMotor1.getCurrentPosition();
                intLeft2MotorEncoderPosition = robotDrive.baseMotor2.getCurrentPosition();
                intRight1MotorEncoderPosition = robotDrive.baseMotor3.getCurrentPosition();
                intRight2MotorEncoderPosition = robotDrive.baseMotor4.getCurrentPosition();

                // ramp up speed - need to write function to ramp up speed
                dblDistanceFromStartLeft1 = Math.abs(mintStartPositionLeft1 - intLeft1MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();
                dblDistanceFromStartLeft2 = Math.abs(mintStartPositionLeft2 - intLeft2MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();
                dblDistanceFromStartRight1 = Math.abs(mintStartPositionRight1 - intRight1MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();
                dblDistanceFromStartRight2 = Math.abs(mintStartPositionRight2 - intRight2MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();

                //if moving ramp up
                dblDistanceFromStart = (dblDistanceFromStartLeft1 + dblDistanceFromStartRight1 + dblDistanceFromStartLeft2 + dblDistanceFromStartRight2) / 4;

                //determine how close to target we are
                dblDistanceToEndLeft1 = (mintStepLeftTarget1 - intLeft1MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();
                dblDistanceToEndLeft2 = (mintStepLeftTarget2 - intLeft2MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();
                dblDistanceToEndRight1 = (mintStepRightTarget1 - intRight1MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();
                dblDistanceToEndRight2 = (mintStepRightTarget2 - intRight2MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();

                //if getting close ramp down speed
                dblDistanceToEnd = (dblDistanceToEndLeft1 + dblDistanceToEndRight1 + dblDistanceToEndLeft2 + dblDistanceToEndRight2) / 4;

                //parameter 1 or 4 is use gyro for direction,  setting either of these to 1 will get gyro correction
                // if parameter 1 is true
                // parameter 2 is the error
                // parameter 3 is the gain coefficient
                if ((mblnUseGyro)) {
                    //use Gyro to run heading
                    // adjust relative speed based on heading error.
                    dblError = getDriveError(mdblRobotParm1);
                    dblSteer = getDriveSteer(dblError, mdblRobotParm1);
                    fileLogger.writeEvent(3,"dblError " + dblError);
                    fileLogger.writeEvent(3,"dblSteer " + dblSteer);
                    fileLogger.writeEvent(3,"Heading " + mdblStepDistanceDir);

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (mdblStepDistanceX < 0)
                        dblSteer *= -1.0;

                    dblStepSpeedTempLeft = dblStepSpeedTempLeft - dblSteer;
                    dblStepSpeedTempRight = dblStepSpeedTempRight + dblSteer;

                    // Normalize speeds if any one exceeds +/- 1.0;
                    dblMaxSpeed = Math.max(Math.abs(dblStepSpeedTempLeft), Math.abs(dblStepSpeedTempRight));
                    if (dblMaxSpeed > 1.0) {
                        dblStepSpeedTempLeft /= dblMaxSpeed;
                        dblStepSpeedTempRight /= dblMaxSpeed;
                    }

                }
                fileLogger.writeEvent(3,"dblDistanceToEnd " + dblDistanceToEnd);

                if (mblnRobotLastPos) {
                    if (dblDistanceToEnd <= 3.0) {
                        fileLogger.writeEvent(3,"mblnRobotLastPos Complete         ");
                        mblnNextStepLastPos = true;
                        mblnDisableVisionProcessing = false;  //enable vision processing
                        mintCurrentStateDriveHeading = Constants.stepState.STATE_COMPLETE;
                        deleteParallelStep();
                    }
                }

                //if within error margin stop
                //if (robotDrive.leftMotor1.isBusy() && robotDrive.rightMotor1.isBusy()) {
                //get motor busy state bitmap is right2, right1, left2, left1
                if (((robotDrive.getHardwareDriveIsBusy() & (robotConfigSettings.motors.leftMotor1.toInt() | robotConfigSettings.motors.rightMotor1.toInt())) == (robotConfigSettings.motors.leftMotor1.toInt() | robotConfigSettings.motors.rightMotor1.toInt()))) {
                    fileLogger.writeEvent(3,"Encoder counts per inch = " + ourRobotConfig.getCOUNTS_PER_INCH() + " dblDistanceFromStart " + dblDistanceFromStart + " dblDistanceToEnd " + dblDistanceToEnd + " Power Level Left " + dblStepSpeedTempLeft + " Power Level Right " + dblStepSpeedTempRight + " Running to target  L1, L2, R1, R2  " + mintStepLeftTarget1 + ", " + mintStepLeftTarget2 + ", " + mintStepRightTarget1 + ",  " + mintStepRightTarget2 + ", " + " Running at position L1 " + intLeft1MotorEncoderPosition + " L2 " + intLeft2MotorEncoderPosition + " R1 " + intRight1MotorEncoderPosition + " R2 " + intRight2MotorEncoderPosition);
                    dashboard.displayPrintf(3, "Path1", "Running to %7d :%7d", mintStepLeftTarget1, mintStepRightTarget1);
                    dashboard.displayPrintf(4, "Path2", "Running at %7d :%7d", intLeft1MotorEncoderPosition, intRight1MotorEncoderPosition);
                    dashboard.displayPrintf(5, "Path3", "Running at %7d :%7d", intLeft2MotorEncoderPosition, intRight2MotorEncoderPosition);
                    // set power on motor controller to update speeds
                    robotDrive.setHardwareDriveLeftMotorPower(dblStepSpeedTempLeft);
                    robotDrive.setHardwareDriveRightMotorPower(dblStepSpeedTempRight);
                } else {
                    // Stop all motion;
                    robotDrive.setHardwareDrivePower(0);
                    fileLogger.writeEvent(2,"Complete         ");
                    mblnDisableVisionProcessing = false;  //enable vision processing
                    mintCurrentStateDriveHeading = Constants.stepState.STATE_COMPLETE;
                    deleteParallelStep();
                }

                //check timeout value
                if (mStateTime.seconds() > mdblStepTimeout) {
                    fileLogger.writeEvent(1,"Timeout:- " + mStateTime.seconds());
                    //  Transition to a new state.
                    mintCurrentStateDriveHeading = Constants.stepState.STATE_COMPLETE;
                    deleteParallelStep();
                }
                break;
        }
    }

    //--------------------------------------------------------------------------
    //  Execute the state.
    //--------------------------------------------------------------------------

    private void PivotTurnStep()  //should be same as radius turn with radius of 1/2 robot width, so this function can be deleted once radius turn is completed
    {
        fileLogger.setEventTag("PivotTurnStep()");

        double dblDistanceToEndLeft1;
        double dblDistanceToEndLeft2;
        double dblDistanceToEndRight1;
        double dblDistanceToEndRight2;
        int intLeft1MotorEncoderPosition;
        int intLeft2MotorEncoderPosition;
        int intRight1MotorEncoderPosition;
        int intRight2MotorEncoderPosition;

        switch (mintCurrentStatePivotTurn) {
            case STATE_INIT: {
                // set motor controller to mode
                robotDrive.setHardwareDriveRunUsingEncoders();
                mblnDisableVisionProcessing = true;  //disable vision processing
                mdblStepTurnL = 0;
                mdblStepTurnR = 0;

                switch (mstrRobotCommand.substring(0, 3)) {
                    case "LPE":
                        mdblStepTurnL = Double.parseDouble(mstrRobotCommand.substring(3));
                        mdblStepTurnR = 0;
                        break;
                    case "RPE":
                        mdblStepTurnL = 0;
                        mdblStepTurnR = Double.parseDouble(mstrRobotCommand.substring(3));
                        break;
                }

                fileLogger.writeEvent(2,"mdblStepTurnL      :- " + mdblStepTurnL);
                fileLogger.writeEvent(2,"mdblStepTurnR      :- " + mdblStepTurnR);

                // Turn On RUN_TO_POSITION
                if (mdblStepTurnR == 0) {
                    // Determine new target position
                    fileLogger.writeEvent(2,"Current LPosition:-" + robotDrive.baseMotor1.getCurrentPosition());

                    // Get Current Encoder positions
                    if (mblnNextStepLastPos) {
                        mintStartPositionLeft1 = mintLastEncoderDestinationLeft1;
                        mintStartPositionLeft2 = mintLastEncoderDestinationLeft2;
                        mintStartPositionRight1 = mintLastEncoderDestinationRight1;
                        mintStartPositionRight2 = mintLastEncoderDestinationRight2;
                    } else {
                        mintStartPositionLeft1 = robotDrive.baseMotor1.getCurrentPosition();
                        mintStartPositionLeft2 = robotDrive.baseMotor2.getCurrentPosition();
                        mintStartPositionRight1 = robotDrive.baseMotor3.getCurrentPosition();
                        mintStartPositionRight2 = robotDrive.baseMotor4.getCurrentPosition();
                    }
                    mblnNextStepLastPos = false;

                    // Determine new target position
                    mintStepLeftTarget1 = mintStartPositionLeft1 + (int) (mdblStepTurnL * ourRobotConfig.getCOUNTS_PER_DEGREE());
                    mintStepLeftTarget2 = mintStartPositionLeft2 + (int) (mdblStepTurnL * ourRobotConfig.getCOUNTS_PER_DEGREE());
                    mintStepRightTarget1 = mintStartPositionRight1 + (int) (mdblStepTurnR * ourRobotConfig.getCOUNTS_PER_DEGREE());
                    mintStepRightTarget2 = mintStartPositionRight2 + (int) (mdblStepTurnR * ourRobotConfig.getCOUNTS_PER_DEGREE());

                    //store the encoder positions so next step can calculate destination
                    mintLastEncoderDestinationLeft1 = mintStepLeftTarget1;
                    mintLastEncoderDestinationLeft2 = mintStepLeftTarget2;
                    mintLastEncoderDestinationRight1 = mintStepRightTarget1;
                    mintLastEncoderDestinationRight2 = mintStepRightTarget2;
                    fileLogger.writeEvent(2,"mintStepLeftTarget1:-  " + mintStepLeftTarget1 + " mintStepLeftTarget2:-  " + mintStepLeftTarget2);

                    // pass target position to motor controller
                    robotDrive.baseMotor1.setTargetPosition(mintStepLeftTarget1);
                    robotDrive.baseMotor2.setTargetPosition(mintStepLeftTarget2);

                    // set left motor controller to mode
                    robotDrive.setHardwareDriveLeftRunToPosition();

                    // set power on motor controller to start moving
                    robotDrive.setHardwareDriveLeftMotorPower(Math.abs(mdblStepSpeed));
                } else {
                    // Determine new target position
                    fileLogger.writeEvent(2,"Current RPosition:-" + robotDrive.baseMotor3.getCurrentPosition());

                    // Get Current Encoder positions
                    if (mblnNextStepLastPos) {
                        mintStartPositionLeft1 = mintLastEncoderDestinationLeft1;
                        mintStartPositionLeft2 = mintLastEncoderDestinationLeft2;
                        mintStartPositionRight1 = mintLastEncoderDestinationRight1;
                        mintStartPositionRight2 = mintLastEncoderDestinationRight2;
                    } else {
                        mintStartPositionLeft1 = robotDrive.baseMotor1.getCurrentPosition();
                        mintStartPositionLeft2 = robotDrive.baseMotor2.getCurrentPosition();
                        mintStartPositionRight1 = robotDrive.baseMotor3.getCurrentPosition();
                        mintStartPositionRight2 = robotDrive.baseMotor4.getCurrentPosition();
                    }
                    mblnNextStepLastPos = false;

                    // Determine new target position
                    mintStepLeftTarget1 = mintStartPositionLeft1 + (int) (mdblStepTurnL * ourRobotConfig.getCOUNTS_PER_DEGREE());
                    mintStepLeftTarget2 = mintStartPositionLeft2 + (int) (mdblStepTurnL * ourRobotConfig.getCOUNTS_PER_DEGREE());
                    mintStepRightTarget1 = mintStartPositionRight1 + (int) (mdblStepTurnR * ourRobotConfig.getCOUNTS_PER_DEGREE());
                    mintStepRightTarget2 = mintStartPositionRight2 + (int) (mdblStepTurnR * ourRobotConfig.getCOUNTS_PER_DEGREE());

                    //store the encoder positions so next step can calculate destination
                    mintLastEncoderDestinationLeft1 = mintStepLeftTarget1;
                    mintLastEncoderDestinationLeft2 = mintStepLeftTarget2;
                    mintLastEncoderDestinationRight1 = mintStepRightTarget1;
                    mintLastEncoderDestinationRight2 = mintStepRightTarget2;

                    fileLogger.writeEvent(3,"mintStepRightTarget1:- " + mintStepRightTarget1 + " mintStepRightTarget2:- " + mintStepRightTarget2);

                    // pass target position to motor controller
                    robotDrive.baseMotor3.setTargetPosition(mintStepRightTarget1);
                    robotDrive.baseMotor4.setTargetPosition(mintStepRightTarget2);

                    // set right motor controller to mode, Turn On RUN_TO_POSITION
                    robotDrive.setHardwareDriveRightRunToPosition();

                    // set power on motor controller to start moving
                    robotDrive.setHardwareDriveRightMotorPower(Math.abs(mdblStepSpeed));
                }

                //store the encoder positions so next step can calculate destination
                mintLastEncoderDestinationLeft1 = mintStepLeftTarget1;
                mintLastEncoderDestinationLeft2 = mintStepLeftTarget2;
                mintLastEncoderDestinationRight1 = mintStepRightTarget1;
                mintLastEncoderDestinationRight2 = mintStepRightTarget2;

                fileLogger.writeEvent(3,"gblStepLeftTarget:- " + mintStepLeftTarget1 + " mintStepLeftTarget2 :- " + mintStepLeftTarget2);
                fileLogger.writeEvent(3,"gblStepRightTarget:- " + mintStepRightTarget1 + " mintStepRightTarget2:- " + mintStepRightTarget2);
                mintCurrentStatePivotTurn = Constants.stepState.STATE_RUNNING;
            }
            break;
            case STATE_RUNNING: {

                intLeft1MotorEncoderPosition = robotDrive.baseMotor1.getCurrentPosition();
                intLeft2MotorEncoderPosition = robotDrive.baseMotor2.getCurrentPosition();
                intRight1MotorEncoderPosition = robotDrive.baseMotor3.getCurrentPosition();
                intRight2MotorEncoderPosition = robotDrive.baseMotor4.getCurrentPosition();

                //determine how close to target we are
                dblDistanceToEndLeft1 = (mintStepLeftTarget1 - intLeft1MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();
                dblDistanceToEndLeft2 = (mintStepLeftTarget2 - intLeft2MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();
                dblDistanceToEndRight1 = (mintStepRightTarget1 - intRight1MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();
                dblDistanceToEndRight2 = (mintStepRightTarget2 - intRight2MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();

                fileLogger.writeEvent(3,"Current LPosition1:-" + robotDrive.baseMotor1.getCurrentPosition() + " LTarget:- " + mintStepLeftTarget1 + " LPosition2:-" + robotDrive.baseMotor2.getCurrentPosition() + " LTarget2:- " + mintStepLeftTarget2);
                fileLogger.writeEvent(3,"Current RPosition1:-" + robotDrive.baseMotor3.getCurrentPosition() + " RTarget:- " + mintStepRightTarget1 + " RPosition2:-" + robotDrive.baseMotor4.getCurrentPosition() + " RTarget2:- " + mintStepRightTarget2);

                if (mdblStepTurnR == 0) {
                    fileLogger.writeEvent(3,"Running.......");
                    dashboard.displayPrintf(3, "Target", "Running to %7d :%7d", mintStepLeftTarget1, mintStepRightTarget1);
                    dashboard.displayPrintf(4, "Actual_Left", "Running at %7d :%7d", intLeft1MotorEncoderPosition, intLeft2MotorEncoderPosition);
                    dashboard.displayPrintf(5, "ActualRight", "Running at %7d :%7d", intRight1MotorEncoderPosition, intRight2MotorEncoderPosition);

                    if (mblnRobotLastPos) {
                        if (((dblDistanceToEndLeft1 + dblDistanceToEndLeft2) / 2) < 2.0) {
                            mblnNextStepLastPos = true;
                            mblnDisableVisionProcessing = false;  //enable vision processing
                            mintCurrentStatePivotTurn = Constants.stepState.STATE_COMPLETE;
                            deleteParallelStep();
                        }
                    }
                    //if (!robotDrive.leftMotor1.isBusy()) {
                    //get motor busy state bitmap is right2, right1, left2, left1
                    if (((robotDrive.getHardwareDriveIsBusy() & robotConfigSettings.motors.leftMotor1.toInt()) == robotConfigSettings.motors.leftMotor1.toInt())) {
                        fileLogger.writeEvent(1,"Complete........");
                        mblnDisableVisionProcessing = false;  //enable vision processing
                        mintCurrentStatePivotTurn = Constants.stepState.STATE_COMPLETE;
                        deleteParallelStep();
                    }
                } else if (mdblStepTurnL == 0) {
                    fileLogger.writeEvent(3,"Running.......");
                    dashboard.displayPrintf(3, "Target", "Running to %7d :%7d", mintStepLeftTarget1, mintStepRightTarget1);
                    dashboard.displayPrintf(4, "Actual_Left", "Running at %7d :%7d", intLeft1MotorEncoderPosition, intLeft2MotorEncoderPosition);
                    dashboard.displayPrintf(5, "ActualRight", "Running at %7d :%7d", intRight1MotorEncoderPosition, intRight2MotorEncoderPosition);

                    if (mblnRobotLastPos) {
                        if (((dblDistanceToEndRight1 + dblDistanceToEndRight2) / 2) < 2.2) {
                            mblnNextStepLastPos = true;
                            mblnDisableVisionProcessing = false;  //enable vision processing
                            mintCurrentStatePivotTurn = Constants.stepState.STATE_COMPLETE;
                            deleteParallelStep();
                        }
                    }
                    //if (!robotDrive.rightMotor1.isBusy()) {
                    //get motor busy state bitmap is right2, right1, left2, left1
                    if (((robotDrive.getHardwareDriveIsBusy() & robotConfigSettings.motors.rightMotor1.toInt()) == robotConfigSettings.motors.rightMotor1.toInt())) {
                        fileLogger.writeEvent(1,"Complete.......");
                        mblnDisableVisionProcessing = false;  //enable vision processing
                        mintCurrentStatePivotTurn = Constants.stepState.STATE_COMPLETE;
                        deleteParallelStep();
                    }
                } else {
                    // Stop all motion by setting power to 0
                    robotDrive.setHardwareDrivePower(0);
                    fileLogger.writeEvent(1,"Complete.......");
                    mblnDisableVisionProcessing = false;  //enable vision processing
                    mintCurrentStatePivotTurn = Constants.stepState.STATE_COMPLETE;
                    deleteParallelStep();
                }
            }
            //check timeout value
            if (mStateTime.seconds() > mdblStepTimeout) {
                fileLogger.writeEvent(1,"Timeout:- " + mStateTime.seconds());
                //  Transition to a new state.
                mintCurrentStatePivotTurn = Constants.stepState.STATE_COMPLETE;
                deleteParallelStep();
            }
            break;
        }
    }

    private void TankTurnStep() {
        fileLogger.setEventTag("TankTurnStep()");
        double dblDistanceToEndLeft1;
        double dblDistanceToEndLeft2;
        double dblDistanceToEndRight1;
        double dblDistanceToEndRight2;
        int intLeft1MotorEncoderPosition;
        int intLeft2MotorEncoderPosition;
        int intRight1MotorEncoderPosition;
        int intRight2MotorEncoderPosition;

        switch (mintCurrentStateTankTurn) {
            case STATE_INIT: {
                // set motor controller to mode
                robotDrive.setHardwareDriveRunUsingEncoders();

                mblnDisableVisionProcessing = true;  //disable vision processing

                mdblStepTurnL = 0;
                mdblStepTurnR = 0;

                // Get Current Encoder positions
                if (mblnNextStepLastPos) {
                    mintStartPositionLeft1 = mintLastEncoderDestinationLeft1;
                    mintStartPositionLeft2 = mintLastEncoderDestinationLeft2;
                    mintStartPositionRight1 = mintLastEncoderDestinationRight1;
                    mintStartPositionRight2 = mintLastEncoderDestinationRight2;
                } else {
                    mintStartPositionLeft1 = robotDrive.baseMotor1.getCurrentPosition();
                    mintStartPositionLeft2 = robotDrive.baseMotor2.getCurrentPosition();
                    mintStartPositionRight1 = robotDrive.baseMotor3.getCurrentPosition();
                    mintStartPositionRight2 = robotDrive.baseMotor4.getCurrentPosition();
                }
                mblnNextStepLastPos = false;

                // Determine new target position
                switch (mstrRobotCommand.substring(0, 3)) {
                    case "LTE":
                        mintStepLeftTarget1 = mintStartPositionLeft1 - (int) (0.5 * Double.parseDouble(mstrRobotCommand.substring(3)) * ourRobotConfig.getCOUNTS_PER_DEGREE());
                        mintStepLeftTarget2 = mintStartPositionLeft2 - (int) (0.5 * Double.parseDouble(mstrRobotCommand.substring(3)) * ourRobotConfig.getCOUNTS_PER_DEGREE());
                        mintStepRightTarget1 = mintStartPositionRight1 + (int) (0.5 * Double.parseDouble(mstrRobotCommand.substring(3)) * ourRobotConfig.getCOUNTS_PER_DEGREE());
                        mintStepRightTarget2 = mintStartPositionRight2 + (int) (0.5 * Double.parseDouble(mstrRobotCommand.substring(3)) * ourRobotConfig.getCOUNTS_PER_DEGREE());
                        break;
                    case "RTE":
                        mintStepLeftTarget1 = mintStartPositionLeft1 + (int) (0.5 * Double.parseDouble(mstrRobotCommand.substring(3)) * ourRobotConfig.getCOUNTS_PER_DEGREE());
                        mintStepLeftTarget2 = mintStartPositionLeft2 + (int) (0.5 * Double.parseDouble(mstrRobotCommand.substring(3)) * ourRobotConfig.getCOUNTS_PER_DEGREE());
                        mintStepRightTarget1 = mintStartPositionRight1 - (int) (0.5 * Double.parseDouble(mstrRobotCommand.substring(3)) * ourRobotConfig.getCOUNTS_PER_DEGREE());
                        mintStepRightTarget2 = mintStartPositionRight2 - (int) (0.5 * Double.parseDouble(mstrRobotCommand.substring(3)) * ourRobotConfig.getCOUNTS_PER_DEGREE());
                        break;
                }

                //store the encoder positions so next step can calculate destination
                mintLastEncoderDestinationLeft1 = mintStepLeftTarget1;
                mintLastEncoderDestinationLeft2 = mintStepLeftTarget2;
                mintLastEncoderDestinationRight1 = mintStepRightTarget1;
                mintLastEncoderDestinationRight2 = mintStepRightTarget2;

                fileLogger.writeEvent(3,"Current LPosition1:- " + robotDrive.baseMotor1.getCurrentPosition() + " mintStepLeftTarget1:-   " + mintStepLeftTarget1);
                fileLogger.writeEvent(3,"Current LPosition2:- " + robotDrive.baseMotor2.getCurrentPosition() + " mintStepLeftTarget2:-   " + mintStepLeftTarget2);
                fileLogger.writeEvent(3,"Current RPosition1:- " + robotDrive.baseMotor3.getCurrentPosition() + " mintStepRightTarget1:- " + mintStepRightTarget1);
                fileLogger.writeEvent(3,"Current RPosition2:- " + robotDrive.baseMotor4.getCurrentPosition() + " mintStepRightTarget2:- " + mintStepRightTarget2);

                // pass target position to motor controller
                robotDrive.baseMotor1.setTargetPosition(mintStepLeftTarget1);
                robotDrive.baseMotor2.setTargetPosition(mintStepLeftTarget2);
                robotDrive.baseMotor3.setTargetPosition(mintStepRightTarget1);
                robotDrive.baseMotor4.setTargetPosition(mintStepRightTarget2);
                // set motor controller to mode
                robotDrive.setHardwareDriveRunToPosition();
                // set power on motor controller to start moving
                robotDrive.setHardwareDrivePower(Math.abs(mdblStepSpeed));
                fileLogger.writeEvent(2,"mintStepLeftTarget1 :- " + mintStepLeftTarget1);
                fileLogger.writeEvent(2,"mintStepLeftTarget2 :- " + mintStepLeftTarget2);
                fileLogger.writeEvent(2,"mintStepRightTarget1:- " + mintStepRightTarget1);
                fileLogger.writeEvent(2,"mintStepRightTarget2:- " + mintStepRightTarget2);

                mintCurrentStateTankTurn = Constants.stepState.STATE_RUNNING;
            }
            break;
            case STATE_RUNNING: {
                robotDrive.baseMotor1.setTargetPosition(mintStepLeftTarget1);
                robotDrive.baseMotor2.setTargetPosition(mintStepLeftTarget2);
                robotDrive.baseMotor3.setTargetPosition(mintStepRightTarget1);
                robotDrive.baseMotor4.setTargetPosition(mintStepRightTarget2);

                // set power on motor controller to start moving
                robotDrive.setHardwareDrivePower(Math.abs(mdblStepSpeed));

                intLeft1MotorEncoderPosition = robotDrive.baseMotor1.getCurrentPosition();
                intLeft2MotorEncoderPosition = robotDrive.baseMotor2.getCurrentPosition();
                intRight1MotorEncoderPosition = robotDrive.baseMotor3.getCurrentPosition();
                intRight2MotorEncoderPosition = robotDrive.baseMotor4.getCurrentPosition();

                //determine how close to target we are
                dblDistanceToEndLeft1 = (mintStepLeftTarget1 - intLeft1MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();
                dblDistanceToEndLeft2 = (mintStepLeftTarget2 - intLeft2MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();
                dblDistanceToEndRight1 = (mintStepRightTarget1 - intRight1MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();
                dblDistanceToEndRight2 = (mintStepRightTarget2 - intRight2MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();

                fileLogger.writeEvent(3,"Current LPosition1:- " + intLeft1MotorEncoderPosition + " LTarget1:- " + mintStepLeftTarget1);
                fileLogger.writeEvent(3,"Current LPosition2:- " + intLeft2MotorEncoderPosition + " LTarget2:- " + mintStepLeftTarget2);
                fileLogger.writeEvent(3,"Current RPosition1:- " + intRight1MotorEncoderPosition + " RTarget1:- " + mintStepRightTarget1);
                fileLogger.writeEvent(3,"Current RPosition2:- " + intRight2MotorEncoderPosition + " RTarget2:- " + mintStepRightTarget2);

                dashboard.displayPrintf(4, LABEL_WIDTH, "Left  Target: ", "Running to %7d :%7d", mintStepLeftTarget1, mintStepLeftTarget2);
                dashboard.displayPrintf(5, LABEL_WIDTH, "Left  Actual: ", "Running at %7d :%7d", intLeft1MotorEncoderPosition, intLeft2MotorEncoderPosition);
                dashboard.displayPrintf(6, LABEL_WIDTH, "Right Target: ", "Running to %7d :%7d", mintStepRightTarget1, mintStepRightTarget2);
                dashboard.displayPrintf(7, LABEL_WIDTH, "Right Actual: ", "Running at %7d :%7d", intRight1MotorEncoderPosition, intRight2MotorEncoderPosition);

                if (mblnRobotLastPos) {
                    if (((Math.abs(dblDistanceToEndRight1 + dblDistanceToEndRight2) / 2) < 2.2) && ((Math.abs(dblDistanceToEndLeft1 + dblDistanceToEndLeft2) / 2) < 2.2)) {
                        mblnNextStepLastPos = true;
                        mblnDisableVisionProcessing = false;  //enable vision processing
                        mintCurrentStateTankTurn = Constants.stepState.STATE_COMPLETE;
                        deleteParallelStep();
                    }
                }

                //if within error margin stop
                //get motor busy state bitmap is right2, right1, left2, left1
                if (!robotDrive.baseMotor1.isBusy() || (!robotDrive.baseMotor3.isBusy())) {
                    fileLogger.writeEvent(3,"Complete.......");
                    robotDrive.setHardwareDrivePower(0);
                    mblnDisableVisionProcessing = false;  //enable vision processing
                    mintCurrentStateTankTurn = Constants.stepState.STATE_COMPLETE;
                    deleteParallelStep();
                }
            }
            //check timeout value
            if (mStateTime.seconds() > mdblStepTimeout) {
                fileLogger.writeEvent(1,"Timeout:- " + mStateTime.seconds());
                //  Transition to a new state.
                mintCurrentStateTankTurn = Constants.stepState.STATE_COMPLETE;
                deleteParallelStep();
            }
            break;
        }
    }

    //this has not been programmed, do not use
    private void RadiusTurnStep() {
        fileLogger.setEventTag("RadiusTurnStep()");
        double dblDistanceToEndLeft1;
        double dblDistanceToEndLeft2;
        double dblDistanceToEndRight1;
        double dblDistanceToEndRight2;
        int intLeft1MotorEncoderPosition;
        int intLeft2MotorEncoderPosition;
        int intRight1MotorEncoderPosition;
        int intRight2MotorEncoderPosition;
        double dblArcLengthRadiusTurnInner;             //used to calculate the arc length when doing a radius turn
        double rdblArcLengthRadiusTurnOuter;             //used to calculate the arc length when doing a radius turn
        double rdblSpeedOuter;                           //used to calculate the speed of the outer wheels during the turn
        double rdblSpeedInner;                           //used to calculate the speed of the inner wheels during the turn

        switch (mintCurrentStateRadiusTurn) {
            case STATE_INIT: {
                robotDrive.setHardwareDriveRunUsingEncoders();

                mblnDisableVisionProcessing = true;  //disable vision processing

                mdblRobotTurnAngle = Double.parseDouble(mstrRobotCommand.substring(3));
                fileLogger.writeEvent(3,"mdblRobotTurnAngle" + mdblRobotTurnAngle);

                //calculate the distance to travel based on the angle we are turning
                // length = radius x angle (in radians)
                rdblArcLengthRadiusTurnOuter = ((Double.parseDouble(mstrRobotCommand.substring(3)) / 180) * Math.PI) * mdblRobotParm1;
                dblArcLengthRadiusTurnInner = ((Double.parseDouble(mstrRobotCommand.substring(3)) / 180) * Math.PI) * (mdblRobotParm1 - (ourRobotConfig.getROBOT_TRACK()));
                //rdblArcLengthRadiusTurnOuter = ((Double.parseDouble(mstrRobotCommand.substring(3)) / 180) *  Math.PI) * (mdblRobotParm1 + (0.5 * ROBOT_TRACK));

                rdblSpeedOuter = mdblStepSpeed;

                if (rdblSpeedOuter >= 0.58) {
                    rdblSpeedOuter = 0.58;  //This is the maximum speed, anything above 0.6 is the same as a speed of 1 for drive to position
                }
                rdblSpeedInner = dblArcLengthRadiusTurnInner / rdblArcLengthRadiusTurnOuter * rdblSpeedOuter * 0.96;
                fileLogger.writeEvent(3,"dblArcLengthRadiusTurnInner " + dblArcLengthRadiusTurnInner);
                fileLogger.writeEvent(3,"rdblArcLengthRadiusTurnOuter " + rdblArcLengthRadiusTurnOuter);
                fileLogger.writeEvent(3,"rdblSpeedOuter " + rdblSpeedOuter);
                fileLogger.writeEvent(3,"rdblSpeedInner " + rdblSpeedInner);

                // Get Current Encoder positions
                if (mblnNextStepLastPos) {
                    mintStartPositionLeft1 = mintLastEncoderDestinationLeft1;
                    mintStartPositionLeft2 = mintLastEncoderDestinationLeft2;
                    mintStartPositionRight1 = mintLastEncoderDestinationRight1;
                    mintStartPositionRight2 = mintLastEncoderDestinationRight2;
                } else {
                    mintStartPositionLeft1 = robotDrive.baseMotor1.getCurrentPosition();
                    mintStartPositionLeft2 = robotDrive.baseMotor2.getCurrentPosition();
                    mintStartPositionRight1 = robotDrive.baseMotor3.getCurrentPosition();
                    mintStartPositionRight2 = robotDrive.baseMotor4.getCurrentPosition();
                }
                mblnNextStepLastPos = false;

                // Determine new target position
                switch (mstrRobotCommand.substring(0, 3)) {
                    case "LRE":
                        mintStepLeftTarget1 = mintStartPositionLeft1 + (int) (dblArcLengthRadiusTurnInner * ourRobotConfig.getCOUNTS_PER_INCH());
                        mintStepLeftTarget2 = mintStartPositionLeft2 + (int) (dblArcLengthRadiusTurnInner * ourRobotConfig.getCOUNTS_PER_INCH());
                        mintStepRightTarget1 = mintStartPositionRight1 + (int) (rdblArcLengthRadiusTurnOuter * ourRobotConfig.getCOUNTS_PER_INCH());
                        mintStepRightTarget2 = mintStartPositionRight2 + (int) (rdblArcLengthRadiusTurnOuter * ourRobotConfig.getCOUNTS_PER_INCH());

                        //store the encoder positions so next step can calculate destination
                        mintLastEncoderDestinationLeft1 = mintStepLeftTarget1;
                        mintLastEncoderDestinationLeft2 = mintStepLeftTarget2;
                        mintLastEncoderDestinationRight1 = mintStepRightTarget1;
                        mintLastEncoderDestinationRight2 = mintStepRightTarget2;

                        // pass target position to motor controller
                        robotDrive.baseMotor1.setTargetPosition(mintStepLeftTarget1);
                        robotDrive.baseMotor2.setTargetPosition(mintStepLeftTarget2);
                        robotDrive.baseMotor3.setTargetPosition(mintStepRightTarget1);
                        robotDrive.baseMotor4.setTargetPosition(mintStepRightTarget2);

                        // set motor controller to mode
                        robotDrive.setHardwareDriveRunToPosition();

                        // set power on motor controller to start moving
                        robotDrive.setHardwareDriveLeftMotorPower(rdblSpeedInner);  //left side is inner when turning left
                        robotDrive.setHardwareDriveRightMotorPower(rdblSpeedOuter);  //right side is outer when turning left
                        break;
                    case "RRE":
                        mintStepLeftTarget1 = mintStartPositionLeft1 + (int) (rdblArcLengthRadiusTurnOuter * ourRobotConfig.getCOUNTS_PER_INCH());
                        mintStepLeftTarget2 = mintStartPositionLeft2 + (int) (rdblArcLengthRadiusTurnOuter * ourRobotConfig.getCOUNTS_PER_INCH());
                        mintStepRightTarget1 = mintStartPositionRight1 + (int) (dblArcLengthRadiusTurnInner * ourRobotConfig.getCOUNTS_PER_INCH());
                        mintStepRightTarget2 = mintStartPositionRight2 + (int) (dblArcLengthRadiusTurnInner * ourRobotConfig.getCOUNTS_PER_INCH());

                        //store the encoder positions so next step can calculate destination
                        mintLastEncoderDestinationLeft1 = mintStepLeftTarget1;
                        mintLastEncoderDestinationLeft2 = mintStepLeftTarget2;
                        mintLastEncoderDestinationRight1 = mintStepRightTarget1;
                        mintLastEncoderDestinationRight2 = mintStepRightTarget2;

                        // pass target position to motor controller
                        robotDrive.baseMotor1.setTargetPosition(mintStepLeftTarget1);
                        robotDrive.baseMotor2.setTargetPosition(mintStepLeftTarget2);
                        robotDrive.baseMotor3.setTargetPosition(mintStepRightTarget1);
                        robotDrive.baseMotor4.setTargetPosition(mintStepRightTarget2);

                        // set motor controller to mode
                        robotDrive.setHardwareDriveRunToPosition();

                        // set power on motor controller to start moving
                        robotDrive.setHardwareDriveLeftMotorPower(rdblSpeedOuter);  //left side is outer when turning left
                        robotDrive.setHardwareDriveRightMotorPower(rdblSpeedInner);  //right side is inner when turning left
                        break;
                }

                fileLogger.writeEvent(3,"Current LPosition1:- " + robotDrive.baseMotor1.getCurrentPosition() + " mintStepLeftTarget1:-   " + mintStepLeftTarget1);
                fileLogger.writeEvent(3,"Current LPosition2:- " + robotDrive.baseMotor2.getCurrentPosition() + " mintStepLeftTarget2:-   " + mintStepLeftTarget2);
                fileLogger.writeEvent(3,"Current RPosition1:- " + robotDrive.baseMotor3.getCurrentPosition() + " mintStepRightTarget1:- " + mintStepRightTarget1);
                fileLogger.writeEvent(3,"Current RPosition2:- " + robotDrive.baseMotor4.getCurrentPosition() + " mintStepRightTarget2:- " + mintStepRightTarget2);
                mintCurrentStateRadiusTurn = Constants.stepState.STATE_RUNNING;
            }
            break;
            case STATE_RUNNING: {
                intLeft1MotorEncoderPosition = robotDrive.baseMotor1.getCurrentPosition();
                intLeft2MotorEncoderPosition = robotDrive.baseMotor2.getCurrentPosition();
                intRight1MotorEncoderPosition = robotDrive.baseMotor3.getCurrentPosition();
                intRight2MotorEncoderPosition = robotDrive.baseMotor4.getCurrentPosition();

                //determine how close to target we are
                dblDistanceToEndLeft1 = (mintStepLeftTarget1 - intLeft1MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();
                dblDistanceToEndLeft2 = (mintStepLeftTarget2 - intLeft2MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();
                dblDistanceToEndRight1 = (mintStepRightTarget1 - intRight1MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();
                dblDistanceToEndRight2 = (mintStepRightTarget2 - intRight2MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();
                fileLogger.writeEvent(3,"Current LPosition1:- " + intLeft1MotorEncoderPosition + " LTarget1:- " + mintStepLeftTarget1);
                fileLogger.writeEvent(3,"Current LPosition2:- " + intLeft2MotorEncoderPosition + " LTarget2:- " + mintStepLeftTarget2);
                fileLogger.writeEvent(3,"Current RPosition1:- " + intRight1MotorEncoderPosition + " RTarget1:- " + mintStepRightTarget1);
                fileLogger.writeEvent(3,"Current RPosition2:- " + intRight2MotorEncoderPosition + " RTarget2:- " + mintStepRightTarget2);
                dashboard.displayPrintf(1, LABEL_WIDTH, "Target: ", "Running to %7d :%7d", mintStepLeftTarget1, mintStepRightTarget1);
                dashboard.displayPrintf(2, LABEL_WIDTH, "Actual_Left: ", "Running at %7d :%7d", intLeft1MotorEncoderPosition, intLeft2MotorEncoderPosition);
                dashboard.displayPrintf(3, LABEL_WIDTH, "ActualRight: ", "Running at %7d :%7d", intRight1MotorEncoderPosition, intRight2MotorEncoderPosition);

                if (mblnRobotLastPos) {
                    if ((((dblDistanceToEndRight1 + dblDistanceToEndRight2) / 2) < 2) && (((dblDistanceToEndLeft1 + dblDistanceToEndLeft2) / 2) < 2)) {
                        mblnNextStepLastPos = true;
                        mblnDisableVisionProcessing = false;  //enable vision processing
                        mintCurrentStateRadiusTurn = Constants.stepState.STATE_COMPLETE;
                        deleteParallelStep();
                    }
                }

                if (!robotDrive.baseMotor1.isBusy() || (!robotDrive.baseMotor3.isBusy())) {
                    robotDrive.setHardwareDrivePower(0);
                    fileLogger.writeEvent(1,"Complete         ");
                    mblnDisableVisionProcessing = false;  //enable vision processing
                    mintCurrentStateRadiusTurn = Constants.stepState.STATE_COMPLETE;
                    deleteParallelStep();
                }
            }
            //check timeout value
            if (mStateTime.seconds() > mdblStepTimeout) {
                fileLogger.writeEvent(1,"Timeout:- " + mStateTime.seconds());
                //  Transition to a new state.
                mintCurrentStateRadiusTurn = Constants.stepState.STATE_COMPLETE;
                deleteParallelStep();
            }
            break;
        }
    }

    private void MecanumStrafe() {
        fileLogger.setEventTag("MecanumStrafe()");
        int direction;

        double dblDistanceToEndLeft1;
        double dblDistanceToEndLeft2;
        double dblDistanceToEndRight1;
        double dblDistanceToEndRight2;
        int intLeft1MotorEncoderPosition;
        int intLeft2MotorEncoderPosition;
        int intRight1MotorEncoderPosition;
        int intRight2MotorEncoderPosition;
        double rdblSpeed;

        switch (mintCurrentStateMecanumStrafe) {
            case STATE_INIT: {
                double adafruitIMUHeading;
                double currentHeading;

                mdblStepDistanceX = Double.parseDouble(mstrRobotCommand.substring(3));

                robotDrive.setHardwareDriveRunUsingEncoders();
                mblnDisableVisionProcessing = true;  //disable vision processing

                adafruitIMUHeading = getAdafruitHeading();
                currentHeading = adafruitIMUHeading;
                //mdblRobotTurnAngle = Double.parseDouble(mstrRobotCommand.substring(3));
                //fileLogger.writeEvent(3, "MecanumStrafe", "USING HEADING FROM IMU=" + useAdafruitIMU);
                //fileLogger.writeEvent(3, "MecanumStrafe()", "mdblRobotTurnAngle " + mdblRobotTurnAngle + " currentHeading " + currentHeading);
                //mdblTurnAbsoluteGyro = Double.parseDouble(newAngleDirection((int) currentHeading, (int) mdblRobotTurnAngle).substring(3));

                robotDrive.setHardwareDriveRunToPosition();

                // Get Current Encoder positions
                if (mblnNextStepLastPos) {
                    mintStartPositionLeft1 = mintLastEncoderDestinationLeft1;
                    mintStartPositionLeft2 = mintLastEncoderDestinationLeft2;
                    mintStartPositionRight1 = mintLastEncoderDestinationRight1;
                    mintStartPositionRight2 = mintLastEncoderDestinationRight2;
                } else {
                    mintStartPositionLeft1 = robotDrive.baseMotor1.getCurrentPosition();
                    mintStartPositionLeft2 = robotDrive.baseMotor2.getCurrentPosition();
                    mintStartPositionRight1 = robotDrive.baseMotor3.getCurrentPosition();
                    mintStartPositionRight2 = robotDrive.baseMotor4.getCurrentPosition();
                }
                mblnNextStepLastPos = false;

                mintStepLeftTarget1 = mintStartPositionLeft1 + (int) (mdblStepDistanceX * ourRobotConfig.getCOUNTS_PER_INCH_STRAFE_LEFT_OFFSET() * ourRobotConfig.getCOUNTS_PER_INCH_STRAFE() * ourRobotConfig.getCOUNTS_PER_INCH_STRAFE_FRONT_OFFSET());
                mintStepLeftTarget2 = mintStartPositionLeft2 - (int) (mdblStepDistanceX * ourRobotConfig.getCOUNTS_PER_INCH_STRAFE_LEFT_OFFSET() * ourRobotConfig.getCOUNTS_PER_INCH_STRAFE() * ourRobotConfig.getCOUNTS_PER_INCH_STRAFE_REAR_OFFSET());
                mintStepRightTarget1 = mintStartPositionRight1 - (int) (mdblStepDistanceX * ourRobotConfig.getCOUNTS_PER_INCH_STRAFE_RIGHT_OFFSET() * ourRobotConfig.getCOUNTS_PER_INCH_STRAFE() * ourRobotConfig.getCOUNTS_PER_INCH_STRAFE_FRONT_OFFSET());
                mintStepRightTarget2 = mintStartPositionRight2 + (int) (mdblStepDistanceX * ourRobotConfig.getCOUNTS_PER_INCH_STRAFE_RIGHT_OFFSET() * ourRobotConfig.getCOUNTS_PER_INCH_STRAFE() + ourRobotConfig.getCOUNTS_PER_INCH_STRAFE_REAR_OFFSET());

                //store the encoder positions so next step can calculate destination
                mintLastEncoderDestinationLeft1 = mintStepLeftTarget1;
                mintLastEncoderDestinationLeft2 = mintStepLeftTarget2;
                mintLastEncoderDestinationRight1 = mintStepRightTarget1;
                mintLastEncoderDestinationRight2 = mintStepRightTarget2;

                // pass target position to motor controller
                robotDrive.baseMotor1.setTargetPosition(mintStepLeftTarget1);
                robotDrive.baseMotor2.setTargetPosition(mintStepLeftTarget2);
                robotDrive.baseMotor3.setTargetPosition(mintStepRightTarget1);
                robotDrive.baseMotor4.setTargetPosition(mintStepRightTarget2);

                // set motor controller to mode
                robotDrive.setHardwareDriveRunToPosition();
                rdblSpeed = mdblStepSpeed;

                if (rdblSpeed >= 0.7) {
                    rdblSpeed = 0.7;  //This is the maximum speed, anything above 0.6 is the same as a speed of 1 for drive to position
                }
                // set power on motor controller to start moving
                robotDrive.setHardwareDrivePower(rdblSpeed);  //set motor power
                mintCurrentStateMecanumStrafe = Constants.stepState.STATE_RUNNING;
            }
            break;
            case STATE_RUNNING: {
                rdblSpeed = mdblStepSpeed;
                if (rdblSpeed >= 0.7) {
                    rdblSpeed = 0.7;  //This is the maximum speed, anything above 0.6 is the same as a speed of 1 for drive to position
                }
                // pass target position to motor controller
                robotDrive.baseMotor1.setTargetPosition(mintStepLeftTarget1);
                robotDrive.baseMotor2.setTargetPosition(mintStepLeftTarget2);
                robotDrive.baseMotor3.setTargetPosition(mintStepRightTarget1);
                robotDrive.baseMotor4.setTargetPosition(mintStepRightTarget2);
                robotDrive.setHardwareDrivePower(rdblSpeed);  //set motor power

                double adafruitIMUHeading;

                adafruitIMUHeading = getAdafruitHeading();
                mdblGyrozAccumulated = adafruitIMUHeading;
                mdblGyrozAccumulated = teamAngleAdjust(mdblGyrozAccumulated);//Set variables to MRgyro readings
                mdblTurnAbsoluteGyro = Double.parseDouble(newAngleDirectionGyro((int) mdblGyrozAccumulated, (int) mdblRobotTurnAngle).substring(3));
                String mstrDirection = (newAngleDirectionGyro((int) mdblGyrozAccumulated, (int) mdblRobotTurnAngle).substring(0, 3));
                fileLogger.writeEvent(3, "USING HEADING FROM IMU=" + useAdafruitIMU);
                fileLogger.writeEvent(3, "Running, mdblGyrozAccumulated = " + mdblGyrozAccumulated);
                fileLogger.writeEvent(3, "Running, mdblTurnAbsoluteGyro = " + mdblTurnAbsoluteGyro);
                fileLogger.writeEvent(3, "Running, mstrDirection        = " + mstrDirection);
                fileLogger.writeEvent(3, "Running, adafruitIMUHeading   = " + adafruitIMUHeading);

                intLeft1MotorEncoderPosition = robotDrive.baseMotor1.getCurrentPosition();
                intLeft2MotorEncoderPosition = robotDrive.baseMotor2.getCurrentPosition();
                intRight1MotorEncoderPosition = robotDrive.baseMotor3.getCurrentPosition();
                intRight2MotorEncoderPosition = robotDrive.baseMotor4.getCurrentPosition();

                //determine how close to target we are
                dblDistanceToEndLeft1 = (mintStepLeftTarget1 - intLeft1MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();
                dblDistanceToEndLeft2 = (mintStepLeftTarget2 - intLeft2MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();
                dblDistanceToEndRight1 = (mintStepRightTarget1 - intRight1MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();
                dblDistanceToEndRight2 = (mintStepRightTarget2 - intRight2MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();
                fileLogger.writeEvent(3, "Current LPosition1:- " + intLeft1MotorEncoderPosition + " LTarget1:- " + mintStepLeftTarget1);
                fileLogger.writeEvent(3, "Current LPosition2:- " + intLeft2MotorEncoderPosition + " LTarget2:- " + mintStepLeftTarget2);
                fileLogger.writeEvent(3, "Current RPosition1:- " + intRight1MotorEncoderPosition + " RTarget1:- " + mintStepRightTarget1);
                fileLogger.writeEvent(3, "Current RPosition2:- " + intRight2MotorEncoderPosition + " RTarget2:- " + mintStepRightTarget2);
                dashboard.displayPrintf(4,  "Mecanum Strafe Positions moving " + mdblStepDistanceX);
                dashboard.displayPrintf(4, LABEL_WIDTH, "Left  Target: ", "Running to %7d :%7d", mintStepLeftTarget1, mintStepLeftTarget2);
                dashboard.displayPrintf(5, LABEL_WIDTH, "Left  Actual: ", "Running at %7d :%7d", intLeft1MotorEncoderPosition, intLeft2MotorEncoderPosition);
                dashboard.displayPrintf(6, LABEL_WIDTH, "Right Target: ", "Running to %7d :%7d", mintStepRightTarget1, mintStepRightTarget2);
                dashboard.displayPrintf(7, LABEL_WIDTH, "Right Actual: ", "Running at %7d :%7d", intRight1MotorEncoderPosition, intRight2MotorEncoderPosition);

                if (mblnRobotLastPos) {
                    if ((((dblDistanceToEndRight1 + dblDistanceToEndRight2) / 2) < 2) && (((dblDistanceToEndLeft1 + dblDistanceToEndLeft2) / 2) < 2)) {
                        mblnNextStepLastPos = true;
                        mintCurrentStateMecanumStrafe = Constants.stepState.STATE_COMPLETE;
                        deleteParallelStep();
                    }
                }

                if (!robotDrive.baseMotor1.isBusy() || (!robotDrive.baseMotor3.isBusy())) {
                    robotDrive.setHardwareDrivePower(0);
                    fileLogger.writeEvent(1, "Complete         ");
                    mintCurrentStateMecanumStrafe = Constants.stepState.STATE_COMPLETE;
                    deleteParallelStep();
                }

            } //end Case Running
            //check timeout value
            if (mStateTime.seconds() > mdblStepTimeout) {
                fileLogger.writeEvent(1, "Timeout:- " + mStateTime.seconds());
                //  Transition to a new state.
                mintCurrentStateMecanumStrafe = Constants.stepState.STATE_COMPLETE;
                deleteParallelStep();
            }
            break;
        }
    }

    private void TankTurnGyroHeading() {
        fileLogger.setEventTag("TankTurnGyroHeading()");
        switch (mintCurrentStateTankTurnGyroHeading) {
            case STATE_INIT: {
                double adafruitIMUHeading;

                adafruitIMUHeading = getAdafruitHeading();

                mdblPowerBoost = 0;
                mintStableCount = 0;
                mstrWiggleDir = "";
                mdblRobotTurnAngle = Double.parseDouble(mstrRobotCommand.substring(3));
                fileLogger.writeEvent(3,"USING HEADING FROM IMU=" + useAdafruitIMU);
                fileLogger.writeEvent(3,"mdblRobotTurnAngle " + mdblRobotTurnAngle + " adafruitIMUHeading " + adafruitIMUHeading);
                //mdblTurnAbsoluteGyro = Double.parseDouble(newAngleDirection((int) adafruitIMUHeading, (int) mdblRobotTurnAngle).substring(3));
                mdblTurnAbsoluteGyro = TOWR5291Utils.getNewHeading((int) adafruitIMUHeading, (int) mdblRobotTurnAngle);

                robotDrive.setHardwareDriveRunWithoutEncoders();

                mintCurrentStateTankTurnGyroHeading = Constants.stepState.STATE_RUNNING;
            }
            break;
            case STATE_RUNNING: {
                double adafruitIMUHeading;

                adafruitIMUHeading = getAdafruitHeading();

                mdblGyrozAccumulated = adafruitIMUHeading;
                mdblGyrozAccumulated = teamAngleAdjust(mdblGyrozAccumulated);//Set variables to MRgyro readings
                mdblTurnAbsoluteGyro = Double.parseDouble(newAngleDirectionGyro((int) mdblGyrozAccumulated, (int) mdblRobotTurnAngle).substring(3));
                String mstrDirection = (newAngleDirectionGyro((int) mdblGyrozAccumulated, (int) mdblRobotTurnAngle).substring(0, 3));
                fileLogger.writeEvent(3,"USING HEADING FROM IMU=" + useAdafruitIMU);
                fileLogger.writeEvent(3,"Running, mdblGyrozAccumulated = " + mdblGyrozAccumulated);
                fileLogger.writeEvent(3,"Running, mdblTurnAbsoluteGyro = " + mdblTurnAbsoluteGyro);
                fileLogger.writeEvent(3,"Running, mstrDirection        = " + mstrDirection);
                fileLogger.writeEvent(3,"Running, adafruitIMUHeading   = " + adafruitIMUHeading);

                if (Math.abs(mdblTurnAbsoluteGyro) > 21) {  //Continue while the robot direction is further than three degrees from the target
                    mintStableCount = 0;
                    fileLogger.writeEvent(3,"High Speed.....");
                    if (mstrDirection.equals("LTE")) {
                        //want to turn left
                        fileLogger.writeEvent(3,"Left Turn.....");
                        if (mstrWiggleDir.equals("RTE")) {
                            mdblPowerBoost = mdblPowerBoost - 0.01;
                            mintPowerBoostCount = 0;
                        }
                        mstrWiggleDir = "LTE";
                        robotDrive.setHardwareDriveLeftMotorPower(mdblStepSpeed);
                        robotDrive.setHardwareDriveRightMotorPower(-mdblStepSpeed);
                    } else if (mstrDirection.equals("RTE")) {
                        //want to turn left
                        if (mstrWiggleDir.equals("LTE")) {
                            mdblPowerBoost = mdblPowerBoost - 0.01;
                            mintPowerBoostCount = 0;
                        }
                        mstrWiggleDir = "RTE";
                        fileLogger.writeEvent(3,"Right Turn.....");
                        robotDrive.setHardwareDriveLeftMotorPower(-mdblStepSpeed);
                        robotDrive.setHardwareDriveRightMotorPower(mdblStepSpeed);
                    }
                } else if (Math.abs(mdblTurnAbsoluteGyro) > mdblRobotParm1) {  //Continue while the robot direction is further than three degrees from the target
                    mintStableCount = 0;
                    mintPowerBoostCount++;
                    if (mintPowerBoostCount > 50) {
                        mdblPowerBoost = mdblPowerBoost + 0.01;
                        mintPowerBoostCount = 0;
                    }
                    fileLogger.writeEvent(3, "TankTurnGyro()", "Slow Speed Nearing final angle.....");
                    if (mstrDirection.equals("LTE")) {
                        //want to turn left
                        if (mstrWiggleDir.equals("RTE")) {
                            mdblPowerBoost = mdblPowerBoost - 0.01;
                            mintPowerBoostCount = 0;
                        }
                        mstrWiggleDir = "LTE";
                        fileLogger.writeEvent(3, "TankTurnGyro()", "Left Turn.....");
                        robotDrive.setHardwareDriveLeftMotorPower(.12 + mdblPowerBoost);
                        robotDrive.setHardwareDriveRightMotorPower(-(0.12 + mdblPowerBoost));
                    } else if (mstrDirection.equals("RTE")) {
                        //want to turn left
                        if (mstrWiggleDir.equals("LTE")) {
                            mdblPowerBoost = mdblPowerBoost - 0.01;
                            mintPowerBoostCount = 0;
                        }
                        mstrWiggleDir = "RTE";
                        fileLogger.writeEvent(3, "TankTurnGyro()", "Right Turn.....");
                        robotDrive.setHardwareDriveLeftMotorPower(-(0.12 + mdblPowerBoost));
                        robotDrive.setHardwareDriveRightMotorPower(0.12 + mdblPowerBoost);
                    }
                } else {
                    mintStableCount++;
                    if (mintStableCount > 20) {
                        fileLogger.writeEvent(3, "TankTurnGyro()", "Complete.....");
                        robotDrive.setHardwareDriveRunUsingEncoders();
                        robotDrive.setHardwareDrivePower(0);
                        mintCurrentStateTankTurnGyroHeading = Constants.stepState.STATE_COMPLETE;
                        deleteParallelStep();
                    }
                }
            } //end Case Running
            //check timeout value
            if (mStateTime.seconds() > mdblStepTimeout) {
                fileLogger.writeEvent(1, "TankTurnGyro()", "Timeout:- " + mStateTime.seconds());
                //  Transition to a new state.
                mintCurrentStateTankTurnGyroHeading = Constants.stepState.STATE_COMPLETE;
                deleteParallelStep();
            }
            break;
        }
    }

    private void TankTurnGyroHeadingEncoder()
    {
        fileLogger.setEventTag("TankTurnGyroHeadingEncoder()");
        switch (mintCurrentStateGyroTurnEncoder5291){
            case STATE_INIT:
            {
                double adafruitIMUHeading;
                adafruitIMUHeading = getAdafruitHeading();
                mdblPowerBoost = 0;
                mintStableCount = 0;
                mstrWiggleDir = "";
                mdblRobotTurnAngle = Double.parseDouble(mstrRobotCommand.substring(3));
                fileLogger.writeEvent(3,"USE ADAFRUIT IMU = " + useAdafruitIMU + ",mdblRobotTurnAngle " + mdblRobotTurnAngle + " adafruitIMUHeading " + adafruitIMUHeading);
                //mdblTurnAbsoluteGyro = Double.parseDouble(newAngleDirection((int)adafruitIMUHeading, (int) mdblRobotTurnAngle).substring(3));
                mdblTurnAbsoluteGyro = TOWR5291Utils.getNewHeading((int) adafruitIMUHeading, (int) mdblRobotTurnAngle);
                mintCurrentStateGyroTurnEncoder5291 = Constants.stepState.STATE_RUNNING;
            }
            break;
            case STATE_RUNNING: {

                double adafruitIMUHeading;
                adafruitIMUHeading = getAdafruitHeading();
                mdblGyrozAccumulated = teamAngleAdjust(mdblGyrozAccumulated); //Set variables to MRgyro readings
                mdblTurnAbsoluteGyro = Double.parseDouble(newAngleDirectionGyro ((int)mdblGyrozAccumulated, (int)mdblRobotTurnAngle).substring(3));
                String mstrDirection = (newAngleDirectionGyro ((int)mdblGyrozAccumulated, (int)mdblRobotTurnAngle).substring(0, 3));
                fileLogger.writeEvent(3,"USING HEADING FROM IMU=" + useAdafruitIMU);
                fileLogger.writeEvent(3,"Running, mdblGyrozAccumulated = " + mdblGyrozAccumulated);
                fileLogger.writeEvent(3,"Running, mdblTurnAbsoluteGyro = " + mdblTurnAbsoluteGyro);
                fileLogger.writeEvent(3,"Running, mstrDirection        = " + mstrDirection);
                fileLogger.writeEvent(3,"Running, adafruitIMUHeading   = " + adafruitIMUHeading);
                autonomousStepsFile.insertSteps(3,  newAngleDirectionGyro ((int)mdblGyrozAccumulated, (int)mdblRobotTurnAngle),0,0,0, mdblStepSpeed,true , false, false, 0, 0, 0, 0, 0, 0,  mintCurrentStep + 1);
                mintCurrentStateGyroTurnEncoder5291 = Constants.stepState.STATE_COMPLETE;
                deleteParallelStep();
            }
            //check timeout value
            if (mStateTime.seconds() > mdblStepTimeout) {
                fileLogger.writeEvent(1,"Timeout:- " + mStateTime.seconds());
                //  Transition to a new state.
                mintCurrentStateGyroTurnEncoder5291 = Constants.stepState.STATE_COMPLETE;
                deleteParallelStep();
            }
            break;
        }
    }

    private void VuforiaLocalise ()
    {

        int intCorrectionAngle = 0;
        String strCorrectionCommand = "DELAY";

        fileLogger.setEventTag("VuforiaLocalise()");
        switch (mintCurrentStateVuforiaLocalise5291) {
            case STATE_INIT: {
                //ensure vision processing is enabled
                mblnDisableVisionProcessing = false;  //enable vision processing
                mintCurrentStateVuforiaLocalise5291 = Constants.stepState.STATE_RUNNING;
                fileLogger.writeEvent(3,"Initialised........");
            }
            break;
            case STATE_RUNNING:
            {
                fileLogger.writeEvent(3, "Running.......");
                fileLogger.writeEvent(3, "localiseRobotPos " + localiseRobotPos );
                if (!localiseRobotPos) {
                    //need to rerun this step as we cannot get localisation and need to adjust robot to see if we can see a target
                    autonomousStepsFile.insertSteps(3, "VFL",0,0,0,0.5,false, false, false, 0,    0,    0,    0,    0,    0,  mintCurrentStep + 1);
                    fileLogger.writeEvent(3,"Not Localised, inserting a new step" );
                    //need a delay, as Vuforia is slow to update
                    autonomousStepsFile.insertSteps(2, "DELAY", 0,0,0,0,false,false, false, 500, 0, 0, 0, 0, 0, mintCurrentStep + 1);

                    //need to adjust robot so we can see target, lets turn robot 180 degrees, if we are facing RED drivers we will end up facing BLUE targets,
                    //if we are facing blue drives we will end up facing RED targets.
                    //if we can't localise we need to abort autonomous so lets try a few things to see if we can localise,
                    // first we will try turning around,
                    // second we will move forward 2 feet
                    // third - abort
                    //Parameter 1 - stop turning once localisation is achieved
                    autonomousStepsFile.insertSteps(3, "RIGHTENCODERTURN",0,0,135, 0.5, true, false, true, 1,    0,    0,    0,    0,    0, mintCurrentStep + 1);
                    mintCurrentStateVuforiaLocalise5291 = Constants.stepState.STATE_COMPLETE;
                    deleteParallelStep();
                    break;
                }

                int intLocalisedRobotBearing = (int)localisedRobotBearing;
                fileLogger.writeEvent(3,"Localised, determining angles.... intLocalisedRobotBearing= " + intLocalisedRobotBearing + " Alliancecolour= " + ourRobotConfig.getAllianceColor());
                //vuforia angles or 0 towards the BLUE drivers, AStar 0 is to the BLUE beacons
                if (ourRobotConfig.getAllianceColor().equals("Red")) {
                    //double check localisation
                    if ((intLocalisedRobotBearing > 3) && (intLocalisedRobotBearing < 177)) {
                        intCorrectionAngle = (180 - intLocalisedRobotBearing);
                        strCorrectionCommand = "LEFTENCODERTURN";
                    } else if ((intLocalisedRobotBearing > 183) && (intLocalisedRobotBearing < 357)) {
                        intCorrectionAngle = (180 - (360 - intLocalisedRobotBearing));
                        strCorrectionCommand = "RIGHTENCODERTURN";
                    } else {
                        mintCurrentStateVuforiaLocalise5291 = Constants.stepState.STATE_COMPLETE;
                        deleteParallelStep();
                        break;
                    }
                }

                if (ourRobotConfig.getAllianceColor().equals("Blue")) {
                    if ((intLocalisedRobotBearing > 273) && (intLocalisedRobotBearing < 360)) {
                        intCorrectionAngle = (intLocalisedRobotBearing - 270);
                        strCorrectionCommand = "LEFTENCODERTURN";
                    } else if ((intLocalisedRobotBearing > 0) && (intLocalisedRobotBearing < 91)) {
                        intCorrectionAngle = (90 + intLocalisedRobotBearing);
                        strCorrectionCommand = "LEFTENCODERTURN";
                    } else if ((intLocalisedRobotBearing > 90) && (intLocalisedRobotBearing < 267)) {
                        intCorrectionAngle = (270 - intLocalisedRobotBearing);
                        strCorrectionCommand = "RIGHTENCODERTURN";
                    } else {
                        mintCurrentStateVuforiaLocalise5291 = Constants.stepState.STATE_COMPLETE;
                        deleteParallelStep();
                        break;
                    }
                }
                //double check localisation
                autonomousStepsFile.insertSteps(3, "VFL",0,0,0,0.5,false, false, false, 0,    0,    0,    0,    0,    0,  mintCurrentStep + 1);
                //need a delay, as Vuforia is slow to update
                autonomousStepsFile.insertSteps(2, "DELAY", 0,0,0,0,false,false, false, 500, 0, 0, 0, 0, 0, mintCurrentStep + 1);
                //load the angle to adjust
                autonomousStepsFile.insertSteps(3, strCorrectionCommand,0,0,intCorrectionAngle,0.5,true, false, false, 0, 0, 0, 0, 0, 0, mintCurrentStep + 1);
                mintCurrentStateVuforiaLocalise5291 = Constants.stepState.STATE_COMPLETE;
                deleteParallelStep();
            }
            //check timeout value
            if (mStateTime.seconds() > mdblStepTimeout)
            {
                fileLogger.writeEvent(1,"Timeout:- " + mStateTime.seconds());
                //  Transition to a new state.
                mintCurrentStateVuforiaLocalise5291 = Constants.stepState.STATE_COMPLETE;
                deleteParallelStep();
            }
            break;
        }
    }

    private void VuforiaMove ()
    {
        int intCorrectionAngle = 0;
        String strCorrectionCommand = "DELAY";

        fileLogger.setEventTag("VuforiaMove()");
        switch (mintCurStVuforiaMove5291) {
            case STATE_INIT: {
                //ensure vision processing is enable
                mblnDisableVisionProcessing = false;  //enable vision processing

                mintCurStVuforiaMove5291 = Constants.stepState.STATE_RUNNING;
                fileLogger.writeEvent(2, "Initialised");
            }
            break;
            case STATE_RUNNING:
            {
                fileLogger.writeEvent(2, "Running" );
                fileLogger.writeEvent(2, "localiseRobotPos " + localiseRobotPos );

                if (!localiseRobotPos)
                {
                    //need to do something gere to try and get localise
                    mintCurStVuforiaMove5291 = Constants.stepState.STATE_COMPLETE;
                    deleteParallelStep();
                    break;
                }

                int currentX = (int)localisedRobotX;
                int currentY = (int)localisedRobotY;
                int intLocalisedRobotBearing = (int)localisedRobotBearing;
                double requiredMoveX = (currentX - (int)mdblRobotParm4);
                double requiredMoveY = (currentY - (int)mdblRobotParm5);

                double requiredMoveDistance = ((Math.sqrt(requiredMoveX * requiredMoveX + requiredMoveY * requiredMoveY)) / 25.4);
                double requiredMoveAngletemp1 = requiredMoveX/requiredMoveY;
                double requiredMoveAngletemp2 = Math.atan(requiredMoveAngletemp1);
                double requiredMoveAngletemp3 = Math.toDegrees(requiredMoveAngletemp2);
                int requiredMoveAngle = (int)Math.abs(requiredMoveAngletemp3);

                fileLogger.writeEvent(2,"Temp Values requiredMoveAngletemp1 " + requiredMoveAngletemp1 + " requiredMoveAngletemp2 " + requiredMoveAngletemp2 + " requiredMoveAngletemp3 " + requiredMoveAngletemp3);
                fileLogger.writeEvent(2,"Temp Values currentX " + currentX + " currentY " + currentY);
                fileLogger.writeEvent(2,"Localised, determining angles....Alliancecolour= " + ourRobotConfig.getAllianceColor() + " intLocalisedRobotBearing= " + intLocalisedRobotBearing + " CurrentX= " + currentX + " CurrentY= " + currentY);
                fileLogger.writeEvent(2,"Localised, determining angles....requiredMoveX " + requiredMoveX + " requiredMoveY " + requiredMoveY);
                fileLogger.writeEvent(2,"Localised, determining angles....requiredMoveDistance " + requiredMoveDistance + " requiredMoveAngle " + requiredMoveAngle);

                if ((((int) mdblRobotParm5) > currentY) && ((int) mdblRobotParm4 > currentX)) {
                    requiredMoveAngle = 90 - requiredMoveAngle;
                } else if ((((int) mdblRobotParm5) > currentY) && ((int) mdblRobotParm4 < currentX)) {
                    requiredMoveAngle =  90 + requiredMoveAngle;
                } else if ((((int) mdblRobotParm5) < currentY) && ((int) mdblRobotParm4 > currentX)) {
                    requiredMoveAngle = 270 + requiredMoveAngle;
                } else if ((((int) mdblRobotParm5) < currentY) && ((int) mdblRobotParm4 < currentX)) {
                    requiredMoveAngle = 270 - requiredMoveAngle;
                }

                intCorrectionAngle = TOWR5291Utils.getNewHeading((int)localisedRobotBearing, (int)mdblRobotParm1);

                autonomousStepsFile.insertSteps(3, "FORWARD", requiredMoveDistance,0,0,0.6, false, false, false, 0, 0, 0, 0, 0, 0,  mintCurrentStep + 1);
                if (intCorrectionAngle < 0) {
                    autonomousStepsFile.insertSteps(3, "LEFTTURN", 0,0, intCorrectionAngle,.5,true, false, false, 0, 0, 0, 0, 0, 0, mintCurrentStep + 1);

                } else if (intCorrectionAngle > 0) {
                    autonomousStepsFile.insertSteps(3, "RIGHTTURN", 0,0, intCorrectionAngle,.5,true, false, false, 0, 0, 0, 0, 0, 0, mintCurrentStep + 1);
                }
                mintCurStVuforiaMove5291 = Constants.stepState.STATE_COMPLETE;
                deleteParallelStep();
            }
            //check timeout value
            if (mStateTime.seconds() > mdblStepTimeout)
            {
                fileLogger.writeEvent(1,"Timeout:- "  + mStateTime.seconds());
                //  Transition to a new state.
                mintCurStVuforiaMove5291 = Constants.stepState.STATE_COMPLETE;
                deleteParallelStep();
            }
            break;
        }
    }

    private void VuforiaTurn ()
    {
        fileLogger.setEventTag("VuforiaTurn()");
        int intCorrectionAngle;
        switch (mintCurStVuforiaTurn5291) {
            case STATE_INIT: {
                //ensure vision processing is enabled
                mblnDisableVisionProcessing     = false;  //enable vision processing
                mintCurStVuforiaTurn5291        = Constants.stepState.STATE_RUNNING;
                fileLogger.writeEvent(2,"Initialised");
            }
            break;
            case STATE_RUNNING:
            {
                fileLogger.writeEvent(2,"Running" );
                fileLogger.writeEvent(2,"localiseRobotPos " + localiseRobotPos );

                if (!localiseRobotPos)
                {
                    //need to do something here to try and get localised
                    mintCurStVuforiaTurn5291    = Constants.stepState.STATE_COMPLETE;
                    deleteParallelStep();
                    break;
                }
                intCorrectionAngle = TOWR5291Utils.getNewHeading((int)localisedRobotBearing, (int)mdblRobotParm1);
                if (intCorrectionAngle < 0) {
                    autonomousStepsFile.insertSteps(3, "LEFTTURN", 0,0,intCorrectionAngle,.5,true, false, false, 0, 0, 0, 0, 0, 0, mintCurrentStep + 1);

                } else if (intCorrectionAngle > 0) {
                    autonomousStepsFile.insertSteps(3, "RIGHTTURN", 0,0,intCorrectionAngle,.5,true, false, false, 0, 0, 0, 0, 0, 0, mintCurrentStep + 1);
                }
                fileLogger.writeEvent(2,"Localised, determining angles....Alliancecolour= " + ourRobotConfig.getAllianceColor() + " localisedRobotBearing= " + localisedRobotBearing  + " requiredMoveAngle " + mdblRobotParm1);
                mintCurStVuforiaTurn5291 = Constants.stepState.STATE_COMPLETE;
                deleteParallelStep();
            }
            //check timeout value
            if (mStateTime.seconds() > mdblStepTimeout)
            {
                fileLogger.writeEvent(1, "Timeout:- " + mStateTime.seconds());
                //  Transition to a new state.
                mintCurStVuforiaTurn5291 = Constants.stepState.STATE_COMPLETE;
                deleteParallelStep();
            }
            break;
        }
    }

    private void DelayStep ()
    {
        fileLogger.setEventTag("DelayStep()");
        switch (mintCurrentStepDelay) {
            case STATE_INIT: {
                mintStepDelay = (int)(mdblRobotParm1);
                mintCurrentStepDelay = Constants.stepState.STATE_RUNNING;
                fileLogger.writeEvent(3,"Init Delay Time......." + mintStepDelay);
            }
            break;
            case STATE_RUNNING:
            {
                if (mStateTime.milliseconds() >= mintStepDelay)
                {
                    fileLogger.writeEvent(1,"Complete.......");
                    mintCurrentStepDelay = Constants.stepState.STATE_COMPLETE;
                    deleteParallelStep();
                }
            }
            //check timeout value
            if (mStateTime.seconds() > mdblStepTimeout)
            {
                fileLogger.writeEvent(1,"Timeout:- " + mStateTime.seconds());
                //  Transition to a new state.
                mintCurrentStepDelay = Constants.stepState.STATE_COMPLETE;
                deleteParallelStep();
            }
            break;
        }
    }

    private double teamAngleAdjust ( double angle ) {
        fileLogger.setEventTag("teamAngleAdjust()");
        fileLogger.writeEvent(2,"teamAngleAdjust - angle " + angle + " allianceColor " + ourRobotConfig.getAllianceColor());

        if (ourRobotConfig.getAllianceColor().equals("Red")) {
            //angle = angle + 90;  if starting against the wall
            //angle = angle + 225; if starting at 45 to the wall facing the beacon
            angle = angle + 225;
            if (angle > 360) {
                angle = angle - 360;
            }
            fileLogger.writeEvent(2,"In RED Angle " + angle);

        } else
        if (ourRobotConfig.getAllianceColor().equals("Blue")) {
            //angle = angle - 180;;  if starting against the wall
            angle = angle - 135;
            if (angle < 0) {
                angle = angle + 360;
            }
        }
        return angle;
    }

    private String newAngleDirectionGyro (int currentDirection, int newDirection)
    {
        fileLogger.setEventTag("newAngleDirectionGyro()");
        int intAngle1;

        //calculate the smallest angle
        if (currentDirection < newDirection) {
            intAngle1 = (newDirection - currentDirection);
            if (intAngle1 > 180)
            {
                intAngle1 = (currentDirection + 360 - newDirection);
                return "LTE" + intAngle1;
            }
            return "RTE" + intAngle1;
        }
        else
        {
            intAngle1 = (currentDirection - newDirection);
            if (intAngle1 > 180)
            {
                intAngle1 = (newDirection + 360 - currentDirection);
                return "RTE" + intAngle1;
            }
            return "LTE" + intAngle1;
        }
    }

    private String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

    /**
     * Converts a reading of the optical sensor into centimeters. This computation
     * could be adjusted by altering the numeric parameters, or by providing an alternate
     * calculation in a subclass.
     */
    private double cmFromOptical(int opticalReading)
    {
        double pParam = -1.02001;
        double qParam = 0.00311326;
        double rParam = -8.39366;
        int    sParam = 10;

        if (opticalReading < sParam)
            return 0;
        else
            return pParam * Math.log(qParam * (rParam + opticalReading));
    }

    private int cmUltrasonic(int rawUS)
    {
        return rawUS;
    }

    private double cmOptical(int rawOptical)
    {
        return cmFromOptical(rawOptical);
    }

    public double getDistance(int rawUS, int rawOptical, DistanceUnit unit)
    {
        double cmOptical = cmOptical(rawOptical);
        double cm        = cmOptical > 0 ? cmOptical : cmUltrasonic(rawUS);
        return unit.fromUnit(DistanceUnit.CM, cm);
    }

    private void readRangeSensors()
    {
        fileLogger.setEventTag("readRangeSensors()");
        //adds 100ms to scan time, try use this as little as possible
        range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
        mdblRangeSensor1 = getDistance(range1Cache[0] & 0xFF, range1Cache[1] & 0xFF, DistanceUnit.CM);
        mdblRangeSensor2 = getDistance(range2Cache[0] & 0xFF, range2Cache[1] & 0xFF, DistanceUnit.CM);
        fileLogger.writeEvent(2,"mdblRangeSensor1 " + mdblRangeSensor1 + ",mdblRangeSensor2 " + mdblRangeSensor2);
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    private double getDriveError(double targetAngle) {
        fileLogger.setEventTag("getDriveError()");
        double robotError;
        double robotErrorIMU;
        double robotErrorGyro;
        double MRgyroHeading;
        double adafruitIMUHeading;

        adafruitIMUHeading = getAdafruitHeading();

        fileLogger.writeEvent(2,"targetAngle " + targetAngle);
        fileLogger.writeEvent(2,"Adafruit IMU Reading " + adafruitIMUHeading);
        // calculate error in -179 to +180 range  (
        robotErrorIMU = targetAngle - teamAngleAdjust(adafruitIMUHeading);
        robotError = robotErrorIMU;
        fileLogger.writeEvent(2,"USING HEADING FROM IMU=" + useAdafruitIMU);
        fileLogger.writeEvent(2,"robotErrorIMU " + robotError + ", getAdafruitHeading() " + adafruitIMUHeading + " teamAngleAdjust(adafruitIMUHeading) "  + teamAngleAdjust(adafruitIMUHeading));

        if (robotError > 180)
            robotError -= 360;
        if (robotError <= -180)
            robotError += 360;

        fileLogger.writeEvent(2,"robotError2 " + robotError);
        return -robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     */
    private double getDriveSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
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

    private void sendServosHome(Servo servoGlyphGripTopLeft, Servo servoGlyphGripBotLeft, Servo servoGlyphGripTopRight, Servo servoGlyphGripBotRight, Servo servoJewelLeft, Servo servoJewelRight) {
        moveServo(servoGlyphGripTopLeft, SERVOLIFTLEFTTOP_HOME, SERVOLIFTLEFTTOP_MIN_RANGE, SERVOLIFTLEFTTOP_MAX_RANGE);
        moveServo(servoGlyphGripBotLeft, SERVOLIFTLEFTBOT_HOME, SERVOLIFTLEFTBOT_MIN_RANGE, SERVOLIFTLEFTBOT_MAX_RANGE);
        moveServo(servoGlyphGripTopRight, SERVOLIFTRIGHTTOP_HOME, SERVOLIFTRIGHTTOP_MIN_RANGE, SERVOLIFTRIGHTTOP_MAX_RANGE);
        moveServo(servoGlyphGripBotRight, SERVOLIFTRIGHTBOT_HOME, SERVOLIFTRIGHTBOT_MIN_RANGE, SERVOLIFTRIGHTBOT_MAX_RANGE);
        moveServo(servoJewelLeft, SERVOJEWELLEFT_HOME, SERVOJEWELLEFT_MIN_RANGE, SERVOJEWELLEFT_MAX_RANGE);
        moveServo(servoJewelRight, SERVOJEWELRIGHT_HOME, SERVOJEWELRIGHT_MIN_RANGE, SERVOJEWELRIGHT_MAX_RANGE);
    }

    private void moveTopServos(Servo servoGlyphGripTopLeft, Servo servoGlyphGripTopRight, double positionTopLeft, double positionTopRight) {
        moveServo(servoGlyphGripTopLeft, positionTopLeft, SERVOLIFTLEFTTOP_MIN_RANGE, SERVOLIFTLEFTTOP_MAX_RANGE);
        moveServo(servoGlyphGripTopRight, positionTopRight, SERVOLIFTRIGHTTOP_MIN_RANGE, SERVOLIFTRIGHTTOP_MAX_RANGE);
    }

    private void moveBotServos(Servo servoGlyphGripBotLeft, Servo servoGlyphGripBotRight, double positionBotLeft, double positionBotRight) {
        moveServo(servoGlyphGripBotLeft, positionBotLeft, SERVOLIFTLEFTBOT_MIN_RANGE, SERVOLIFTLEFTBOT_MAX_RANGE);
        moveServo(servoGlyphGripBotRight, positionBotRight, SERVOLIFTRIGHTBOT_MIN_RANGE, SERVOLIFTRIGHTBOT_MAX_RANGE);
    }

    private void moveServosPair(Servo servoLeft, Servo servoRight, double positionLeft, double positionRight) {
        moveServo(servoLeft, positionLeft, 0, 180);
        moveServo(servoRight, positionRight, 0, 180);
    }

    private boolean moveServo (Servo Servo, double Position, double RangeMin, double RangeMax ) {
        //if ((Range.scale(Position, 0, 180, 0, 1) < RangeMin ) || (Range.scale(Position, 0, 180, 0, 1) > RangeMax )) {
        //    return false;
        //}
        if ((Position < RangeMin ) || (Position > RangeMax )) {
            return false;
        }

        Servo.setPosition(Range.scale(Position, 0, 180, 0, 1));
        return true;
    }

    private boolean checkAllStatesComplete () {
        if ((mintCurrentStepDelay                       == Constants.stepState.STATE_COMPLETE) &&
                (mintCurStVuforiaTurn5291               == Constants.stepState.STATE_COMPLETE) &&
                (mintCurrentStateVuforiaLocalise5291    == Constants.stepState.STATE_COMPLETE) &&
                (mintCurStVuforiaMove5291               == Constants.stepState.STATE_COMPLETE) &&
                (mintCurrentStateDrive                  == Constants.stepState.STATE_COMPLETE) &&
                (mintCurrentStateDriveHeading           == Constants.stepState.STATE_COMPLETE) &&
                (mintCurrentStatePivotTurn              == Constants.stepState.STATE_COMPLETE) &&
                (mintCurrentStateTankTurn               == Constants.stepState.STATE_COMPLETE) &&
                (mintCurrentStateEyes5291               == Constants.stepState.STATE_COMPLETE) &&
                (mintCurrentStateGyroTurnEncoder5291    == Constants.stepState.STATE_COMPLETE) &&
                (mintCurrentStateTankTurnGyroHeading    == Constants.stepState.STATE_COMPLETE) &&
                (mintCurrentStateMecanumStrafe          == Constants.stepState.STATE_COMPLETE) &&
                (mintCurrentStateRadiusTurn             == Constants.stepState.STATE_COMPLETE)) {
            return true;
        }
        return false;
    }

    private void initDefaultStates() {
        mintCurrentStateStep                = Constants.stepState.STATE_INIT;
        mintCurrentStateTankTurn            = Constants.stepState.STATE_COMPLETE;
        mintCurrentStateDrive               = Constants.stepState.STATE_COMPLETE;
        mintCurrentStateDriveHeading        = Constants.stepState.STATE_COMPLETE;
        mintCurrentStatePivotTurn           = Constants.stepState.STATE_COMPLETE;
        mintCurrentStateRadiusTurn          = Constants.stepState.STATE_COMPLETE;
        mintCurrentStepDelay                = Constants.stepState.STATE_COMPLETE;
        mintCurrentStateVuforiaLocalise5291 = Constants.stepState.STATE_COMPLETE;
        mintCurStVuforiaMove5291            = Constants.stepState.STATE_COMPLETE;
        mintCurStVuforiaTurn5291            = Constants.stepState.STATE_COMPLETE;
        mintCurrentStateEyes5291            = Constants.stepState.STATE_COMPLETE;
        mintCurrentStateTankTurnGyroHeading = Constants.stepState.STATE_COMPLETE;
        mintCurrentStateMecanumStrafe       = Constants.stepState.STATE_COMPLETE;
        mintCurrentStateGyroTurnEncoder5291 = Constants.stepState.STATE_COMPLETE;

    }
}