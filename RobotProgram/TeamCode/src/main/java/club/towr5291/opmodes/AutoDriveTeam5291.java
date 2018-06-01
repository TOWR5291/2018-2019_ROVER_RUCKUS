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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Point3;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import club.towr5291.R;
import club.towr5291.astarpathfinder.A0Star;
import club.towr5291.astarpathfinder.sixValues;
import club.towr5291.functions.AStarGetPathVer2;
import club.towr5291.functions.BeaconAnalysisOCV2;
import club.towr5291.functions.Constants;
import club.towr5291.functions.FileLogger;
import club.towr5291.functions.JewelAnalysisOCV;
import club.towr5291.functions.ReadStepFile;
import club.towr5291.libraries.robotConfigSettings;
import club.towr5291.libraries.LibraryStateSegAuto;
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
@Autonomous(name="5291 Autonomous Drive", group="5291")
public class AutoDriveTeam5291 extends OpModeMasterLinear {
    final int LABEL_WIDTH = 200;

    //set up TAG for logging prefic, this info will appear first in every log statemend
    private static final String TAG = "AutoDriveTeam5291";

    //variable for pathvalues when processing the A*pathfinder
    private AStarGetPathVer2 getPathValues = new AStarGetPathVer2();

    //The autonomous menu settings from the sharepreferences
    private SharedPreferences sharedPreferences;
    private String teamNumber;
    private String allianceColor;
    private String allianceStartPosition;
    private int delay;
    private String robotConfig;

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

    //set up robot variables
    private double COUNTS_PER_MOTOR_REV;            // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
    private double DRIVE_GEAR_REDUCTION;            // This is < 1.0 if geared UP, Tilerunner is geared up
    private double WHEEL_DIAMETER_INCHES;           // For figuring circumference
    private double WHEEL_ACTUAL_FUDGE;              // Fine tuning amount
    private double COUNTS_PER_INCH;
    private double ROBOT_TRACK;                     //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
    private double COUNTS_PER_DEGREE;
    private double WHEEL_TURN_FUDGE;
    private double REVERSE_DIRECTION;               // determines which directin the robot runs when FW is positive or negative when commanded to move a direction
    private int LIFTMAIN_COUNTS_PER_INCH;           //number of encoder counts oer inch
    private int LIFTTOP_COUNTS_PER_INCH;            //number of encoder counts oer inch
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
    private int mintCurrentStep = 1;                                                        // Current Step in State Machine.
    private int mintCurrentStepAStar = 1;                                                   // Current Step in AStar State Machine.
    private stepState mintCurrentStateStep;                                                 // Current State Machine State.
    private stepState mintCurrentStateDrive;                                                // Current State of Drive.
    private stepState mintCurrentStateDriveHeading;                                         // Current State of Drive Heading.
    private stepState mintCurrentStateTankTurn;                                             // Current State of Tank Turn.
    private stepState mintCurrentStatePivotTurn;                                            // Current State of Pivot Turn.
    private stepState mintCurrentStateRadiusTurn;                                           // Current State of Radius Turn.
    private stepState mintCurrentStateVuforiaLocalise5291;                                  // Current State of Vuforia Localisation
    private stepState mintCurStVuforiaMove5291;                                             // Current State of Vuforia Move
    private stepState mintCurStVuforiaTurn5291;                                             // Current State of Vuforia Turn
    private stepState mintCurrentStateJewelColour5291;                                      // Current State Detecting Jewel
    private stepState mintCurrentStateJewelArm5291;                                         // Current State of moving Jewel Arm Out
    private stepState mintCurrentStateTopGripperMove5291;                                   // Current State of moving the top Glyph Gripper
    private stepState mintCurrentStateBotGripperMove5291;                                   // Current State of moving the bottom Glyph Gripper
    private stepState mintCurrentStateTopLiftMove5291;                                      // Current State of Moving the top lift
    private stepState mintCurrentStateMainLiftMove5291;                                     // Current State of moving the main lift
    private stepState mintCurrentStateJewelArmClose5291;                                    // Current State of moving Jewel Arm Home
    private stepState mintCurrentStateGyroTurnEncoder5291;                                  // Current State of the Turn function that take the Gyro as an initial heading
    private stepState mintCurrentStateEyes5291;                                             // Current State of the Eyelids
    private stepState mintCurrentStateTankTurnGyroHeading;                                  // Current State of Tank Turn using Gyro
    private stepState mintCurrentStateMecanumStrafe;                                        // Current State of mecanum strafe
    private stepState mintCurrentStepDelay;                                                 // Current State of Delay (robot doing nothing)
    //private ArrayList<LibraryStateTrack> mValueSteps    = new ArrayList<>();              // Current State of the Step
    private HashMap<String, Integer> mintActiveSteps = new HashMap<>();
    private HashMap<String, Integer> mintActiveStepsCopy = new HashMap<>();

    //motors
    private HardwareDriveMotors robotDrive = new HardwareDriveMotors();   // Use a Pushbot's hardware
    private HardwareDriveMotors armDrive = new HardwareDriveMotors();   // Use a Pushbot's hardware

    //variable for the state engine, declared here so they are accessible throughout the entire opmode with having to pass them through each function
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
    private double mdblStepTimeout;                          //Timeout value ofthe step, the step will abort if the timeout is reached
    private double mdblStepSpeed;                            //When a move command is executed this is the speed the motors will run at
    private String mstrRobotCommand;                         //The command the robot will execute, such as move forward, turn right etc
    private double mdblRobotParm1;                           //First Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mdblRobotParm2;                           //Second Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mdblRobotParm3;                           //Third Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mdblRobotParm4;                           //Fourth Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mdblRobotParm5;                           //Fifth Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mdblRobotParm6;                           //Sixth Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mdblStepTurnL;                            //used when decoding the step, this will indicate if the robot is turning left
    private double mdblStepTurnR;                            //used when decoding the step, this will indicate if the robot is turning right
    private double mdblRobotTurnAngle;                       //used to determine angle the robot will turn
    private double mdblStepDistance;                         //used when decoding the step, this will indicate how far the robot is to move in inches
    private boolean mblnParallel;                            //used to determine if next step will run in parallel - at same time
    private boolean mblnRobotLastPos;                        //used to determine if next step will run from end of last step or from encoder position
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

    //hashmap for the steps to be stored in.  A Hashmap is like a fancy array
    private HashMap<String, LibraryStateSegAuto> autonomousSteps = new HashMap<String, LibraryStateSegAuto>();
    private HashMap<String, String> powerTable = new HashMap<String, String>();
    private ReadStepFile autonomousStepsFile = new ReadStepFile();

    //OpenCV Stuff
    private JewelAnalysisOCV JewelColour = new JewelAnalysisOCV();
    ;
    private int mintCaptureLoop = 0;
    private int mintNumberColourTries = 0;
    private Constants.ObjectColours mColour;

    //servos
    // the servos are on the servo controller
    private final static double SERVOLIFTLEFTTOP_MIN_RANGE = 0;
    private final static double SERVOLIFTLEFTTOP_MAX_RANGE = 180;
    private final static double SERVOLIFTLEFTTOP_HOME = 165; //90
    private final static double SERVOLIFTLEFTTOP_GLYPH_START = 125;  //need to work this out
    private final static double SERVOLIFTLEFTTOP_GLYPH_RELEASE = 60;
    private final static double SERVOLIFTLEFTTOP_GLYPH_GRAB = 30;

    private final static double SERVOLIFTRIGHTTOP_MIN_RANGE = 0;
    private final static double SERVOLIFTRIGHTTOP_MAX_RANGE = 180;
    private final static double SERVOLIFTRIGHTTOP_HOME = 165;
    private final static double SERVOLIFTRIGHTTOP_GLYPH_START = 125;  //need to work this out
    private final static double SERVOLIFTRIGHTTOP_GLYPH_RELEASE = 60;
    private final static double SERVOLIFTRIGHTTOP_GLYPH_GRAB = 30;

    private final static double SERVOLIFTLEFTBOT_MIN_RANGE = 0;
    private final static double SERVOLIFTLEFTBOT_MAX_RANGE = 180;
    private final static double SERVOLIFTLEFTBOT_HOME = 165;
    private final static double SERVOLIFTLEFTBOT_GLYPH_START = 125;  //need to work this out
    private final static double SERVOLIFTLEFTBOT_GLYPH_RELEASE = 60;
    private final static double SERVOLIFTLEFTBOT_GLYPH_GRAB = 30;

    private final static double SERVOLIFTRIGHTBOT_MIN_RANGE = 0;
    private final static double SERVOLIFTRIGHTBOT_MAX_RANGE = 180;
    private final static double SERVOLIFTRIGHTBOT_HOME = 165;
    private final static double SERVOLIFTRIGHTBOT_GLYPH_START = 125;  //need to work this out
    private final static double SERVOLIFTRIGHTBOT_GLYPH_RELEASE = 60;
    private final static double SERVOLIFTRIGHTBOT_GLYPH_GRAB = 30;

    private final static double SERVOJEWELLEFT_MIN_RANGE = 0;
    private final static double SERVOJEWELLEFT_MAX_RANGE = 180;
    private final static double SERVOJEWELLEFT_HOME = 147;
    private final static double SERVOJEWELRIGHT_MIN_RANGE = 4;
    private final static double SERVOJEWELRIGHT_MAX_RANGE = 180;
    private final static double SERVOJEWELRIGHT_HOME = 150;
    private Servo servoGlyphGripTopLeft;
    private Servo servoGlyphGripBotLeft;
    private Servo servoGlyphGripTopRight;
    private Servo servoGlyphGripBotRight;
    private Servo servoJewelLeft;
    private Servo servoJewelRight;

    //LED Strips
    private DeviceInterfaceModule dim;                  // Device Object
    DigitalChannel green1LedChannel;
    DigitalChannel red1LedChannel;
    DigitalChannel blue1LedChannel;
    DigitalChannel green2LedChannel;
    DigitalChannel red2LedChannel;
    DigitalChannel blue2LedChannel;

    //Limit Switches
    DigitalChannel limitswitch1;  // Hardware Device Object
    DigitalChannel limitswitch2;  // Hardware Device Object

    private double mdblLastOn;
    private double mdblLastOff;
    private boolean mblnLEDON;
    private int mintCounts = 0;

    private LEDState mint5291LEDStatus;                                                   // Flash the LED based on the status
    private final boolean LedOn = false;
    private boolean LedOff = true;

    //load variables
    LibraryStateSegAuto processingSteps = new LibraryStateSegAuto(0, 0, "", false, false, 0, 0, 0, 0, 0, 0, 0);
    sixValues[] pathValues = new sixValues[1000];
    A0Star a0Star = new A0Star();
    String fieldOutput;
    HashMap<String, LibraryStateSegAuto> autonomousStepsAStar = new HashMap<>();

    private static HalDashboard dashboard = null;

    public static HalDashboard getDashboard() {
        return dashboard;
    }

    private enum LEDState {
        STATE_ERROR,
        STATE_TEAM,
        STATE_MOVING,
        STATE_OBJECT,
        STATE_SUCCESS,
        STATE_FINISHED
    }

    private enum stepState {
        STATE_INIT,
        STATE_START,
        STATE_RUNNING,
        STATE_PAUSE,
        STATE_COMPLETE,
        STATE_TIMEOUT,
        STATE_ERROR,
        STATE_FINISHED,
        STATE_ASTAR_PRE_INIT,
        STATE_ASTAR_INIT,
        STATE_ASTAR_RUNNING,
        STATE_ASTAR_ERROR,
        STATE_ASTAR_COMPLETE
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
        fileLogger.writeEvent(TAG, "Log Started");
        runtime.reset();
        dashboard.displayPrintf(1, "FileLogger: Started");

        //init openCV
        initOpenCv();
        dashboard.displayPrintf(1, "initRobot OpenCV!");

        {
            fileLogger.writeEvent(3, TAG, "OpenCV Started");
        }

        //load menu settings and setup robot and debug level
        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        teamNumber = sharedPreferences.getString("club.towr5291.Autonomous.TeamNumber", "0000");
        allianceColor = sharedPreferences.getString("club.towr5291.Autonomous.Color", "Red");
        allianceStartPosition = sharedPreferences.getString("club.towr5291.Autonomous.StartPosition", "Left");
        delay = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Delay", "0"));
        robotConfig = sharedPreferences.getString("club.towr5291.Autonomous.RobotConfig", "TileRunnerMecanum2x40");
        debug = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", "1"));

        //adjust debug level
        fileLogger.setDebugLevel(debug);

        dashboard.displayPrintf(3, "robotConfigTeam # " + teamNumber);
        dashboard.displayPrintf(4, "Alliance          " + allianceColor);
        dashboard.displayPrintf(5, "Start Pos         " + allianceStartPosition);
        dashboard.displayPrintf(6, "Start Del         " + delay);
        dashboard.displayPrintf(7, "Robot             " + robotConfig);
        dashboard.displayPrintf(7, "Debug Level       " + debug);

        dashboard.displayPrintf(1, "initRobot SharePreferences!");

        // get a reference to a Modern Robotics DIM, and IO channels.
        dim = hardwareMap.get(DeviceInterfaceModule.class, "dim");   //  Use generic form of device mapping
        green1LedChannel = hardwareMap.get(DigitalChannel.class, "green1");    //  Use generic form of device mapping
        red1LedChannel = hardwareMap.get(DigitalChannel.class, "red1");    //  Use generic form of device mapping
        blue1LedChannel = hardwareMap.get(DigitalChannel.class, "blue1");    //  Use generic form of device mapping
        green2LedChannel = hardwareMap.get(DigitalChannel.class, "green2");    //  Use generic form of device mapping
        red2LedChannel = hardwareMap.get(DigitalChannel.class, "red2");    //  Use generic form of device mapping
        blue2LedChannel = hardwareMap.get(DigitalChannel.class, "blue2");    //  Use generic form of device mapping
        green1LedChannel.setMode(DigitalChannel.Mode.OUTPUT);
        red1LedChannel.setMode(DigitalChannel.Mode.OUTPUT);
        blue1LedChannel.setMode(DigitalChannel.Mode.OUTPUT);
        green2LedChannel.setMode(DigitalChannel.Mode.OUTPUT);
        red2LedChannel.setMode(DigitalChannel.Mode.OUTPUT);
        blue2LedChannel.setMode(DigitalChannel.Mode.OUTPUT);

        LedState(LedOn, LedOn, LedOn, LedOn, LedOn, LedOn);

        dashboard.displayPrintf(1, "initRobot LED Initiated!");

        // get a reference to our digitalTouch object.
        limitswitch1 = hardwareMap.get(DigitalChannel.class, "limittop");
        limitswitch2 = hardwareMap.get(DigitalChannel.class, "limitbot");
        // set the digital channel to input.
        limitswitch1.setMode(DigitalChannel.Mode.INPUT);
        limitswitch2.setMode(DigitalChannel.Mode.INPUT);

        dashboard.displayPrintf(1, "initRobot Limit Switch Initiated!");

        //to add more config options edit strings.xml and AutonomousConfiguration.java
        switch (robotConfig) {
            case "TileRunner-2x40":   //Velocity Vortex Competition Base
                REVERSE_DIRECTION = 1;                                                       // Reverse the direction without significant code changes, (using motor FORWARD REVERSE will affect the driver station as we use same robotconfig file
                COUNTS_PER_MOTOR_REV = 1120;                                                    // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
                DRIVE_GEAR_REDUCTION = 0.7;                                                    // This is < 1.0 if geared UP, Tilerunner is geared up
                WHEEL_DIAMETER_INCHES = 4.0;                                                     // For figuring circumference
                WHEEL_ACTUAL_FUDGE = 1;                                                        // Fine tuning amount
                COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)) * WHEEL_ACTUAL_FUDGE * REVERSE_DIRECTION;
                ROBOT_TRACK = 16.5;                                                     //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
                WHEEL_TURN_FUDGE = 1.0;                                                        // Fine tuning amount
                COUNTS_PER_DEGREE = (((2 * 3.1415 * ROBOT_TRACK) * COUNTS_PER_INCH) / 360) * WHEEL_TURN_FUDGE;
                loadPowerTableTileRunner();                                                         //load the power table
                break;
            case "5291 Tank Tread-2x40 Custom":   //for tank tread base
                REVERSE_DIRECTION = 1;
                COUNTS_PER_MOTOR_REV = 1120;                                                    // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
                DRIVE_GEAR_REDUCTION = 1.0;                                                     // Tank Tread is 1:1 ration
                WHEEL_DIAMETER_INCHES = 3.75;                                                     // For figuring circumference
                WHEEL_ACTUAL_FUDGE = 1;                                                        // Fine tuning amount
                COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)) * WHEEL_ACTUAL_FUDGE * REVERSE_DIRECTION;
                ROBOT_TRACK = 18;                                                     //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
                WHEEL_TURN_FUDGE = 1.12;                                                        // Fine tuning amount
                COUNTS_PER_DEGREE = (((2 * 3.1415 * ROBOT_TRACK) * COUNTS_PER_INCH) / 360) * WHEEL_TURN_FUDGE;
                loadPowerTableTankTread();                                                          //load the power table
                break;
            case "TileRunnerMecanum2x40":
                REVERSE_DIRECTION = 1;                                                        // Reverse the direction without significant code changes, (using motor FORWARD REVERSE will affect the driver station as we use same robotconfig file
                COUNTS_PER_MOTOR_REV = 1120;                                                    // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
                DRIVE_GEAR_REDUCTION = 1.0;                                                     // This is < 1.0 if geared UP, Tilerunner is geared up
                WHEEL_DIAMETER_INCHES = 4.0;                                                     // For figuring circumference
                WHEEL_ACTUAL_FUDGE = 1.02;                                                     // Fine tuning amount
                COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)) * WHEEL_ACTUAL_FUDGE * REVERSE_DIRECTION;
                ROBOT_TRACK = 16.5;                                                     //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
                WHEEL_TURN_FUDGE = 1.0;                                                        // Fine tuning amount
                COUNTS_PER_DEGREE = (((2 * 3.1415 * ROBOT_TRACK) * COUNTS_PER_INCH) / 360) * WHEEL_TURN_FUDGE;
                LIFTMAIN_COUNTS_PER_INCH = 420;                                                   //number of encoder counts per inch
                LIFTTOP_COUNTS_PER_INCH = -420;                                                   //number of encoder counts per inch
                break;
            case "11231 2016 Custom": //2016 - 11231 Drivetrain
                COUNTS_PER_MOTOR_REV = 1120;                                                     // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
                DRIVE_GEAR_REDUCTION = .667;                                                   // (.665) UP INCREASES THE DISTANCE This is < 1.0 if geared UP, Tilerunner is geared up
                WHEEL_DIAMETER_INCHES = 4.0;                                                     // For figuring circumference
                WHEEL_ACTUAL_FUDGE = 1;                                                        // Fine tuning amount
                COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415926535)) * WHEEL_ACTUAL_FUDGE;
                ROBOT_TRACK = 18;                                                     //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
                COUNTS_PER_DEGREE = ((2 * 3.1415926535 * ROBOT_TRACK) * COUNTS_PER_INCH) / 360;
                //loadPowerTableTileRunner();                                                         //load the power table
                break;
            default:  //default for competition TileRunner-2x40
                REVERSE_DIRECTION = 1;
                COUNTS_PER_MOTOR_REV = 1120;                                                     // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
                DRIVE_GEAR_REDUCTION = 1.28;                                                    // This is < 1.0 if geared UP, Tilerunner is geared up
                WHEEL_DIAMETER_INCHES = 4.0;                                                     // For figuring circumference
                WHEEL_ACTUAL_FUDGE = 1;                                                        // Fine tuning amount
                COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)) * WHEEL_ACTUAL_FUDGE * REVERSE_DIRECTION;
                ROBOT_TRACK = 16.5;                                                     //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
                COUNTS_PER_DEGREE = ((2 * 3.1415 * ROBOT_TRACK) * COUNTS_PER_INCH) / 360;
                loadPowerTableTileRunner();                                                         //load the power table
                break;
        }

        dashboard.displayPrintf(1, "initRobot Robot Settings Loaded" + robotConfig);

        fileLogger.writeEvent(1, TAG, "robotConfigTeam #  " + teamNumber);
        fileLogger.writeEvent(1, TAG, "Alliance Colour    " + allianceColor);
        fileLogger.writeEvent(1, TAG, "Alliance Start Pos " + allianceStartPosition);
        fileLogger.writeEvent(1, TAG, "Alliance Delay     " + delay);
        fileLogger.writeEvent(1, TAG, "Robot Config       " + robotConfig);
        fileLogger.writeEvent(3, TAG, "Configuring Robot Parameters - Finished");
        fileLogger.writeEvent(3, TAG, "Loading Autonomous Steps - Start");

        dashboard.displayPrintf(1, "initRobot Loading Steps " + allianceColor + " Team " + teamNumber);

        //load the sequence based on alliance colour and team
        switch (teamNumber) {
            case "5291":
                switch (allianceColor) {
                    case "Red":
                        LedState(LedOff, LedOn, LedOff, LedOff, LedOn, LedOff);
                        switch (allianceStartPosition) {
                            case "Left":
                                autonomousSteps = autonomousStepsFile.ReadStepFileRelicRecovery("5291RedLeft.csv", "none", "none");
                                break;
                            case "Right":
                                autonomousSteps = autonomousStepsFile.ReadStepFileRelicRecovery("5291RedRight.csv", "none", "none");
                                break;
                        }
                        break;
                    case "Blue":
                        LedState(LedOff, LedOff, LedOn, LedOff, LedOff, LedOn);
                        switch (allianceStartPosition) {
                            case "Left":
                                autonomousSteps = autonomousStepsFile.ReadStepFileRelicRecovery("5291BlueLeft.csv", "none", "none");
                                break;
                            case "Right":
                                autonomousSteps = autonomousStepsFile.ReadStepFileRelicRecovery("5291BlueRight.csv", "none", "none");
                                break;
                        }
                        break;
                    case "Test":
                        autonomousSteps = autonomousStepsFile.ReadStepFileRelicRecovery("5291Test.csv", "none", "none");
                        break;
                }
                break;

            case "11230":
                switch (allianceColor) {
                    case "Red":
                        switch (allianceStartPosition) {
                            case "Left":
                                autonomousSteps = autonomousStepsFile.ReadStepFileRelicRecovery("11230RedLeft.csv", "none", "none");
                                break;
                            case "Right":
                                autonomousSteps = autonomousStepsFile.ReadStepFileRelicRecovery("11230RedRight.csv", "none", "none");
                                break;
                        }
                        break;
                    case "Blue":
                        switch (allianceStartPosition) {
                            case "Left":
                                autonomousSteps = autonomousStepsFile.ReadStepFileRelicRecovery("5291BlueRight.csv", "none", "none");
                                break;
                            case "Right":
                                autonomousSteps = autonomousStepsFile.ReadStepFileRelicRecovery("5291BlueRight.csv", "none", "none");
                                break;
                        }
                        break;
                    case "Test":
                        autonomousSteps = autonomousStepsFile.ReadStepFileRelicRecovery("11230Test.csv", "none", "none");
                        break;
                }
                break;

            case "11231":
                switch (allianceColor) {
                    case "Red":
                        switch (allianceStartPosition) {
                            case "Left":
                                autonomousSteps = autonomousStepsFile.ReadStepFileRelicRecovery("11231RedLeft.csv", "none", "none");
                                break;
                            case "Right":
                                autonomousSteps = autonomousStepsFile.ReadStepFileRelicRecovery("11231RedRight.csv", "none", "none");
                                break;
                        }
                        break;
                    case "Blue":
                        switch (allianceStartPosition) {
                            case "Left":
                                autonomousSteps = autonomousStepsFile.ReadStepFileRelicRecovery("11231BleLeft.csv", "none", "none");
                                break;
                            case "Right":
                                autonomousSteps = autonomousStepsFile.ReadStepFileRelicRecovery("11231BlueRight.csv", "none", "none");
                                break;
                        }
                        break;
                    case "Test":
                        autonomousSteps = autonomousStepsFile.ReadStepFileRelicRecovery("11231Test.csv", "none", "none");
                        break;
                }
                break;
        }

        //need to load initial step of a delay based on user input
        autonomousStepsFile.insertSteps(delay + 1, "DEL" + (delay * 1000), false, false, 0, 0, 0, 0, 0, 0, 0, 1);

        dashboard.displayPrintf(1, "initRobot STEPS LOADED");

        fileLogger.writeEvent(3, TAG, "Loading Autonomous Steps - Finished");
        fileLogger.writeEvent(3, TAG, "Configuring Adafruit IMU - Start");

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

        fileLogger.writeEvent(3, TAG, "Configuring Adafruit IMU - Finished");
        fileLogger.writeEvent(3, TAG, "Configuring Motors Base - Start");

        dashboard.displayPrintf(1, "initRobot BaseDrive Loading");

        robotDrive.init(fileLogger, hardwareMap, robotConfigSettings.robotConfigChoice.valueOf(robotConfig));
        robotDrive.init(hardwareMap, robotConfigSettings.robotConfigChoice.valueOf(robotConfig));
        robotDrive.setHardwareDriveResetEncoders();
        robotDrive.setHardwareDriveRunUsingEncoders();

        dashboard.displayPrintf(1, "initRobot BaseDrive Loaded");

        fileLogger.writeEvent(3, TAG, "Configuring Motors Base - Finish");
        fileLogger.writeEvent(3, TAG, "Configuring Motors Arms - Start");

        armDrive.init(fileLogger, hardwareMap, robotConfigSettings.robotConfigChoice.valueOf(robotConfig), "lifttop", "liftbot", null, null);
        armDrive.setHardwareDriveResetEncoders();
        armDrive.setHardwareDriveRunUsingEncoders();

        fileLogger.writeEvent(3, TAG, "Configuring Motors Lifts - Finish");
        fileLogger.writeEvent(3, TAG, "Configuring Range Sensors - Start");

        dashboard.displayPrintf(1, "initRobot Range Sensors Loading");

        RANGE1 = hardwareMap.i2cDevice.get("range1");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();
        RANGE2 = hardwareMap.i2cDevice.get("range2");
        RANGE2Reader = new I2cDeviceSynchImpl(RANGE2, RANGE2ADDRESS, false);
        RANGE2Reader.engage();

        dashboard.displayPrintf(1, "initRobot Range Sensors Loaded");

        fileLogger.writeEvent(3, TAG, "Configuring Range Sensors - Finish");
        fileLogger.writeEvent(3, TAG, "Resetting State Engine - Start");

        mintCurrentStateStep = stepState.STATE_INIT;
        mintCurrentStateTankTurn = stepState.STATE_COMPLETE;
        mintCurrentStateDrive = stepState.STATE_COMPLETE;
        mintCurrentStateDriveHeading = stepState.STATE_COMPLETE;
        mintCurrentStatePivotTurn = stepState.STATE_COMPLETE;
        mintCurrentStateRadiusTurn = stepState.STATE_COMPLETE;
        mintCurrentStepDelay = stepState.STATE_COMPLETE;
        mintCurrentStateVuforiaLocalise5291 = stepState.STATE_COMPLETE;
        mintCurStVuforiaMove5291 = stepState.STATE_COMPLETE;
        mintCurStVuforiaTurn5291 = stepState.STATE_COMPLETE;
        mintCurrentStateEyes5291 = stepState.STATE_COMPLETE;
        mintCurrentStateJewelColour5291 = stepState.STATE_COMPLETE;
        mintCurrentStateJewelArm5291 = stepState.STATE_COMPLETE;
        mintCurrentStateTopGripperMove5291 = stepState.STATE_COMPLETE;
        mintCurrentStateBotGripperMove5291 = stepState.STATE_COMPLETE;
        mintCurrentStateTopLiftMove5291 = stepState.STATE_COMPLETE;
        mintCurrentStateMainLiftMove5291 = stepState.STATE_COMPLETE;
        mintCurrentStateJewelArmClose5291 = stepState.STATE_COMPLETE;
        mintCurrentStateTankTurnGyroHeading = stepState.STATE_COMPLETE;
        mintCurrentStateMecanumStrafe = stepState.STATE_COMPLETE;
        mintCurrentStateGyroTurnEncoder5291 = stepState.STATE_COMPLETE;

        mint5291LEDStatus = LEDState.STATE_TEAM;
        mblnNextStepLastPos = false;

        fileLogger.writeEvent(3, TAG, "Resetting State Engine - Finish");
        fileLogger.writeEvent(3, TAG, "Configuring Vuforia - Start");

        dashboard.displayPrintf(1, "initRobot VUFORIA Loading");

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
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 1);

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
                .translation(mmPerInch * (float) targetPoint.x, mmPerInch * (float) targetPoint.y, mmPerInch * (float) targetPoint.z)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, (float) targetAngle.x, (float) targetAngle.y, (float) targetAngle.z));

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
        //activate vuforia
        RelicRecovery.activate();

        //set up variable for our capturedimage
        Image rgb = null;

        dashboard.displayPrintf(1, "initRobot VUFORIA Loaded");
        fileLogger.writeEvent(3, TAG, "Configuring Servos - Start");

        //config the servos
        servoGlyphGripTopLeft = hardwareMap.servo.get("griptopleft");
        servoGlyphGripBotLeft = hardwareMap.servo.get("gripbotleft");
        servoGlyphGripTopLeft.setDirection(Servo.Direction.REVERSE);
        servoGlyphGripBotLeft.setDirection(Servo.Direction.REVERSE);
        servoGlyphGripTopRight = hardwareMap.servo.get("griptopright");
        servoGlyphGripBotRight = hardwareMap.servo.get("gripbotright");
        servoJewelLeft = hardwareMap.servo.get("jewelleft");
        servoJewelRight = hardwareMap.servo.get("jewelright");

        //lock the jewel arms home
        sendServosHome(servoGlyphGripTopLeft, servoGlyphGripBotLeft, servoGlyphGripTopRight, servoGlyphGripBotRight, servoJewelLeft, servoJewelRight);
        fileLogger.writeEvent(3, TAG, "Configuring Servos - Finish");

        dashboard.displayPrintf(1, "initRobot Servos Loaded");

        // get a reference to our digitalTouch object.
        limitswitch1 = hardwareMap.get(DigitalChannel.class, "limittop");
        limitswitch2 = hardwareMap.get(DigitalChannel.class, "limitbot");

        // set the digital channel to input.
        limitswitch1.setMode(DigitalChannel.Mode.INPUT);
        limitswitch2.setMode(DigitalChannel.Mode.INPUT);

        fileLogger.writeEvent(1, TAG, "Set Limit Switches");

        dashboard.displayPrintf(1, "initRobot Limit Switches Configured");

        //vuMark will be the position to load the glyph
        //lets try to get colour during init and the vumark while waiting for start
        int vuMarkLoop = 0;

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        while (vuMarkLoop < 5) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark.toString().equals("UNKNOWN")) {
                vuMarkLoop++;
                Thread.sleep(100);
            } else
                break;
        }

        Mat tmp = new Mat();

//        try {
//            VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take(); //takes the frame at the head of the queue
//            long numImages = frame.getNumImages();
//
//            for (int i = 0; i < numImages; i++) {
//                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
//                    rgb = frame.getImage(i);
//                    break;
//                }
//            }
//
//            /*rgb is now the Image object that we’ve used in the video*/
//            Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
//            bm.copyPixelsFromBuffer(rgb.getPixels());
//
//            //put the image into a MAT for OpenCV
//            tmp = new Mat(rgb.getWidth(), rgb.getHeight(), CvType.CV_8UC4);
//            Utils.bitmapToMat(bm, tmp);
//            //close the frame, prevents memory leaks and crashing
//            frame.close();
//            mColour = JewelColour.JewelAnalysisOCV(fileLogger, tmp, 1);
//            fileLogger.writeEvent(3, TAG, "Init Colour Returned " + mColour + " Column " + vuMark.toString());
//        } catch (InterruptedException e) {
//            dashboard.displayPrintf(1, "VUFORIA --- ERROR ERROR ERROR");
//            dashboard.displayPrintf(2, "VUFORIA --- ERROR ERROR ERROR");
//            fileLogger.writeEvent(3, TAG, "Init Colour Returned " + mColour + " Column " + vuMark.toString());
//        }

//        dashboard.displayPrintf(2, "Jewel Colour-" + mColour + " Column-" + vuMark.toString());
        fileLogger.writeEvent(3, TAG, "Configuring Vuforia - Finished");
        fileLogger.writeEvent(3, TAG, "Configuring Robot Parameters - Start");
        dashboard.displayPrintf(1, "Init - Complete, Wait for Start");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        dashboard.clearDisplay();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //the main loop.  this is where the action happens
        while (opModeIsActive()) {
            if (!mblnDisableVisionProcessing) {
                //start capturing frames for analysis
                if (mblnReadyToCapture) {
                    //vuMark will be the position to load the glyph
                    vuMark = RelicRecoveryVuMark.from(relicTemplate);

                    tmp = new Mat();
                    try {
                        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take(); //takes the frame at the head of the queue
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
                        fileLogger.writeEvent(3, TAG, "Init Colour Returned " + mColour + " Column " + vuMark.toString());
                    }

                    mColour = JewelColour.JewelAnalysisOCV(fileLogger, tmp, mintCaptureLoop);
                    fileLogger.writeEvent(1, "OPENCV", "Returned " + mColour);
                    dashboard.displayPrintf(1, "Object -" + mColour);
                    mintCaptureLoop++;
                }

                //use vuforia to get locations informatio
                for (VuforiaTrackable trackable : allTrackables) {
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
                    fileLogger.writeEvent(1, TAG, "mintCurrentStateStep:- " + mintCurrentStateStep + " mintCurrentStateStep " + mintCurrentStateStep);
                    fileLogger.writeEvent(1, TAG, "About to check if step exists " + mintCurrentStep);
                    // get step from hashmap, send it to the initStep for decoding
                    if (autonomousSteps.containsKey(String.valueOf(mintCurrentStep))) {
                        fileLogger.writeEvent(1, TAG, "Step Exists TRUE " + mintCurrentStep + " about to get the values from the step");
                        initStep();
                    } else {
                        mintCurrentStateStep = stepState.STATE_FINISHED;
                    }
                    break;
                case STATE_START:

                    break;
                case STATE_RUNNING:

                    loadParallelSteps();
                    for (String stKey : mintActiveStepsCopy.keySet()) {
                        fileLogger.writeEvent(1, "STATE_RUNNING", "Looping through Parallel steps, found " + stKey);
                        mintStepNumber = mintActiveStepsCopy.get(stKey);
                        loadActiveStep(mintStepNumber);
                        fileLogger.writeEvent(1, "STATE_RUNNING", "About to run " + mstrRobotCommand.substring(0, 3));
                        processSteps(mstrRobotCommand.substring(0, 3));
                    }

                    if ((mintCurrentStepDelay == stepState.STATE_COMPLETE) &&
                            (mintCurStVuforiaTurn5291 == stepState.STATE_COMPLETE) &&
                            (mintCurrentStateVuforiaLocalise5291 == stepState.STATE_COMPLETE) &&
                            (mintCurStVuforiaMove5291 == stepState.STATE_COMPLETE) &&
                            (mintCurrentStateDrive == stepState.STATE_COMPLETE) &&
                            (mintCurrentStateDriveHeading == stepState.STATE_COMPLETE) &&
                            (mintCurrentStatePivotTurn == stepState.STATE_COMPLETE) &&
                            (mintCurrentStateTankTurn == stepState.STATE_COMPLETE) &&
                            (mintCurrentStateEyes5291 == stepState.STATE_COMPLETE) &&
                            (mintCurrentStateJewelColour5291 == stepState.STATE_COMPLETE) &&
                            (mintCurrentStateJewelArm5291 == stepState.STATE_COMPLETE) &&
                            (mintCurrentStateTopGripperMove5291 == stepState.STATE_COMPLETE) &&
                            (mintCurrentStateBotGripperMove5291 == stepState.STATE_COMPLETE) &&
                            (mintCurrentStateTopLiftMove5291 == stepState.STATE_COMPLETE) &&
                            (mintCurrentStateMainLiftMove5291 == stepState.STATE_COMPLETE) &&
                            (mintCurrentStateJewelArmClose5291 == stepState.STATE_COMPLETE) &&
                            (mintCurrentStateGyroTurnEncoder5291 == stepState.STATE_COMPLETE) &&
                            (mintCurrentStateTankTurnGyroHeading == stepState.STATE_COMPLETE) &&
                            (mintCurrentStateMecanumStrafe == stepState.STATE_COMPLETE) &&
                            (mintCurrentStateRadiusTurn == stepState.STATE_COMPLETE)) {
                        mintCurrentStateStep = stepState.STATE_COMPLETE;
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
                                mintCurrentStateStep = stepState.STATE_COMPLETE;
                        }
                    }
                    break;
                case STATE_PAUSE:
                    break;
                case STATE_COMPLETE:
                    fileLogger.writeEvent(1, TAG, "Step Complete - Current Step:- " + mintCurrentStep);
                    //  Transition to a new state and next step.
                    mintCurrentStep++;
                    mintCurrentStateStep = stepState.STATE_INIT;
                    break;
                case STATE_TIMEOUT:
                    robotDrive.setHardwareDrivePower(0);
                    //  Transition to a new state.
                    mintCurrentStateStep = stepState.STATE_FINISHED;
                    break;
                case STATE_ERROR:
                    dashboard.displayPrintf(2, "STATE", "ERROR WAITING TO FINISH " + mintCurrentStep);
                    break;
                case STATE_FINISHED:
                    robotDrive.setHardwareDrivePower(0);
                    //stop the logging
                    if (fileLogger != null) {
                        fileLogger.writeEvent(1, TAG, "Step FINISHED - FINISHED");
                        fileLogger.writeEvent(1, TAG, "Stopped");
                        Log.d(TAG, "FileLogger Stopped");
                        fileLogger.close();
                        fileLogger = null;
                    }
                    //deactivate vuforia
                    RelicRecovery.deactivate();
                    dashboard.displayPrintf(1, "STATE", "FINISHED " + mintCurrentStep);
                    break;
                case STATE_ASTAR_PRE_INIT:
                    mintCurrentStepAStar = 1;                                          //init the Step for AStar
                    //get start point
                    //get end point
                    int startX = (int) processingSteps.getmRobotParm1();
                    int startY = (int) processingSteps.getmRobotParm2();
                    int startZ = (int) processingSteps.getmRobotParm3();
                    int endX = (int) processingSteps.getmRobotParm4();
                    int endY = (int) processingSteps.getmRobotParm5();
                    int endDir = (int) processingSteps.getmRobotParm6();

                    //before using the path in the command lets check if we can localise
                    if (lastLocation != null) {
                        //lets get locations for AStar, direction is most important
                        //x and y position for Vuforia are in mm, AStar in Inches
                        //counter clockwise rotation (x,y) = (-x, y)
                        //origin is center of field
                        //Astar is top right so need to add in 6 feet to each value
                        startX = (int) (localisedRobotX / 25.4) + 72;
                        startY = (int) (localisedRobotY / 25.4) + 72;
                        //need to rotate the axis -90 degrees
                        startZ = (int) localisedRobotBearing;

                        if ((startZ > 357) && (startZ < 3))
                            startZ = 90;
                        else if ((startZ > 267) && (startZ < 273))
                            startZ = 0;
                        else if ((startZ > 177) && (startZ < 183))
                            startZ = 270;
                        else if ((startZ > 87) && (startZ < 93))
                            startZ = 180;

                        fileLogger.writeEvent(1, TAG, "AStar Init - Localised Values");
                        fileLogger.writeEvent(1, TAG, "AStar Init - localisedRobotX:        " + localisedRobotX);
                        fileLogger.writeEvent(1, TAG, "AStar Init - localisedRobotY:        " + localisedRobotY);
                        fileLogger.writeEvent(1, TAG, "AStar Init - localisedRobotBearing:  " + localisedRobotBearing);
                        fileLogger.writeEvent(1, TAG, "AStar Init - startX:                 " + startX);
                        fileLogger.writeEvent(1, TAG, "AStar Init - startY:                 " + startY);
                        fileLogger.writeEvent(1, TAG, "AStar Init - startZ:                 " + startZ);
                    }

                    //process path
                    pathValues = getPathValues.findPathAStar(startX, startY, startZ, endX, endY, endDir);  //for enhanced
                    fileLogger.writeEvent(1, TAG, "AStar Path - length:                 " + pathValues.length);

                    String[][] mapComplete = new String[A0Star.FIELDWIDTH][A0Star.FIELDWIDTH];

                    //write path to logfile to verify path
                    for (int y = 0; y < a0Star.fieldLength; y++) {
                        for (int x = 0; x < a0Star.fieldWidth; x++) {
                            switch (allianceColor) {
                                case "Red":
                                    if (a0Star.walkableRed[y][x]) {
                                        mapComplete[y][x] = "1";
                                        if ((x == startX) && (y == startY))
                                            mapComplete[y][x] = "S";
                                        else if ((x == endX) && (y == endY))
                                            mapComplete[y][x] = "E";
                                    } else {
                                        mapComplete[y][x] = "0";
                                    }
                                    break;

                                case "Blue":
                                    if (a0Star.walkableBlue[y][x]) {
                                        mapComplete[y][x] = "1";
                                        if ((x == startX) && (y == startY))
                                            mapComplete[y][x] = "S";
                                        else if ((x == endX) && (y == endY))
                                            mapComplete[y][x] = "E";
                                    } else {
                                        if ((x == startX) && (y == startY)) {
                                            mapComplete[y][x] = "1";
                                        } else {
                                            mapComplete[y][x] = "0";
                                        }
                                    }
                                    break;

                                default:
                                    if (a0Star.walkable[y][x]) {
                                        mapComplete[y][x] = "1";
                                        if ((x == startX) && (y == startY))
                                            mapComplete[y][x] = "S";
                                        else if ((x == endX) && (y == endY))
                                            mapComplete[y][x] = "E";
                                    }
                                    break;
                            }
                        }
                    }

                    //plot out path..
                    for (int i = 0; i < pathValues.length; i++) {
                        fileLogger.writeEvent(1, TAG, "Path " + pathValues[i].val1 + " " + pathValues[i].val2 + " " + pathValues[i].val3 + " Dir:= " + pathValues[i].val4);
                        if (((int) pathValues[i].val1 == 0) && ((int) pathValues[i].val3 == 0) && ((int) pathValues[i].val2 == 0) && ((int) pathValues[i].val4 == 0))
                            break;
                        mapComplete[(int) pathValues[i].val3][(int) pathValues[i].val2] = "P";
                        if ((pathValues[i].val2 == startX) && (pathValues[i].val3 == startY)) {
                            mapComplete[(int) pathValues[i].val3][(int) pathValues[i].val2] = "S";
                        }
                    }
                    mapComplete[endY][endX] = "E";
                    fieldOutput = "";

                    for (int y = 0; y < a0Star.fieldLength; y++) {
                        for (int x = 0; x < a0Star.fieldWidth; x++) {
                            fieldOutput = "" + fieldOutput + mapComplete[y][x];
                        }
                        if (debug >= 2) {
                            fileLogger.writeEvent(TAG, fieldOutput);
                        }
                        fieldOutput = "";
                    }

                    //load path in Hashmap
                    boolean dirChanged;
                    boolean processingAStarSteps = true;
                    int startSegment = 1;
                    int numberOfMoves;
                    int key = 0;
                    int lastDirection = 0;
                    int lasti = 0;
                    String strAngleChange = "RT00";

                    while (processingAStarSteps) {
                        numberOfMoves = 0;
                        for (int i = startSegment; i < pathValues.length; i++) {
                            numberOfMoves++;
                            fileLogger.writeEvent(2, TAG, "Path " + pathValues[i].val1 + " " + pathValues[i].val2 + " " + pathValues[i].val3 + " Dir:= " + pathValues[i].val4);
                            if (((int) pathValues[i].val1 == 0) && ((int) pathValues[i].val2 == 0) && ((int) pathValues[i].val3 == 0)) {
                                fileLogger.writeEvent(2, TAG, "End Detected");
                                //end of the sequence,
                                lastDirection = (int) pathValues[i - 1].val4;
                                processingAStarSteps = false;
                                lasti = i;
                            }
                            //need to check if the first step is in a different direction that the start
                            if (i == 1) {
                                if (startZ != pathValues[i].val4) {  //need to turn
                                    strAngleChange = getAngle(startZ, (int) pathValues[i].val4);
                                    fileLogger.writeEvent(2, TAG, "First Step Need to turn Robot " + strAngleChange + " Path " + pathValues[i].val1 + " " + pathValues[i].val2 + " " + pathValues[i].val3 + " Dir:= " + pathValues[i].val4);
                                    fileLogger.writeEvent(2, TAG, "Adding Command (" + key + ", 10, " + strAngleChange + ", 0, 0, 0, 0, 0, 0, 1, false) ");
                                    autonomousStepsAStar.put(String.valueOf(key), new LibraryStateSegAuto(key, 10, strAngleChange, false, false, 0, 0, 0, 0, 0, 0, 1));
                                    key++;
                                    dirChanged = true;
                                } else {
                                    dirChanged = false;    //no change in direction
                                }
                            } else {
                                //work out the sequence not the first step
                                if (pathValues[i - 1].val4 != pathValues[i].val4) {  //need to turn
                                    strAngleChange = getAngle((int) pathValues[i - 1].val4, (int) pathValues[i].val4);
                                    dirChanged = true;
                                } else {
                                    dirChanged = false;    //no change in direction
                                }
                            }
                            if ((dirChanged) || (!processingAStarSteps)) {
                                //found end of segment
                                int AStarPathAngle;
                                if (i == 1) {
                                    AStarPathAngle = startZ;
                                } else {
                                    AStarPathAngle = (int) pathValues[i - 1].val4;
                                }
                                switch (AStarPathAngle) {
                                    case 0:
                                    case 90:
                                    case 180:
                                    case 270:
                                        fileLogger.writeEvent(2, TAG, "Heading on a Straight line " + (numberOfMoves) + " Path");
                                        fileLogger.writeEvent(2, TAG, "Adding Command (" + key + ", 10, " + "FW" + (numberOfMoves) + ", false, false, 0, 0, 0, 0, 0, 0, 0.8, false) ");
                                        autonomousStepsAStar.put(String.valueOf(key), new LibraryStateSegAuto(key, 10, "FW" + numberOfMoves, false, false, 0, 0, 0, 0, 0, 0, 0.8));
                                        numberOfMoves = 0;
                                        key++;
                                        break;
                                    case 45:
                                    case 135:
                                    case 225:
                                    case 315:
                                        fileLogger.writeEvent(2, TAG, "Heading on a Straight line " + (int) ((numberOfMoves) * 1.4142) + " Path");
                                        fileLogger.writeEvent(2, TAG, "Adding Command (" + key + ", 10, " + "FW" + (int) ((numberOfMoves) * 1.4142) + ", false, false, 0, 0, 0, 0, 0, 0, .8, false) ");
                                        autonomousStepsAStar.put(String.valueOf(key), new LibraryStateSegAuto(key, 10, "FW" + (int) (numberOfMoves * 1.4142), false, false, 0, 0, 0, 0, 0, 0, 1));
                                        numberOfMoves = 0;
                                        key++;
                                        break;
                                }
                                fileLogger.writeEvent(2, TAG, "Need to turn Robot " + strAngleChange + " Path " + pathValues[i].val1 + " " + pathValues[i].val2 + " " + pathValues[i].val3 + " Dir:= " + pathValues[i].val4);
                                fileLogger.writeEvent(2, TAG, "Adding Command (" + key + ", 10, " + strAngleChange + ", 0, 0, 0, 0, 0, 0, 1, false) ");
                                autonomousStepsAStar.put(String.valueOf(key), new LibraryStateSegAuto(key, 10, strAngleChange, false, false, 0, 0, 0, 0, 0, 0, 0.4));
                                key++;
                            }
                            if (!processingAStarSteps)
                                break;

                        }
                        //need to work out the direction we are facing and the required direction
                        if ((lastDirection != endDir) && (!processingAStarSteps)) {
                            fileLogger.writeEvent(2, TAG, "Sraight Moves Robot End Of Sequence - Need to Trun Robot");
                            strAngleChange = getAngle((int) pathValues[lasti - 1].val4, endDir);
                            fileLogger.writeEvent(1, TAG, "Adding Command (" + key + ", 10, " + strAngleChange + ", 0, 0, 0, 0, 0, 0, 1, false) ");
                            autonomousStepsAStar.put(String.valueOf(key), new LibraryStateSegAuto(key, 10, strAngleChange, false, false, 0, 0, 0, 0, 0, 0, 0.4));
                            key++;
                        }
                    }
                    mintCurrentStateStep = stepState.STATE_ASTAR_INIT;

                    break;
                case STATE_ASTAR_INIT: {
                    fileLogger.writeEvent(1, TAG, "About to check if step exists " + mintCurrentStepAStar);
                    // get step from hashmap, send it to the initStep for decoding
                    if (autonomousStepsAStar.containsKey(String.valueOf(mintCurrentStepAStar))) {
                        fileLogger.writeEvent(1, TAG, "Step Exists TRUE " + mintCurrentStepAStar + " about to get the values from the step");
                        processingSteps = autonomousStepsAStar.get(String.valueOf(mintCurrentStepAStar));      //read the step from the hashmap
                        autonomousStepsAStar.remove(String.valueOf(mintCurrentStepAStar));                     //remove the step from the hashmap
                        fileLogger.writeEvent(1, TAG, "Got the values for step " + mintCurrentStepAStar + " about to decode and removed them");
                        //decode the step from hashmap
                        initAStarStep(processingSteps);
                    } else {
                        //if no steps left in hashmap then complete
                        mintCurrentStateStep = stepState.STATE_ASTAR_COMPLETE;
                    }
                }
                break;
                case STATE_ASTAR_RUNNING: {
                    //move robot according AStar hashmap
                    TankTurnStep();
                    PivotTurnStep();
                    RadiusTurnStep();
                    DriveStepHeading();
                    if ((mintCurrentStateDriveHeading == stepState.STATE_COMPLETE) &&
                            (mintCurrentStatePivotTurn == stepState.STATE_COMPLETE) &&
                            (mintCurrentStateTankTurn == stepState.STATE_COMPLETE) &&
                            (mintCurrentStateRadiusTurn == stepState.STATE_COMPLETE)) {
                        //increment ASTar Steps Counter
                        mintCurrentStepAStar++;
                        mintCurrentStateStep = stepState.STATE_ASTAR_INIT;
                    }

                }
                break;
                case STATE_ASTAR_ERROR: {
                    //do something on error
                }
                break;
                case STATE_ASTAR_COMPLETE: {
                    //empty hashmap ready for next AStar processing.
                    //clear AStar step counter ready for next AStar process
                    mintCurrentStepAStar = 0;
                    //when complete, keep processing normal step
                    fileLogger.writeEvent(1, TAG, "A* Path Completed:- " + mintCurrentStep);
                    //  Transition to a new state and next step.
                    mintCurrentStep++;
                    mintCurrentStateStep = stepState.STATE_INIT;
                }
                break;
            }

            //process LED status
            //ERROR - FLASH RED 3 TIMES
            switch (mint5291LEDStatus) {
                case STATE_TEAM:        //FLASH Alliance Colour
                    switch (allianceColor) {
                        case "Red":
                            LedState(LedOff, LedOn, LedOff, LedOff, LedOn, LedOff);
                            break;
                        case "Blue":
                            LedState(LedOff, LedOff, LedOn, LedOff, LedOff, LedOn);
                            break;
                        default:
                            LedState(LedOn, LedOn, LedOn, LedOn, LedOn, LedOn);
                            break;
                    }
                case STATE_ERROR:       //Flash RED 3 times Rapidly
                    if ((!mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOff + 250))) {
                        mdblLastOn = mStateTime.milliseconds();
                        mblnLEDON = true;
                        LedState(LedOff, LedOn, LedOff, LedOff, LedOn, LedOff);
                    } else if ((mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOn + 750))) {
                        mdblLastOff = mStateTime.milliseconds();
                        mblnLEDON = false;
                        LedState(LedOff, LedOff, LedOff, LedOff, LedOff, LedOff);
                    }
                    break;
                case STATE_SUCCESS:       //Flash GREEN 3 times Rapidly
                    if ((!mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOff + 250))) {
                        mdblLastOn = mStateTime.milliseconds();
                        mblnLEDON = true;
                        LedState(LedOn, LedOff, LedOff, LedOn, LedOff, LedOff);
                    } else if ((mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOn + 250))) {
                        mdblLastOff = mStateTime.milliseconds();
                        mblnLEDON = false;
                        LedState(LedOff, LedOff, LedOff, LedOff, LedOff, LedOff);
                        mintCounts++;
                    }
                    if (mintCounts >= 5) {
                        mintCounts = 0;
                        mint5291LEDStatus = LEDState.STATE_TEAM;
                    }
                    break;
                case STATE_OBJECT:       //

                    if ((!mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOff + 500))) {
                        mdblLastOn = mStateTime.milliseconds();
                        mblnLEDON = true;
                        if (mColour == Constants.ObjectColours.OBJECT_BLUE) {    //means red is to the right
                            LedState(LedOff, LedOff, LedOn, LedOff, LedOff, LedOff);
                        } else if (mColour == Constants.ObjectColours.OBJECT_RED) {
                            LedState(LedOff, LedOn, LedOff, LedOff, LedOff, LedOff);
                        }
                    } else if ((mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOn + 500))) {
                        mdblLastOff = mStateTime.milliseconds();
                        mblnLEDON = false;
                        if (mColour == Constants.ObjectColours.OBJECT_BLUE_RED) {    //means red is to the right
                            LedState(LedOff, LedOff, LedOff, LedOff, LedOn, LedOff);
                        } else if (mColour == Constants.ObjectColours.OBJECT_RED_BLUE) {
                            LedState(LedOff, LedOff, LedOff, LedOff, LedOff, LedOn);
                        } else if (mColour == Constants.ObjectColours.OBJECT_BLUE) {
                            LedState(LedOff, LedOff, LedOff, LedOn, LedOff, LedOff);
                        } else if (mColour == Constants.ObjectColours.OBJECT_BLUE) {
                            LedState(LedOff, LedOff, LedOff, LedOff, LedOn, LedOff);
                        }
                        mintCounts++;
                    }
                    if (mintCounts >= 10) {
                        mintCounts = 0;
                        mint5291LEDStatus = LEDState.STATE_TEAM;
                    }
                    break;
                case STATE_FINISHED:      //Solid Green
                    LedState(LedOn, LedOff, LedOff, LedOn, LedOff, LedOff);
                    break;

            }
        }
        //opmode not active anymore
    }

    private void loadActiveStep(int step) {

        LibraryStateSegAuto mStateSegAuto = autonomousSteps.get(String.valueOf(step));
        fileLogger.writeEvent(1, "loadActiveStep()", "Got the values for step " + step + " about to decode");
        mdblStepDistance = 0;
        mdblStepTimeout = mStateSegAuto.getmRobotTimeOut();
        mdblStepSpeed = mStateSegAuto.getmRobotSpeed();
        mstrRobotCommand = mStateSegAuto.getmRobotCommand();
        mdblRobotParm1 = mStateSegAuto.getmRobotParm1();
        mdblRobotParm2 = mStateSegAuto.getmRobotParm2();
        mdblRobotParm3 = mStateSegAuto.getmRobotParm3();
        mdblRobotParm4 = mStateSegAuto.getmRobotParm4();
        mdblRobotParm5 = mStateSegAuto.getmRobotParm5();
        mdblRobotParm6 = mStateSegAuto.getmRobotParm6();
        mblnParallel = mStateSegAuto.getmRobotParallel();
        mblnRobotLastPos = mStateSegAuto.getmRobotLastPos();
    }

    private void loadParallelSteps() {
        mintActiveStepsCopy.clear();
        for (String stKey : mintActiveSteps.keySet()) {
            fileLogger.writeEvent(2, "loadParallelSteps()", "Loading Active Parallel Step " + stKey);
            mintActiveStepsCopy.put(stKey, mintActiveSteps.get(stKey));
        }
    }

    private void deleteParallelStep() {
        for (String stKey : mintActiveStepsCopy.keySet()) {
            int tempStep = mintActiveStepsCopy.get(stKey);
            if (mintStepNumber == tempStep) {
                fileLogger.writeEvent(2, "deleteParallelStep()", "Removing Parallel Step " + tempStep);
                if (mintActiveSteps.containsKey(stKey))
                    mintActiveSteps.remove(stKey);
            }
        }
    }

    private void processSteps(String stepName) {

        switch (stepName) {
            case "DEL":
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
            case "JWA":  // Use the Camera to detect the colour of the Jewel
                DetectJewelColour();
                break;
            case "JWO":  // Open The Jewel Arms and create step to move robot
                OpenJewelServo();
                break;
            case "JWC":  // Close the Jewel Arms
                CloseJewelServo();
                break;
            case "MLF":  // Main Lift UP
                MainLiftMove();
                break;
            case "TLF":  // Main Lift UP
                TopLiftMove();
                break;
            case "TGR":  // Main Lift UP
                TopGripperMove();
                break;
            case "BGR":  // Main Lift UP
                BotGripperMove();
                break;
            case "MST":
                MecanumStrafe();
                break;
        }
    }

    //--------------------------------------------------------------------------
    //  Initialise the state.
    //--------------------------------------------------------------------------
    //private void initStep (LibraryStateSegAuto mStateSegAuto) {
    private void initStep() {
        fileLogger.writeEvent(3, "initstep()", "Starting to Decode Step " + mintCurrentStep);

        if (!(mintActiveSteps.containsValue(mintCurrentStep))) {
            mintActiveSteps.put(String.valueOf(mintCurrentStep), mintCurrentStep);
            fileLogger.writeEvent(3, "initstep()", "Put step into hashmap mintActiveSteps " + mintCurrentStep);
        }

        loadActiveStep(mintCurrentStep);
        mintCurrentStateStep = stepState.STATE_RUNNING;
        // Reset the state time, and then change to next state.
        mStateTime.reset();

        switch (mstrRobotCommand.substring(0, 3)) {
            case "DEL":
                mintCurrentStepDelay = stepState.STATE_INIT;
                break;
            case "GTH":
                mintCurrentStateTankTurnGyroHeading = stepState.STATE_INIT;
                break;
            case "MST":
                mintCurrentStateMecanumStrafe = stepState.STATE_INIT;
                break;
            case "LTE":
                mintCurrentStateTankTurn = stepState.STATE_INIT;
                break;
            case "RTE":
                mintCurrentStateTankTurn = stepState.STATE_INIT;
                break;
            case "LPE":
                mintCurrentStatePivotTurn = stepState.STATE_INIT;
                break;
            case "RPE":
                mintCurrentStatePivotTurn = stepState.STATE_INIT;
                break;
            case "LRE":  // Left turn with a Radius in Parm 1
                mintCurrentStateRadiusTurn = stepState.STATE_INIT;
                break;
            case "RRE":  // Right turn with a Radius in Parm 1
                mintCurrentStateRadiusTurn = stepState.STATE_INIT;
                break;
            case "FWE":  // Drive forward a distance in inches and power setting
                mintCurrentStateDriveHeading = stepState.STATE_INIT;
                break;
            case "ASE":  // Plot a course using A* algorithm, accuracy in Parm 1
                mintCurrentStateStep = stepState.STATE_ASTAR_PRE_INIT;
                break;
            case "VFL":  // Position the robot using vuforia parameters ready fro AStar  RObot should postion pointing to Red wall and Blue wall where targets are located
                mintCurrentStateVuforiaLocalise5291 = stepState.STATE_INIT;
                break;
            case "VME":  // Move the robot using localisation from the targets
                mintCurStVuforiaMove5291 = stepState.STATE_INIT;
                break;
            case "VTE":  // Turn the Robot using information from Vuforia and Pythag
                mintCurStVuforiaTurn5291 = stepState.STATE_INIT;
                break;
            case "GTE":  // Special Function, 5291 Move forward until line is found
                mintCurrentStateGyroTurnEncoder5291 = stepState.STATE_INIT;
                break;
            case "EYE":  // Special Function, 5291 Move forward until line is found
                mintCurrentStateEyes5291 = stepState.STATE_INIT;
                break;
            case "FNC":  //  Run a special Function with Parms

                break;
            case "JWA":
                mintCurrentStateJewelColour5291 = stepState.STATE_INIT;
                break;
            case "JWO":  //Open the Jewel Arms
                mintCurrentStateJewelArm5291 = stepState.STATE_INIT;
                break;
            case "JWC":  //close the jewel Arms
                mintCurrentStateJewelArmClose5291 = stepState.STATE_INIT;
                break;
            case "MLF":  // Main Lift UP
                mintCurrentStateMainLiftMove5291 = stepState.STATE_INIT;
                break;
            case "TLF":  // Main Lift UP
                mintCurrentStateTopLiftMove5291 = stepState.STATE_INIT;
                break;
            case "TGR":  //Top Gripper Position
                mintCurrentStateTopGripperMove5291 = stepState.STATE_INIT;
                break;
            case "BGR":  //Bottom Gripper Position
                mintCurrentStateBotGripperMove5291 = stepState.STATE_INIT;
                break;
        }

        fileLogger.writeEvent(2, "initStep()", "Current Step          :- " + mintCurrentStep);
        fileLogger.writeEvent(2, "initStep()", "mdblStepTimeout       :- " + mdblStepTimeout);
        fileLogger.writeEvent(2, "initStep()", "mdblStepSpeed         :- " + mdblStepSpeed);
        fileLogger.writeEvent(2, "initStep()", "mstrRobotCommand      :- " + mstrRobotCommand);
        fileLogger.writeEvent(2, "initStep()", "mblnParallel          :- " + mblnParallel);
        fileLogger.writeEvent(2, "initStep()", "mblnRobotLastPos      :- " + mblnRobotLastPos);
        fileLogger.writeEvent(2, "initStep()", "mdblRobotParm1        :- " + mdblRobotParm1);
        fileLogger.writeEvent(2, "initStep()", "mdblRobotParm2        :- " + mdblRobotParm2);
        fileLogger.writeEvent(2, "initStep()", "mdblRobotParm3        :- " + mdblRobotParm3);
        fileLogger.writeEvent(2, "initStep()", "mdblRobotParm4        :- " + mdblRobotParm4);
        fileLogger.writeEvent(2, "initStep()", "mdblRobotParm5        :- " + mdblRobotParm5);
        fileLogger.writeEvent(2, "initStep()", "mdblRobotParm6        :- " + mdblRobotParm6);
        fileLogger.writeEvent(2, "initStep()", "mdblStepDistance      :- " + mdblStepDistance);
        fileLogger.writeEvent(2, "initStep()", "mdblStepTurnL         :- " + mdblStepTurnL);
        fileLogger.writeEvent(2, "initStep()", "mdblStepTurnR         :- " + mdblStepTurnR);
    }

    private void initAStarStep(LibraryStateSegAuto mStateSegAuto) {
        mdblStepDistance = 0;
        fileLogger.writeEvent(3, "initAStarStep", "Starting to Decode AStar Step ");
        // Reset the state time, and then change to next state.
        mStateTime.reset();

        mdblStepTimeout = mStateSegAuto.getmRobotTimeOut();
        mdblStepSpeed = mStateSegAuto.getmRobotSpeed();
        mstrRobotCommand = mStateSegAuto.getmRobotCommand();
        mdblRobotParm1 = mStateSegAuto.getmRobotParm1();
        mdblRobotParm2 = mStateSegAuto.getmRobotParm2();
        mdblRobotParm3 = mStateSegAuto.getmRobotParm3();
        mdblRobotParm4 = mStateSegAuto.getmRobotParm4();
        mdblRobotParm5 = mStateSegAuto.getmRobotParm5();
        mdblRobotParm6 = mStateSegAuto.getmRobotParm6();

        mintCurrentStateStep = stepState.STATE_ASTAR_RUNNING;

        switch (mstrRobotCommand.substring(0, 3)) {
            case "DEL":
                mintCurrentStepDelay = stepState.STATE_INIT;
                break;
            case "LTE":
                mintCurrentStateTankTurn = stepState.STATE_INIT;
                break;
            case "RTE":
                mintCurrentStateTankTurn = stepState.STATE_INIT;
                break;
            case "LPE":
                mintCurrentStatePivotTurn = stepState.STATE_INIT;
                break;
            case "RPE":
                mintCurrentStatePivotTurn = stepState.STATE_INIT;
                break;
            case "LRE":  // Left turn with a Radius in Parm 1
                mintCurrentStateRadiusTurn = stepState.STATE_INIT;
                break;
            case "RRE":  // Right turn with a Radius in Parm 1
                mintCurrentStateRadiusTurn = stepState.STATE_INIT;
                break;
            case "FWE":  // Drive forward a distance in inches and power setting
                mintCurrentStateDriveHeading = stepState.STATE_INIT;
                break;
            case "ASE":  // Plot a course using A* algorithm, accuracy in Parm 1
                mintCurrentStateStep = stepState.STATE_ASTAR_PRE_INIT;
                break;
            case "FNC":  //  Run a special Function with Parms

                break;
        }
        fileLogger.writeEvent(2, "initAStarStep()", "Current Step          :- " + mintCurrentStep);
        fileLogger.writeEvent(2, "initAStarStep()", "mdblStepTimeout       :- " + mdblStepTimeout);
        fileLogger.writeEvent(2, "initAStarStep()", "mdblStepSpeed         :- " + mdblStepSpeed);
        fileLogger.writeEvent(2, "initAStarStep()", "mstrRobotCommand      :- " + mstrRobotCommand);
        fileLogger.writeEvent(2, "initAStarStep()", "mdblRobotParm1        :- " + mdblRobotParm1);
        fileLogger.writeEvent(2, "initAStarStep()", "mdblRobotParm2        :- " + mdblRobotParm2);
        fileLogger.writeEvent(2, "initAStarStep()", "mdblRobotParm3        :- " + mdblRobotParm3);
        fileLogger.writeEvent(2, "initAStarStep()", "mdblRobotParm4        :- " + mdblRobotParm4);
        fileLogger.writeEvent(2, "initAStarStep()", "mdblRobotParm5        :- " + mdblRobotParm5);
        fileLogger.writeEvent(2, "initAStarStep()", "mdblRobotParm6        :- " + mdblRobotParm6);
        fileLogger.writeEvent(2, "initAStarStep()", "mdblStepDistance      :- " + mdblStepDistance);
        fileLogger.writeEvent(2, "initAStarStep()", "mdblStepTurnL         :- " + mdblStepTurnL);
        fileLogger.writeEvent(2, "initAStarStep()", "mdblStepTurnR         :- " + mdblStepTurnR);
    }

    private void DriveStepHeading() {
        double dblStepSpeedTemp;
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
                mdblStepDistance = Double.parseDouble(mstrRobotCommand.substring(3));
                fileLogger.writeEvent(2, "runningDriveHeadingStep", "mdblStepDistance   :- " + mdblStepDistance);
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

                mintStepLeftTarget1 = mintStartPositionLeft1 + (int) (mdblStepDistance * COUNTS_PER_INCH);
                mintStepLeftTarget2 = mintStartPositionLeft2 + (int) (mdblStepDistance * COUNTS_PER_INCH);
                mintStepRightTarget1 = mintStartPositionRight1 + (int) (mdblStepDistance * COUNTS_PER_INCH);
                mintStepRightTarget2 = mintStartPositionRight2 + (int) (mdblStepDistance * COUNTS_PER_INCH);

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

                fileLogger.writeEvent(2, "runningDriveHeadingStep", "mStepLeftTarget1 :- " + mintStepLeftTarget1 + " mStepLeftTarget2 :- " + mintStepLeftTarget2);
                fileLogger.writeEvent(2, "runningDriveHeadingStep", "mStepRightTarget1:- " + mintStepRightTarget1 + " mStepRightTarget2:- " + mintStepRightTarget2);

                if (!(robotDrive.baseMotor1.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION))) {
                    // set motor controller to mode, Turn On RUN_TO_POSITION
                    robotDrive.setHardwareDriveRunToPosition();
                }

                mintCurrentStateDriveHeading = stepState.STATE_RUNNING;
                robotDrive.setHardwareDrivePower(Math.abs(mdblStepSpeed));

                break;
            case STATE_RUNNING:

                int gyroDelay;

                if (useAdafruitIMU) {
                    gyroDelay = 0;
                } else {
                    gyroDelay = 300;
                }

                dblStepSpeedTemp = mdblStepSpeed;

                intLeft1MotorEncoderPosition = robotDrive.baseMotor1.getCurrentPosition();
                intLeft2MotorEncoderPosition = robotDrive.baseMotor2.getCurrentPosition();
                intRight1MotorEncoderPosition = robotDrive.baseMotor3.getCurrentPosition();
                intRight2MotorEncoderPosition = robotDrive.baseMotor4.getCurrentPosition();

                // ramp up speed - need to write function to ramp up speed
                dblDistanceFromStartLeft1 = Math.abs(mintStartPositionLeft1 - intLeft1MotorEncoderPosition) / COUNTS_PER_INCH;
                dblDistanceFromStartLeft2 = Math.abs(mintStartPositionLeft2 - intLeft2MotorEncoderPosition) / COUNTS_PER_INCH;
                dblDistanceFromStartRight1 = Math.abs(mintStartPositionRight1 - intRight1MotorEncoderPosition) / COUNTS_PER_INCH;
                dblDistanceFromStartRight2 = Math.abs(mintStartPositionRight2 - intRight2MotorEncoderPosition) / COUNTS_PER_INCH;

                //if moving ramp up
                dblDistanceFromStart = (dblDistanceFromStartLeft1 + dblDistanceFromStartRight1 + dblDistanceFromStartLeft2 + dblDistanceFromStartRight2) / 4;

                //determine how close to target we are
                dblDistanceToEndLeft1 = (mintStepLeftTarget1 - intLeft1MotorEncoderPosition) / COUNTS_PER_INCH;
                dblDistanceToEndLeft2 = (mintStepLeftTarget2 - intLeft2MotorEncoderPosition) / COUNTS_PER_INCH;
                dblDistanceToEndRight1 = (mintStepRightTarget1 - intRight1MotorEncoderPosition) / COUNTS_PER_INCH;
                dblDistanceToEndRight2 = (mintStepRightTarget2 - intRight2MotorEncoderPosition) / COUNTS_PER_INCH;

                //if getting close ramp down speed
                dblDistanceToEnd = (dblDistanceToEndLeft1 + dblDistanceToEndRight1 + dblDistanceToEndLeft2 + dblDistanceToEndRight2) / 4;

                //parameter 1 or 4 is use gyro for direction,  setting either of these to 1 will get gyro correction
                // if parameter 1 is true
                // parameter 2 is the error
                // parameter 3 is the gain coefficient
                if ((mdblRobotParm1 == 1) || (mdblRobotParm4 == 1)) {
                    dblLeftSpeed = dblStepSpeedTemp;
                    dblRightSpeed = dblStepSpeedTemp;
                    //use Gyro to run heading
                    // adjust relative speed based on heading error.
                    if ((mStateTime.milliseconds() > gyroDelay)) {
                        dblError = getDriveError(mdblRobotParm2);
                        dblSteer = getDriveSteer(dblError, mdblRobotParm3);
                        fileLogger.writeEvent(3, "runningDriveHeadingStep", "dblError " + dblError);
                        fileLogger.writeEvent(3, "runningDriveHeadingStep", "dblSteer " + dblSteer);
                        fileLogger.writeEvent(3, "runningDriveHeadingStep", "Heading " + mdblRobotParm2);

                        // if driving in reverse, the motor correction also needs to be reversed
                        if (mdblStepDistance < 0)
                            dblSteer *= -1.0;

                        dblLeftSpeed = dblStepSpeedTemp - dblSteer;
                        dblRightSpeed = dblStepSpeedTemp + dblSteer;

                        // Normalize speeds if any one exceeds +/- 1.0;
                        dblMaxSpeed = Math.max(Math.abs(dblLeftSpeed), Math.abs(dblRightSpeed));
                        if (dblMaxSpeed > 1.0) {
                            dblLeftSpeed /= dblMaxSpeed;
                            dblRightSpeed /= dblMaxSpeed;
                        }
                    }
                } else {
                    dblLeftSpeed = dblStepSpeedTemp;
                    dblRightSpeed = dblStepSpeedTemp;
                }
                fileLogger.writeEvent(3, "runningDriveHeadingStep", "dblDistanceToEnd " + dblDistanceToEnd);

                if (mblnRobotLastPos) {
                    if (dblDistanceToEnd <= 3.0) {
                        fileLogger.writeEvent(3, "runningDriveHeadingStep", "mblnRobotLastPos Complete         ");
                        mblnNextStepLastPos = true;
                        mblnDisableVisionProcessing = false;  //enable vision processing
                        mintCurrentStateDriveHeading = stepState.STATE_COMPLETE;
                        deleteParallelStep();
                    }
                }

                //if within error margin stop
                //if (robotDrive.leftMotor1.isBusy() && robotDrive.rightMotor1.isBusy()) {
                //get motor busy state bitmap is right2, right1, left2, left1
                if (((robotDrive.getHardwareDriveIsBusy() & (robotConfigSettings.motors.leftMotor1.toInt() | robotConfigSettings.motors.rightMotor1.toInt())) == (robotConfigSettings.motors.leftMotor1.toInt() | robotConfigSettings.motors.rightMotor1.toInt()))) {
                    fileLogger.writeEvent(3, "runningDriveHeadingStep", "Encoder counts per inch = " + COUNTS_PER_INCH + " dblDistanceFromStart " + dblDistanceFromStart + " dblDistanceToEnd " + dblDistanceToEnd + " Power Level " + dblStepSpeedTemp + " Running to target  L1, L2, R1, R2  " + mintStepLeftTarget1 + ", " + mintStepLeftTarget2 + ", " + mintStepRightTarget1 + ",  " + mintStepRightTarget2 + ", " + " Running at position L1 " + intLeft1MotorEncoderPosition + " L2 " + intLeft2MotorEncoderPosition + " R1 " + intRight1MotorEncoderPosition + " R2 " + intRight2MotorEncoderPosition);
                    dashboard.displayPrintf(3, "Path1", "Running to %7d :%7d", mintStepLeftTarget1, mintStepRightTarget1);
                    dashboard.displayPrintf(4, "Path2", "Running at %7d :%7d", intLeft1MotorEncoderPosition, intRight1MotorEncoderPosition);
                    dashboard.displayPrintf(5, "Path3", "Running at %7d :%7d", intLeft2MotorEncoderPosition, intRight2MotorEncoderPosition);
                    // set power on motor controller to update speeds
                    robotDrive.setHardwareDriveLeftMotorPower(dblLeftSpeed);
                    robotDrive.setHardwareDriveRightMotorPower(dblRightSpeed);
                } else {
                    // Stop all motion;
                    robotDrive.setHardwareDrivePower(0);
                    fileLogger.writeEvent(2, "runningDriveHeadingStep", "Complete         ");
                    mblnDisableVisionProcessing = false;  //enable vision processing
                    mintCurrentStateDriveHeading = stepState.STATE_COMPLETE;
                    deleteParallelStep();
                }

                //check timeout value
                if (mStateTime.seconds() > mdblStepTimeout) {
                    fileLogger.writeEvent(1, "runningDriveHeadingStep", "Timeout:- " + mStateTime.seconds());
                    //  Transition to a new state.
                    mintCurrentStateDriveHeading = stepState.STATE_COMPLETE;
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

                fileLogger.writeEvent(2, "PivotTurnStep", "mdblStepTurnL      :- " + mdblStepTurnL);
                fileLogger.writeEvent(2, "PivotTurnStep", "mdblStepTurnR      :- " + mdblStepTurnR);

                // Turn On RUN_TO_POSITION
                if (mdblStepTurnR == 0) {
                    // Determine new target position
                    fileLogger.writeEvent(2, "PivotTurnStep", "Current LPosition:-" + robotDrive.baseMotor1.getCurrentPosition());

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
                    mintStepLeftTarget1 = mintStartPositionLeft1 + (int) (mdblStepTurnL * COUNTS_PER_DEGREE);
                    mintStepLeftTarget2 = mintStartPositionLeft2 + (int) (mdblStepTurnL * COUNTS_PER_DEGREE);
                    mintStepRightTarget1 = mintStartPositionRight1 + (int) (mdblStepTurnR * COUNTS_PER_DEGREE);
                    mintStepRightTarget2 = mintStartPositionRight2 + (int) (mdblStepTurnR * COUNTS_PER_DEGREE);

                    //store the encoder positions so next step can calculate destination
                    mintLastEncoderDestinationLeft1 = mintStepLeftTarget1;
                    mintLastEncoderDestinationLeft2 = mintStepLeftTarget2;
                    mintLastEncoderDestinationRight1 = mintStepRightTarget1;
                    mintLastEncoderDestinationRight2 = mintStepRightTarget2;
                    fileLogger.writeEvent(2, "PivotTurnStep", "mintStepLeftTarget1:-  " + mintStepLeftTarget1 + " mintStepLeftTarget2:-  " + mintStepLeftTarget2);

                    // pass target position to motor controller
                    robotDrive.baseMotor1.setTargetPosition(mintStepLeftTarget1);
                    robotDrive.baseMotor2.setTargetPosition(mintStepLeftTarget2);

                    // set left motor controller to mode
                    robotDrive.setHardwareDriveLeftRunToPosition();

                    // set power on motor controller to start moving
                    robotDrive.setHardwareDriveLeftMotorPower(Math.abs(mdblStepSpeed));
                } else {
                    // Determine new target position
                    fileLogger.writeEvent(2, "PivotTurnStep", "Current RPosition:-" + robotDrive.baseMotor3.getCurrentPosition());

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
                    mintStepLeftTarget1 = mintStartPositionLeft1 + (int) (mdblStepTurnL * COUNTS_PER_DEGREE);
                    mintStepLeftTarget2 = mintStartPositionLeft2 + (int) (mdblStepTurnL * COUNTS_PER_DEGREE);
                    mintStepRightTarget1 = mintStartPositionRight1 + (int) (mdblStepTurnR * COUNTS_PER_DEGREE);
                    mintStepRightTarget2 = mintStartPositionRight2 + (int) (mdblStepTurnR * COUNTS_PER_DEGREE);

                    //store the encoder positions so next step can calculate destination
                    mintLastEncoderDestinationLeft1 = mintStepLeftTarget1;
                    mintLastEncoderDestinationLeft2 = mintStepLeftTarget2;
                    mintLastEncoderDestinationRight1 = mintStepRightTarget1;
                    mintLastEncoderDestinationRight2 = mintStepRightTarget2;

                    fileLogger.writeEvent(3, "PivotTurnStep", "mintStepRightTarget1:- " + mintStepRightTarget1 + " mintStepRightTarget2:- " + mintStepRightTarget2);

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

                fileLogger.writeEvent(3, "PivotTurnStep", "gblStepLeftTarget :- " + mintStepLeftTarget1 + " mintStepLeftTarget2 :- " + mintStepLeftTarget2);
                fileLogger.writeEvent(3, "PivotTurnStep", "gblStepRightTarget:- " + mintStepRightTarget1 + " mintStepRightTarget2:- " + mintStepRightTarget2);
                mintCurrentStatePivotTurn = stepState.STATE_RUNNING;
            }
            break;
            case STATE_RUNNING: {

                intLeft1MotorEncoderPosition = robotDrive.baseMotor1.getCurrentPosition();
                intLeft2MotorEncoderPosition = robotDrive.baseMotor2.getCurrentPosition();
                intRight1MotorEncoderPosition = robotDrive.baseMotor3.getCurrentPosition();
                intRight2MotorEncoderPosition = robotDrive.baseMotor4.getCurrentPosition();

                //determine how close to target we are
                dblDistanceToEndLeft1 = (mintStepLeftTarget1 - intLeft1MotorEncoderPosition) / COUNTS_PER_INCH;
                dblDistanceToEndLeft2 = (mintStepLeftTarget2 - intLeft2MotorEncoderPosition) / COUNTS_PER_INCH;
                dblDistanceToEndRight1 = (mintStepRightTarget1 - intRight1MotorEncoderPosition) / COUNTS_PER_INCH;
                dblDistanceToEndRight2 = (mintStepRightTarget2 - intRight2MotorEncoderPosition) / COUNTS_PER_INCH;

                fileLogger.writeEvent(3, "PivotTurnStep", "Current LPosition1:-" + robotDrive.baseMotor1.getCurrentPosition() + " LTarget:- " + mintStepLeftTarget1 + " LPosition2:-" + robotDrive.baseMotor2.getCurrentPosition() + " LTarget2:- " + mintStepLeftTarget2);
                fileLogger.writeEvent(3, "PivotTurnStep", "Current RPosition1:-" + robotDrive.baseMotor3.getCurrentPosition() + " RTarget:- " + mintStepRightTarget1 + " RPosition2:-" + robotDrive.baseMotor4.getCurrentPosition() + " RTarget2:- " + mintStepRightTarget2);

                if (mdblStepTurnR == 0) {
                    fileLogger.writeEvent(3, "PivotTurnStep()", "Running         ");
                    dashboard.displayPrintf(3, "Target", "Running to %7d :%7d", mintStepLeftTarget1, mintStepRightTarget1);
                    dashboard.displayPrintf(4, "Actual_Left", "Running at %7d :%7d", intLeft1MotorEncoderPosition, intLeft2MotorEncoderPosition);
                    dashboard.displayPrintf(5, "ActualRight", "Running at %7d :%7d", intRight1MotorEncoderPosition, intRight2MotorEncoderPosition);

                    if (mblnRobotLastPos) {
                        if (((dblDistanceToEndLeft1 + dblDistanceToEndLeft2) / 2) < 2.0) {
                            mblnNextStepLastPos = true;
                            mblnDisableVisionProcessing = false;  //enable vision processing
                            mintCurrentStatePivotTurn = stepState.STATE_COMPLETE;
                            deleteParallelStep();
                        }
                    }
                    //if (!robotDrive.leftMotor1.isBusy()) {
                    //get motor busy state bitmap is right2, right1, left2, left1
                    if (((robotDrive.getHardwareDriveIsBusy() & robotConfigSettings.motors.leftMotor1.toInt()) == robotConfigSettings.motors.leftMotor1.toInt())) {
                        fileLogger.writeEvent(1, "PivotTurnStep()", "Complete         ");
                        mblnDisableVisionProcessing = false;  //enable vision processing
                        mintCurrentStatePivotTurn = stepState.STATE_COMPLETE;
                        deleteParallelStep();
                    }
                } else if (mdblStepTurnL == 0) {
                    fileLogger.writeEvent(3, "PivotTurnStep()", "Running         ");
                    dashboard.displayPrintf(3, "Target", "Running to %7d :%7d", mintStepLeftTarget1, mintStepRightTarget1);
                    dashboard.displayPrintf(4, "Actual_Left", "Running at %7d :%7d", intLeft1MotorEncoderPosition, intLeft2MotorEncoderPosition);
                    dashboard.displayPrintf(5, "ActualRight", "Running at %7d :%7d", intRight1MotorEncoderPosition, intRight2MotorEncoderPosition);

                    if (mblnRobotLastPos) {
                        if (((dblDistanceToEndRight1 + dblDistanceToEndRight2) / 2) < 2.2) {
                            mblnNextStepLastPos = true;
                            mblnDisableVisionProcessing = false;  //enable vision processing
                            mintCurrentStatePivotTurn = stepState.STATE_COMPLETE;
                            deleteParallelStep();
                        }
                    }
                    //if (!robotDrive.rightMotor1.isBusy()) {
                    //get motor busy state bitmap is right2, right1, left2, left1
                    if (((robotDrive.getHardwareDriveIsBusy() & robotConfigSettings.motors.rightMotor1.toInt()) == robotConfigSettings.motors.rightMotor1.toInt())) {
                        fileLogger.writeEvent(1, "PivotTurnStep()", "Complete         ");
                        mblnDisableVisionProcessing = false;  //enable vision processing
                        mintCurrentStatePivotTurn = stepState.STATE_COMPLETE;
                        deleteParallelStep();
                    }
                } else {
                    // Stop all motion by setting power to 0
                    robotDrive.setHardwareDrivePower(0);
                    fileLogger.writeEvent(1, "PivotTurnStep()", "Complete         ");
                    mblnDisableVisionProcessing = false;  //enable vision processing
                    mintCurrentStatePivotTurn = stepState.STATE_COMPLETE;
                    deleteParallelStep();
                }
            }
            //check timeout value
            if (mStateTime.seconds() > mdblStepTimeout) {
                fileLogger.writeEvent(1, "PivotTurnStep()", "Timeout:- " + mStateTime.seconds());
                //  Transition to a new state.
                mintCurrentStatePivotTurn = stepState.STATE_COMPLETE;
                deleteParallelStep();
            }
            break;
        }
    }

    private void TankTurnStep() {
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
                        mintStepLeftTarget1 = mintStartPositionLeft1 - (int) (0.5 * Double.parseDouble(mstrRobotCommand.substring(3)) * COUNTS_PER_DEGREE);
                        mintStepLeftTarget2 = mintStartPositionLeft2 - (int) (0.5 * Double.parseDouble(mstrRobotCommand.substring(3)) * COUNTS_PER_DEGREE);
                        mintStepRightTarget1 = mintStartPositionRight1 + (int) (0.5 * Double.parseDouble(mstrRobotCommand.substring(3)) * COUNTS_PER_DEGREE);
                        mintStepRightTarget2 = mintStartPositionRight2 + (int) (0.5 * Double.parseDouble(mstrRobotCommand.substring(3)) * COUNTS_PER_DEGREE);
                        break;
                    case "RTE":
                        mintStepLeftTarget1 = mintStartPositionLeft1 + (int) (0.5 * Double.parseDouble(mstrRobotCommand.substring(3)) * COUNTS_PER_DEGREE);
                        mintStepLeftTarget2 = mintStartPositionLeft2 + (int) (0.5 * Double.parseDouble(mstrRobotCommand.substring(3)) * COUNTS_PER_DEGREE);
                        mintStepRightTarget1 = mintStartPositionRight1 - (int) (0.5 * Double.parseDouble(mstrRobotCommand.substring(3)) * COUNTS_PER_DEGREE);
                        mintStepRightTarget2 = mintStartPositionRight2 - (int) (0.5 * Double.parseDouble(mstrRobotCommand.substring(3)) * COUNTS_PER_DEGREE);
                        break;
                }

                //store the encoder positions so next step can calculate destination
                mintLastEncoderDestinationLeft1 = mintStepLeftTarget1;
                mintLastEncoderDestinationLeft2 = mintStepLeftTarget2;
                mintLastEncoderDestinationRight1 = mintStepRightTarget1;
                mintLastEncoderDestinationRight2 = mintStepRightTarget2;

                fileLogger.writeEvent(3, "TankTurnStep()", "Current LPosition1:- " + robotDrive.baseMotor1.getCurrentPosition() + " mintStepLeftTarget1:-   " + mintStepLeftTarget1);
                fileLogger.writeEvent(3, "TankTurnStep()", "Current LPosition2:- " + robotDrive.baseMotor2.getCurrentPosition() + " mintStepLeftTarget2:-   " + mintStepLeftTarget2);
                fileLogger.writeEvent(3, "TankTurnStep()", "Current RPosition1:- " + robotDrive.baseMotor3.getCurrentPosition() + " mintStepRightTarget1:- " + mintStepRightTarget1);
                fileLogger.writeEvent(3, "TankTurnStep()", "Current RPosition2:- " + robotDrive.baseMotor4.getCurrentPosition() + " mintStepRightTarget2:- " + mintStepRightTarget2);

                // pass target position to motor controller
                robotDrive.baseMotor1.setTargetPosition(mintStepLeftTarget1);
                robotDrive.baseMotor2.setTargetPosition(mintStepLeftTarget2);
                robotDrive.baseMotor3.setTargetPosition(mintStepRightTarget1);
                robotDrive.baseMotor4.setTargetPosition(mintStepRightTarget2);
                // set motor controller to mode
                robotDrive.setHardwareDriveRunToPosition();
                // set power on motor controller to start moving
                robotDrive.setHardwareDrivePower(Math.abs(mdblStepSpeed));
                fileLogger.writeEvent(2, "TankTurnStep()", "mintStepLeftTarget1 :- " + mintStepLeftTarget1);
                fileLogger.writeEvent(2, "TankTurnStep()", "mintStepLeftTarget2 :- " + mintStepLeftTarget2);
                fileLogger.writeEvent(2, "TankTurnStep()", "mintStepRightTarget1:- " + mintStepRightTarget1);
                fileLogger.writeEvent(2, "TankTurnStep()", "mintStepRightTarget2:- " + mintStepRightTarget2);

                mintCurrentStateTankTurn = stepState.STATE_RUNNING;
            }
            break;
            case STATE_RUNNING: {

                intLeft1MotorEncoderPosition = robotDrive.baseMotor1.getCurrentPosition();
                intLeft2MotorEncoderPosition = robotDrive.baseMotor2.getCurrentPosition();
                intRight1MotorEncoderPosition = robotDrive.baseMotor3.getCurrentPosition();
                intRight2MotorEncoderPosition = robotDrive.baseMotor4.getCurrentPosition();

                //determine how close to target we are
                dblDistanceToEndLeft1 = (mintStepLeftTarget1 - intLeft1MotorEncoderPosition) / COUNTS_PER_INCH;
                dblDistanceToEndLeft2 = (mintStepLeftTarget2 - intLeft2MotorEncoderPosition) / COUNTS_PER_INCH;
                dblDistanceToEndRight1 = (mintStepRightTarget1 - intRight1MotorEncoderPosition) / COUNTS_PER_INCH;
                dblDistanceToEndRight2 = (mintStepRightTarget2 - intRight2MotorEncoderPosition) / COUNTS_PER_INCH;

                fileLogger.writeEvent(3, "TankTurnStep()", "Current LPosition1:- " + intLeft1MotorEncoderPosition + " LTarget1:- " + mintStepLeftTarget1);
                fileLogger.writeEvent(3, "TankTurnStep()", "Current LPosition2:- " + intLeft2MotorEncoderPosition + " LTarget2:- " + mintStepLeftTarget2);
                fileLogger.writeEvent(3, "TankTurnStep()", "Current RPosition1:- " + intRight1MotorEncoderPosition + " RTarget1:- " + mintStepRightTarget1);
                fileLogger.writeEvent(3, "TankTurnStep()", "Current RPosition2:- " + intRight2MotorEncoderPosition + " RTarget2:- " + mintStepRightTarget2);
                dashboard.displayPrintf(3, "Target", "Running to %7d :%7d", mintStepLeftTarget1, mintStepRightTarget1);
                dashboard.displayPrintf(4, "Actual_Left", "Running at %7d :%7d", intLeft1MotorEncoderPosition, intLeft2MotorEncoderPosition);
                dashboard.displayPrintf(5, "ActualRight", "Running at %7d :%7d", intRight1MotorEncoderPosition, intRight2MotorEncoderPosition);

                if (mblnRobotLastPos) {
                    if (((Math.abs(dblDistanceToEndRight1 + dblDistanceToEndRight2) / 2) < 2.2) && ((Math.abs(dblDistanceToEndLeft1 + dblDistanceToEndLeft2) / 2) < 2.2)) {
                        mblnNextStepLastPos = true;
                        mblnDisableVisionProcessing = false;  //enable vision processing
                        mintCurrentStateTankTurn = stepState.STATE_COMPLETE;
                        deleteParallelStep();
                    }
                }

                //if within error margin stop
                //get motor busy state bitmap is right2, right1, left2, left1
                if (!robotDrive.baseMotor1.isBusy() || (!robotDrive.baseMotor3.isBusy())) {
                    fileLogger.writeEvent(3, "TankTurnStep()", "Complete         ");
                    robotDrive.setHardwareDrivePower(0);
                    mblnDisableVisionProcessing = false;  //enable vision processing
                    mintCurrentStateTankTurn = stepState.STATE_COMPLETE;
                    deleteParallelStep();
                }
            }
            //check timeout value
            if (mStateTime.seconds() > mdblStepTimeout) {
                fileLogger.writeEvent(1, "TankTurnStep()", "Timeout:- " + mStateTime.seconds());
                //  Transition to a new state.
                mintCurrentStateTankTurn = stepState.STATE_COMPLETE;
                deleteParallelStep();
            }
            break;
        }
    }

    //this has not been programmed, do not use
    private void RadiusTurnStep() {
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
                fileLogger.writeEvent(3, "RadiusTurnStep()", "mdblRobotTurnAngle" + mdblRobotTurnAngle);

                //calculate the distance to travel based on the angle we are turning
                // length = radius x angle (in radians)
                rdblArcLengthRadiusTurnOuter = ((Double.parseDouble(mstrRobotCommand.substring(3)) / 180) * Math.PI) * mdblRobotParm1;
                dblArcLengthRadiusTurnInner = ((Double.parseDouble(mstrRobotCommand.substring(3)) / 180) * Math.PI) * (mdblRobotParm1 - (ROBOT_TRACK));
                //rdblArcLengthRadiusTurnOuter = ((Double.parseDouble(mstrRobotCommand.substring(3)) / 180) *  Math.PI) * (mdblRobotParm1 + (0.5 * ROBOT_TRACK));

                rdblSpeedOuter = mdblStepSpeed;

                if (rdblSpeedOuter >= 0.58) {
                    rdblSpeedOuter = 0.58;  //This is the maximum speed, anything above 0.6 is the same as a speed of 1 for drive to position
                }
                rdblSpeedInner = dblArcLengthRadiusTurnInner / rdblArcLengthRadiusTurnOuter * rdblSpeedOuter * 0.96;
                fileLogger.writeEvent(3, "RadiusTurnStep()", "dblArcLengthRadiusTurnInner " + dblArcLengthRadiusTurnInner);
                fileLogger.writeEvent(3, "RadiusTurnStep()", "rdblArcLengthRadiusTurnOuter " + rdblArcLengthRadiusTurnOuter);
                fileLogger.writeEvent(3, "RadiusTurnStep()", "rdblSpeedOuter " + rdblSpeedOuter);
                fileLogger.writeEvent(3, "RadiusTurnStep()", "rdblSpeedInner " + rdblSpeedInner);

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
                        mintStepLeftTarget1 = mintStartPositionLeft1 + (int) (dblArcLengthRadiusTurnInner * COUNTS_PER_INCH);
                        mintStepLeftTarget2 = mintStartPositionLeft2 + (int) (dblArcLengthRadiusTurnInner * COUNTS_PER_INCH);
                        mintStepRightTarget1 = mintStartPositionRight1 + (int) (rdblArcLengthRadiusTurnOuter * COUNTS_PER_INCH);
                        mintStepRightTarget2 = mintStartPositionRight2 + (int) (rdblArcLengthRadiusTurnOuter * COUNTS_PER_INCH);

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
                        mintStepLeftTarget1 = mintStartPositionLeft1 + (int) (rdblArcLengthRadiusTurnOuter * COUNTS_PER_INCH);
                        mintStepLeftTarget2 = mintStartPositionLeft2 + (int) (rdblArcLengthRadiusTurnOuter * COUNTS_PER_INCH);
                        mintStepRightTarget1 = mintStartPositionRight1 + (int) (dblArcLengthRadiusTurnInner * COUNTS_PER_INCH);
                        mintStepRightTarget2 = mintStartPositionRight2 + (int) (dblArcLengthRadiusTurnInner * COUNTS_PER_INCH);

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

                fileLogger.writeEvent(3, "RadiusTurnStep()", "Current LPosition1:- " + robotDrive.baseMotor1.getCurrentPosition() + " mintStepLeftTarget1:-   " + mintStepLeftTarget1);
                fileLogger.writeEvent(3, "RadiusTurnStep()", "Current LPosition2:- " + robotDrive.baseMotor2.getCurrentPosition() + " mintStepLeftTarget2:-   " + mintStepLeftTarget2);
                fileLogger.writeEvent(3, "RadiusTurnStep()", "Current RPosition1:- " + robotDrive.baseMotor3.getCurrentPosition() + " mintStepRightTarget1:- " + mintStepRightTarget1);
                fileLogger.writeEvent(3, "RadiusTurnStep()", "Current RPosition2:- " + robotDrive.baseMotor4.getCurrentPosition() + " mintStepRightTarget2:- " + mintStepRightTarget2);
                mintCurrentStateRadiusTurn = stepState.STATE_RUNNING;
            }
            break;
            case STATE_RUNNING: {
                intLeft1MotorEncoderPosition = robotDrive.baseMotor1.getCurrentPosition();
                intLeft2MotorEncoderPosition = robotDrive.baseMotor2.getCurrentPosition();
                intRight1MotorEncoderPosition = robotDrive.baseMotor3.getCurrentPosition();
                intRight2MotorEncoderPosition = robotDrive.baseMotor4.getCurrentPosition();

                //determine how close to target we are
                dblDistanceToEndLeft1 = (mintStepLeftTarget1 - intLeft1MotorEncoderPosition) / COUNTS_PER_INCH;
                dblDistanceToEndLeft2 = (mintStepLeftTarget2 - intLeft2MotorEncoderPosition) / COUNTS_PER_INCH;
                dblDistanceToEndRight1 = (mintStepRightTarget1 - intRight1MotorEncoderPosition) / COUNTS_PER_INCH;
                dblDistanceToEndRight2 = (mintStepRightTarget2 - intRight2MotorEncoderPosition) / COUNTS_PER_INCH;
                fileLogger.writeEvent(3, "RadiusTurnStep()", "Current LPosition1:- " + intLeft1MotorEncoderPosition + " LTarget1:- " + mintStepLeftTarget1);
                fileLogger.writeEvent(3, "RadiusTurnStep()", "Current LPosition2:- " + intLeft2MotorEncoderPosition + " LTarget2:- " + mintStepLeftTarget2);
                fileLogger.writeEvent(3, "RadiusTurnStep()", "Current RPosition1:- " + intRight1MotorEncoderPosition + " RTarget1:- " + mintStepRightTarget1);
                fileLogger.writeEvent(3, "RadiusTurnStep()", "Current RPosition2:- " + intRight2MotorEncoderPosition + " RTarget2:- " + mintStepRightTarget2);
                dashboard.displayPrintf(1, LABEL_WIDTH, "Target: ", "Running to %7d :%7d", mintStepLeftTarget1, mintStepRightTarget1);
                dashboard.displayPrintf(2, LABEL_WIDTH, "Actual_Left: ", "Running at %7d :%7d", intLeft1MotorEncoderPosition, intLeft2MotorEncoderPosition);
                dashboard.displayPrintf(3, LABEL_WIDTH, "ActualRight: ", "Running at %7d :%7d", intRight1MotorEncoderPosition, intRight2MotorEncoderPosition);

                if (mblnRobotLastPos) {
                    if ((((dblDistanceToEndRight1 + dblDistanceToEndRight2) / 2) < 2) && (((dblDistanceToEndLeft1 + dblDistanceToEndLeft2) / 2) < 2)) {
                        mblnNextStepLastPos = true;
                        mblnDisableVisionProcessing = false;  //enable vision processing
                        mintCurrentStateRadiusTurn = stepState.STATE_COMPLETE;
                        deleteParallelStep();
                    }
                }

                if (!robotDrive.baseMotor1.isBusy() || (!robotDrive.baseMotor3.isBusy())) {
                    robotDrive.setHardwareDrivePower(0);
                    fileLogger.writeEvent(1, "RadiusTurnStep()", "Complete         ");
                    mblnDisableVisionProcessing = false;  //enable vision processing
                    mintCurrentStateRadiusTurn = stepState.STATE_COMPLETE;
                    deleteParallelStep();
                }
            }
            //check timeout value
            if (mStateTime.seconds() > mdblStepTimeout) {
                fileLogger.writeEvent(1, "RadiusTurnStep()", "Timeout:- " + mStateTime.seconds());
                //  Transition to a new state.
                mintCurrentStateRadiusTurn = stepState.STATE_COMPLETE;
                deleteParallelStep();
            }
            break;
        }
    }

    private void MecanumStrafe() {
        int direction;
        switch (mintCurrentStateMecanumStrafe) {
            case STATE_INIT: {
                double adafruitIMUHeading;
                double currentHeading;

                adafruitIMUHeading = getAdafruitHeading();
                currentHeading = adafruitIMUHeading;
                //mdblRobotTurnAngle = Double.parseDouble(mstrRobotCommand.substring(3));
                //fileLogger.writeEvent(3, "MecanumStrafe", "USING HEADING FROM IMU=" + useAdafruitIMU);
                //fileLogger.writeEvent(3, "MecanumStrafe()", "mdblRobotTurnAngle " + mdblRobotTurnAngle + " currentHeading " + currentHeading);
                //mdblTurnAbsoluteGyro = Double.parseDouble(newAngleDirection((int) currentHeading, (int) mdblRobotTurnAngle).substring(3));
                robotDrive.setHardwareDriveRunWithoutEncoders();
                mintCurrentStateMecanumStrafe = stepState.STATE_RUNNING;
            }
            break;
            case STATE_RUNNING: {
                double adafruitIMUHeading;

                adafruitIMUHeading = getAdafruitHeading();
                mdblGyrozAccumulated = adafruitIMUHeading;
                mdblGyrozAccumulated = teamAngleAdjust(mdblGyrozAccumulated);//Set variables to MRgyro readings
                mdblTurnAbsoluteGyro = Double.parseDouble(newAngleDirectionGyro((int) mdblGyrozAccumulated, (int) mdblRobotTurnAngle).substring(3));
                String mstrDirection = (newAngleDirectionGyro((int) mdblGyrozAccumulated, (int) mdblRobotTurnAngle).substring(0, 3));
                fileLogger.writeEvent(3, "MecanumStrafe", "USING HEADING FROM IMU=" + useAdafruitIMU);
                fileLogger.writeEvent(3, "MecanumStrafe()", "Running, mdblGyrozAccumulated = " + mdblGyrozAccumulated);
                fileLogger.writeEvent(3, "MecanumStrafe()", "Running, mdblTurnAbsoluteGyro = " + mdblTurnAbsoluteGyro);
                fileLogger.writeEvent(3, "MecanumStrafe()", "Running, mstrDirection        = " + mstrDirection);
                fileLogger.writeEvent(3, "MecanumStrafe()", "Running, adafruitIMUHeading   = " + adafruitIMUHeading);

                readRangeSensors();
                if (mdblRobotParm1 > 0)
                    direction = 1;
                else
                    direction = -1;
                double dblLeftMotor1 = Range.clip(mdblStepSpeed*direction, -1, 1);
                double dblLeftMotor2 = Range.clip(-mdblStepSpeed*direction, -1, 1);
                double dblRightMotor1 = Range.clip(-mdblStepSpeed*direction, -1, 1);
                double dblRightMotor2 = Range.clip(mdblStepSpeed*direction, -1, 1);

                robotDrive.setHardwareDrivePower(dblLeftMotor1, dblLeftMotor2, dblRightMotor1, dblRightMotor2);

                fileLogger.writeEvent(3, "MecanumStrafe()", "Range Sensor2= " + mdblRangeSensor1);
                fileLogger.writeEvent(3, "MecanumStrafe()", "Range Sensor2= " + mdblRangeSensor2);


            } //end Case Running
            //check timeout value
            if (mStateTime.seconds() > mdblStepTimeout) {
                fileLogger.writeEvent(1, "TankTurnGyro()", "Timeout:- " + mStateTime.seconds());
                //  Transition to a new state.
                mintCurrentStateMecanumStrafe = stepState.STATE_COMPLETE;
                deleteParallelStep();
            }
            break;
        }
    }

    private void TankTurnGyroHeading() {
        switch (mintCurrentStateTankTurnGyroHeading) {
            case STATE_INIT: {
                double adafruitIMUHeading;

                adafruitIMUHeading = getAdafruitHeading();

                mdblPowerBoost = 0;
                mintStableCount = 0;
                mstrWiggleDir = "";
                mdblRobotTurnAngle = Double.parseDouble(mstrRobotCommand.substring(3));
                fileLogger.writeEvent(3, "TankTurnGyroHeadingEncoder", "USING HEADING FROM IMU=" + useAdafruitIMU);
                fileLogger.writeEvent(3, "TankTurnGyro()", "mdblRobotTurnAngle " + mdblRobotTurnAngle + " adafruitIMUHeading " + adafruitIMUHeading);
                mdblTurnAbsoluteGyro = Double.parseDouble(newAngleDirection((int) adafruitIMUHeading, (int) mdblRobotTurnAngle).substring(3));
                robotDrive.setHardwareDriveRunWithoutEncoders();

                mintCurrentStateTankTurnGyroHeading = stepState.STATE_RUNNING;
            }
            break;
            case STATE_RUNNING: {
                double adafruitIMUHeading;

                adafruitIMUHeading = getAdafruitHeading();

                mdblGyrozAccumulated = adafruitIMUHeading;
                mdblGyrozAccumulated = teamAngleAdjust(mdblGyrozAccumulated);//Set variables to MRgyro readings
                mdblTurnAbsoluteGyro = Double.parseDouble(newAngleDirectionGyro((int) mdblGyrozAccumulated, (int) mdblRobotTurnAngle).substring(3));
                String mstrDirection = (newAngleDirectionGyro((int) mdblGyrozAccumulated, (int) mdblRobotTurnAngle).substring(0, 3));
                fileLogger.writeEvent(3, "TankTurnGyroHeadingEncoder", "USING HEADING FROM IMU=" + useAdafruitIMU);
                fileLogger.writeEvent(3, "TankTurnGyro()", "Running, mdblGyrozAccumulated = " + mdblGyrozAccumulated);
                fileLogger.writeEvent(3, "TankTurnGyro()", "Running, mdblTurnAbsoluteGyro = " + mdblTurnAbsoluteGyro);
                fileLogger.writeEvent(3, "TankTurnGyro()", "Running, mstrDirection        = " + mstrDirection);
                fileLogger.writeEvent(3, "TankTurnGyro()", "Running, adafruitIMUHeading   = " + adafruitIMUHeading);

                if (Math.abs(mdblTurnAbsoluteGyro) > 21) {  //Continue while the robot direction is further than three degrees from the target
                    mintStableCount = 0;
                    fileLogger.writeEvent(3, "TankTurnGyro()", "High Speed.....");
                    if (mstrDirection.equals("LTE")) {
                        //want to turn left
                        if (debug >= 3) fileLogger.writeEvent("TankTurnGyro()", "Left Turn.....");
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
                        fileLogger.writeEvent(3, "TankTurnGyro()", "Right Turn.....");
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
                        mintCurrentStateTankTurnGyroHeading = stepState.STATE_COMPLETE;
                        deleteParallelStep();
                    }
                }
            } //end Case Running
            //check timeout value
            if (mStateTime.seconds() > mdblStepTimeout) {
                fileLogger.writeEvent(1, "TankTurnGyro()", "Timeout:- " + mStateTime.seconds());
                //  Transition to a new state.
                mintCurrentStateTankTurnGyroHeading = stepState.STATE_COMPLETE;
                deleteParallelStep();
            }
            break;
        }
    }

    private void MainLiftMove(){
        switch (mintCurrentStateMainLiftMove5291) {
            case STATE_INIT: {
                mintStartPositionMain = armDrive.baseMotor2.getCurrentPosition();
                mintTargetPositionMain = mintStartPositionMain + (int)(mdblRobotParm1 * LIFTMAIN_COUNTS_PER_INCH);
                mintCurrentStateMainLiftMove5291 = stepState.STATE_RUNNING;
                armDrive.baseMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armDrive.baseMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armDrive.baseMotor2.setTargetPosition(mintTargetPositionMain);
                armDrive.setHardwareDriveLeft2MotorPower(mdblStepSpeed);

                fileLogger.writeEvent(2,"MainLiftMove()", "Initialised");
            }
            break;
            case STATE_RUNNING: {
                fileLogger.writeEvent(2,"MainLiftMove()", "Running");
                if ((limitswitch2.getState() == false) && (mdblRobotParm1 < 0)){
                    armDrive.setHardwareDriveLeft2MotorPower(0);
                    armDrive.baseMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    armDrive.baseMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    fileLogger.writeEvent(1,"MainLiftMove()", "all way down:- " );
                    //  Transition to a new state.
                    mintCurrentStateMainLiftMove5291 = stepState.STATE_COMPLETE;
                    deleteParallelStep();
                    mintStartPositionMain = 0;
                    mintTargetPositionMain = 0;
                }

                if (!(armDrive.baseMotor2.isBusy())) {
                    armDrive.setHardwareDriveLeft2MotorPower(0);
                    //  Transition to a new state.
                    mintCurrentStateMainLiftMove5291 = stepState.STATE_COMPLETE;
                    deleteParallelStep();
                }

                //check timeout value
                if (mStateTime.seconds() > mdblStepTimeout) {
                    fileLogger.writeEvent(1,"MainLiftMove()", "Timeout:- " + mStateTime.seconds());
                    armDrive.setHardwareDriveLeft2MotorPower(0);
                    armDrive.baseMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    armDrive.baseMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    //  Transition to a new state.
                    mintCurrentStateMainLiftMove5291 = stepState.STATE_COMPLETE;
                    deleteParallelStep();
                }
            }
        }
    }

    private void TopLiftMove(){
        switch (mintCurrentStateTopLiftMove5291) {
            case STATE_INIT: {
                mintCurrentStateTopLiftMove5291 = stepState.STATE_RUNNING;
                fileLogger.writeEvent(2,"TopLiftMove()", "Initialised");
            }
            break;
            case STATE_RUNNING: {
                if (debug >= 2) fileLogger.writeEvent("TopLiftMove()", "Running");

                //  Transition to a new state.
                mintCurrentStateTopLiftMove5291 = stepState.STATE_COMPLETE;
                deleteParallelStep();

                //check timeout value
                if (mStateTime.seconds() > mdblStepTimeout) {
                    fileLogger.writeEvent(1,"TopLiftMove()", "Timeout:- " + mStateTime.seconds());
                    //  Transition to a new state.
                    mintCurrentStateTopLiftMove5291 = stepState.STATE_COMPLETE;
                    deleteParallelStep();
                }
            }
        }

    }

    private void TopGripperMove(){
        switch (mintCurrentStateTopGripperMove5291) {
            case STATE_INIT: {
                mintCurrentStateTopGripperMove5291 = stepState.STATE_RUNNING;
                fileLogger.writeEvent(2,"TopGripperMove()", "Initialised");
            }
            break;
            case STATE_RUNNING: {
                if (debug >= 2) fileLogger.writeEvent("TopGripperMove()", "Running");

                switch ((int)mdblRobotParm1) {
                    case 1:
                        moveTopServos(servoGlyphGripTopLeft, servoGlyphGripTopRight, SERVOLIFTLEFTTOP_HOME, SERVOLIFTRIGHTTOP_HOME);
                        break;
                    case 2:
                        moveTopServos(servoGlyphGripTopLeft, servoGlyphGripTopRight, SERVOLIFTLEFTTOP_GLYPH_START, SERVOLIFTRIGHTTOP_GLYPH_START);
                        break;
                    case 3:
                        moveTopServos(servoGlyphGripTopLeft, servoGlyphGripTopRight, SERVOLIFTLEFTTOP_GLYPH_RELEASE, SERVOLIFTRIGHTTOP_GLYPH_RELEASE);
                        break;
                    case 4:
                        moveTopServos(servoGlyphGripTopLeft, servoGlyphGripTopRight, SERVOLIFTLEFTTOP_GLYPH_GRAB, SERVOLIFTRIGHTTOP_GLYPH_GRAB);
                        break;
                }


                //  Transition to a new state.
                mintCurrentStateTopGripperMove5291 = stepState.STATE_COMPLETE;
                deleteParallelStep();

                //check timeout value
                if (mStateTime.seconds() > mdblStepTimeout) {
                    fileLogger.writeEvent(1,"TopGripperMove()", "Timeout:- " + mStateTime.seconds());
                    //  Transition to a new state.
                    mintCurrentStateTopGripperMove5291 = stepState.STATE_COMPLETE;
                    deleteParallelStep();
                }
            }
        }

    }


    private void BotGripperMove(){
        switch (mintCurrentStateBotGripperMove5291) {
            case STATE_INIT: {
                mintCurrentStateBotGripperMove5291 = stepState.STATE_RUNNING;
                fileLogger.writeEvent(2,"BotGripperMove()", "Initialised");
            }
            break;
            case STATE_RUNNING: {
                if (debug >= 2) fileLogger.writeEvent("BotGripperMove()", "Running");
                switch ((int)mdblRobotParm1) {
                    case 1:
                        moveTopServos(servoGlyphGripBotLeft, servoGlyphGripBotRight, SERVOLIFTLEFTBOT_HOME, SERVOLIFTRIGHTBOT_HOME);
                        break;
                    case 2:
                        moveTopServos(servoGlyphGripBotLeft, servoGlyphGripBotRight, SERVOLIFTLEFTBOT_GLYPH_START, SERVOLIFTRIGHTBOT_GLYPH_START);
                        break;
                    case 3:
                        moveTopServos(servoGlyphGripBotLeft, servoGlyphGripBotRight, SERVOLIFTLEFTBOT_GLYPH_RELEASE, SERVOLIFTRIGHTBOT_GLYPH_RELEASE);
                        break;
                    case 4:
                        moveTopServos(servoGlyphGripBotLeft, servoGlyphGripBotRight, SERVOLIFTLEFTBOT_GLYPH_GRAB, SERVOLIFTRIGHTBOT_GLYPH_GRAB);
                        break;
                }

                //  Transition to a new state.
                mintCurrentStateBotGripperMove5291 = stepState.STATE_COMPLETE;
                deleteParallelStep();

                //check timeout value
                if (mStateTime.seconds() > mdblStepTimeout) {
                    fileLogger.writeEvent(1,"BotGripperMove()", "Timeout:- " + mStateTime.seconds());
                    //  Transition to a new state.
                    mintCurrentStateBotGripperMove5291 = stepState.STATE_COMPLETE;
                    deleteParallelStep();
                }
            }
        }

    }

    private void OpenJewelServo() {
        switch (mintCurrentStateJewelArm5291) {
            case STATE_INIT: {
                mintCurrentStateJewelArm5291 = stepState.STATE_RUNNING;
                fileLogger.writeEvent(2, "MoveJewelServo()", "Initialised");
            }
            break;
            case STATE_RUNNING: {
                fileLogger.writeEvent(2, "MoveJewelServo()", "Running");

                moveServosPair(servoJewelLeft, servoJewelRight, SERVOJEWELLEFT_MIN_RANGE, SERVOJEWELRIGHT_MIN_RANGE);
                if (allianceColor.equals("Blue")) {
                    if (mColour == Constants.ObjectColours.OBJECT_BLUE) {
                        autonomousStepsFile.insertSteps(3, "FWE27", false, false, 0, 0, 0, 0, 0, 0, 0.6, mintCurrentStep + 1);
                        autonomousStepsFile.insertSteps(3, "JWC0", false, false, 1000, 0, 0, 0, 0, 0, 0, mintCurrentStep + 1);
                        autonomousStepsFile.insertSteps(3, "FWE-2", true, false, 0, 0, 0, 0, 0, 0, 0, mintCurrentStep + 1);
                    } else if (mColour == Constants.ObjectColours.OBJECT_RED) {
                        autonomousStepsFile.insertSteps(3, "JWC0", false, false, 1000, 0, 0, 0, 0, 0, 0, mintCurrentStep + 1);
                        autonomousStepsFile.insertSteps(3, "FWE24", true, false, 0, 0, 0, 0, 0, 0, 0.6, mintCurrentStep + 1);
                    } else {
                        //don't know colour so don't push any Jewel
                        autonomousStepsFile.insertSteps(3, "FWE24", false, false, 0, 0, 0, 0, 0, 0, 0.6, mintCurrentStep + 1);
                        autonomousStepsFile.insertSteps(3, "JWC0", false, false, 1000, 0, 0, 0, 0, 0, 0, mintCurrentStep + 1);
                    }
                } else if (allianceColor.equals("Red")) {
                    if (mColour == Constants.ObjectColours.OBJECT_BLUE) {
                        autonomousStepsFile.insertSteps(3, "JWC0", false, false, 1000, 0, 0, 0, 0, 0, 0, mintCurrentStep + 1);
                        autonomousStepsFile.insertSteps(3, "FWE24", true, false, 0, 0, 0, 0, 0, 0, 0.6, mintCurrentStep + 1);
                    } else if (mColour == Constants.ObjectColours.OBJECT_RED) {
                        //need to move backwards
                        autonomousStepsFile.insertSteps(3, "FWE27", false, false, 0, 0, 0, 0, 0, 0, 0.6, mintCurrentStep + 1);
                        autonomousStepsFile.insertSteps(3, "JWC0", false, false, 1000, 0, 0, 0, 0, 0, 0, mintCurrentStep + 1);
                        autonomousStepsFile.insertSteps(3, "FWE-2", true, false, 0, 0, 0, 0, 0, 0, 0.6, mintCurrentStep + 1);
                    } else {
                        //don't know colour so don't push any Jewel
                        autonomousStepsFile.insertSteps(3, "JWC0", false, false, 1000, 0, 0, 0, 0, 0, 0, mintCurrentStep + 1);
                        autonomousStepsFile.insertSteps(3, "FWE24", true, false, 0, 0, 0, 0, 0, 0, 0.5, mintCurrentStep + 1);
                    }
                }

                fileLogger.writeEvent(1, "MoveJewelServo()", "Complete:- " + mColour);
                mblnDisableVisionProcessing = true;  //disable vision processing
                mblnReadyToCapture = false; //stop OpenCV from doing its thing
                //  Transition to a new state.
                mintCurrentStateJewelArm5291 = stepState.STATE_COMPLETE;
                deleteParallelStep();

                //check timeout value
                if (mStateTime.seconds() > mdblStepTimeout) {
                    fileLogger.writeEvent(1, "MoveJewelServo()", "Timeout:- " + mStateTime.seconds());
                    //  Transition to a new state.
                    mintCurrentStateJewelArm5291 = stepState.STATE_COMPLETE;
                    deleteParallelStep();
                }
            }
        }
    }

    private void CloseJewelServo()
    {
        switch (mintCurrentStateJewelArmClose5291) {
            case STATE_INIT: {
                mintCurrentStateJewelArmClose5291 = stepState.STATE_RUNNING;
                fileLogger.writeEvent(2,"CloseJewelServo()", "Initialised");
            }
            break;
            case STATE_RUNNING: {
                if (debug >= 2) fileLogger.writeEvent("CloseJewelServo()", "Running");

                if (mStateTime.milliseconds() > mdblRobotParm1) {
                    moveServosPair(servoJewelLeft, servoJewelRight, SERVOJEWELLEFT_HOME, SERVOJEWELRIGHT_HOME);
                    fileLogger.writeEvent(2,"CloseJewelServo()", "Returning Servos");
                    //  Transition to a new state.
                    mintCurrentStateJewelArmClose5291 = stepState.STATE_COMPLETE;
                    deleteParallelStep();
                }

                //check timeout value
                if (mStateTime.seconds() > mdblStepTimeout) {
                    fileLogger.writeEvent(1,"CloseJewelServo()", "Timeout:- " + mStateTime.seconds());
                    //  Transition to a new state.
                    mintCurrentStateJewelArmClose5291 = stepState.STATE_COMPLETE;
                    deleteParallelStep();
                }
            }
        }
    }

    private void DetectJewelColour()
    {
        boolean blnColourOK = false;

        switch (mintCurrentStateJewelColour5291) {
            case STATE_INIT: {
                fileLogger.writeEvent(2,"DetectJewelColour()", "Initialising");
                if (!((mColour == Constants.ObjectColours.OBJECT_BLUE) || (mColour == Constants.ObjectColours.OBJECT_RED))) {
                    //ensure vision processing is enable
                    mblnDisableVisionProcessing     = false;    //enable vision processing
                    mblnReadyToCapture              = true;     //let OpenCV start doing its thing
                    mintNumberColourTries           = 0;
                    mintCaptureLoop                 = 0;
                    fileLogger.writeEvent(2,"DetectJewelColour()", "Don't have colour, not bypassing");
                    mintCurrentStateJewelColour5291 = stepState.STATE_RUNNING;
                    fileLogger.writeEvent(2,"DetectJewelColour()", "Initialised");
                } else {
                    fileLogger.writeEvent(2,"DetectJewelColour()", "have colour, bypassing");

                    //  Transition to a new state.
                    mintCurrentStateJewelColour5291 = stepState.STATE_COMPLETE;
                    deleteParallelStep();
                }
            }
            break;
            case STATE_RUNNING: {
                if (!((mColour == Constants.ObjectColours.OBJECT_BLUE) || (mColour == Constants.ObjectColours.OBJECT_RED))) {
                    mblnDisableVisionProcessing     = false;    //enable vision processing
                    mblnReadyToCapture              = true;     //let OpenCV start doing its thing
                }
                fileLogger.writeEvent(2,"DetectJewelColour()", "Running");
                mintNumberColourTries++;

                if (mColour == Constants.ObjectColours.OBJECT_BLUE) {    //means blue is to the right
                    blnColourOK = true;
                } else if (mColour == Constants.ObjectColours.OBJECT_RED) { //means blue is to the right
                    blnColourOK = true;
                }

                fileLogger.writeEvent(2,"DetectJewelColour()", "Returned mColour " + mColour);
                mint5291LEDStatus = LEDState.STATE_OBJECT;

                if ((blnColourOK) && (mintNumberColourTries < 2 ))
                {
                    fileLogger.writeEvent(1,"DetectJewelColour()", "Complete:- " + mColour);
                    mblnDisableVisionProcessing     = true;  //disable vision processing
                    mblnReadyToCapture              = false; //stop OpenCV from doing its thing
                    //  Transition to a new state.
                    mintCurrentStateJewelColour5291 = stepState.STATE_COMPLETE;
                    deleteParallelStep();
                }

                if (mintNumberColourTries >= 2) {
                    mblnReadyToCapture              = false;
                    mblnDisableVisionProcessing     = true;  //disable vision processing
                    mintCurrentStateJewelColour5291 = stepState.STATE_COMPLETE;
                    fileLogger.writeEvent(2,"DetectJewelColour()", "FAILED too many attempts");
                    mblnReadyToCapture = false; //stop OpenCV from doing its thing
                    deleteParallelStep();
                }
            }
            //check timeout value
            if (mStateTime.seconds() > mdblStepTimeout)
            {
                fileLogger.writeEvent(1,"DetectJewelColour()", "Timeout:- " + mStateTime.seconds());
                mblnDisableVisionProcessing         = true;  //disable vision processing
                mblnReadyToCapture                  = false; //stop OpenCV from doing its thing
                //  Transition to a new state.
                mintCurrentStateJewelColour5291     = stepState.STATE_COMPLETE;
                deleteParallelStep();
            }
        }
    }

    private void TankTurnGyroHeadingEncoder()
    {
        switch (mintCurrentStateGyroTurnEncoder5291){
            case STATE_INIT:
            {
                double adafruitIMUHeading;
                adafruitIMUHeading = getAdafruitHeading();
                mdblPowerBoost = 0;
                mintStableCount = 0;
                mstrWiggleDir = "";
                mdblRobotTurnAngle = Double.parseDouble(mstrRobotCommand.substring(3));
                fileLogger.writeEvent(3,"TankTurnGyroHeadingEncoder", "USE ADAFRUIT IMU = " + useAdafruitIMU + ",mdblRobotTurnAngle " + mdblRobotTurnAngle + " adafruitIMUHeading " + adafruitIMUHeading);
                mdblTurnAbsoluteGyro = Double.parseDouble(newAngleDirection((int)adafruitIMUHeading, (int) mdblRobotTurnAngle).substring(3));
                mintCurrentStateGyroTurnEncoder5291 = stepState.STATE_RUNNING;
            }
            break;
            case STATE_RUNNING: {

                double adafruitIMUHeading;
                adafruitIMUHeading = getAdafruitHeading();
                mdblGyrozAccumulated = teamAngleAdjust(mdblGyrozAccumulated); //Set variables to MRgyro readings
                mdblTurnAbsoluteGyro = Double.parseDouble(newAngleDirectionGyro ((int)mdblGyrozAccumulated, (int)mdblRobotTurnAngle).substring(3));
                String mstrDirection = (newAngleDirectionGyro ((int)mdblGyrozAccumulated, (int)mdblRobotTurnAngle).substring(0, 3));
                fileLogger.writeEvent(3,"TankTurnGyroHeadingEncoder", "USING HEADING FROM IMU=" + useAdafruitIMU);
                fileLogger.writeEvent(3,"TankTurnGyroHeadingEncoder", "Running, mdblGyrozAccumulated = " + mdblGyrozAccumulated);
                fileLogger.writeEvent(3,"TankTurnGyroHeadingEncoder", "Running, mdblTurnAbsoluteGyro = " + mdblTurnAbsoluteGyro);
                fileLogger.writeEvent(3,"TankTurnGyroHeadingEncoder", "Running, mstrDirection        = " + mstrDirection);
                fileLogger.writeEvent(3,"TankTurnGyroHeadingEncoder", "Running, adafruitIMUHeading   = " + adafruitIMUHeading);
                autonomousStepsFile.insertSteps(3, newAngleDirectionGyro ((int)mdblGyrozAccumulated, (int)mdblRobotTurnAngle), false, false, 0, 0, 0, 0, 0, 0, mdblStepSpeed, mintCurrentStep + 1);
                mintCurrentStateGyroTurnEncoder5291 = stepState.STATE_COMPLETE;
                deleteParallelStep();
            }
            //check timeout value
            if (mStateTime.seconds() > mdblStepTimeout) {
                fileLogger.writeEvent(1,"TankTurnGyroHeadingEnc", "Timeout:- " + mStateTime.seconds());
                //  Transition to a new state.
                mintCurrentStateGyroTurnEncoder5291 = stepState.STATE_COMPLETE;
                deleteParallelStep();
            }
            break;
        }
    }

    private void VuforiaLocalise ()
    {
        switch (mintCurrentStateVuforiaLocalise5291) {
            case STATE_INIT: {
                //ensure vision processing is enabled
                mblnDisableVisionProcessing = false;  //enable vision processing
                mintCurrentStateVuforiaLocalise5291 = stepState.STATE_RUNNING;
                fileLogger.writeEvent(3,"mintCurrentStateVuforiaLocalise5291", "Initialised");
            }
            break;
            case STATE_RUNNING:
            {
                String strCorrectionAngle;
                fileLogger.writeEvent(3, "mintCurrentStateVuforiaLocalise5291", "Running" );
                fileLogger.writeEvent(3, "mintCurrentStateVuforiaLocalise5291", "localiseRobotPos " + localiseRobotPos );
                if (!localiseRobotPos) {
                    //need to rerun this step as we cannot get localisation and need to adjust robot to see if we can see a target
                    autonomousStepsFile.insertSteps(3, "VFL", false, false, 0,    0,    0,    0,    0,    0,  0.5, mintCurrentStep + 1);
                    fileLogger.writeEvent(3,"mintCurStVuforiaLoc5291", "Not Localised, inserting a new step" );
                    //need a delay, as Vuforia is slow to update
                    autonomousStepsFile.insertSteps(2, "DEL500", false, false, 0, 0, 0, 0, 0, 0, 0, mintCurrentStep + 1);

                    //need to adjust robot so we can see target, lets turn robot 180 degrees, if we are facing RED drivers we will end up facing BLUE targets,
                    //if we are facing blue drives we will end up facing RED targets.
                    //if we can't localise we need to abort autonomous so lets try a few things to see if we can localise,
                    // first we will try turning around,
                    // second we will move forward 2 feet
                    // third - abort
                    //Parameter 1 - stop turning once localisation is achieved
                    autonomousStepsFile.insertSteps(3, "RTE135", false, true, 1,    0,    0,    0,    0,    0,  0.5, mintCurrentStep + 1);
                    mintCurrentStateVuforiaLocalise5291 = stepState.STATE_COMPLETE;
                    deleteParallelStep();
                    break;
                }

                int intLocalisedRobotBearing = (int)localisedRobotBearing;
                fileLogger.writeEvent(3,"mintCurrentStateVuforiaLocalise5291", "Localised, determining angles.... intLocalisedRobotBearing= " + intLocalisedRobotBearing + " Alliancecolour= " + allianceColor);
                //vuforia angles or 0 towards the BLUE drivers, AStar 0 is to the BLUE beacons
                if (allianceColor.equals("Red")) {
                    //double check localisation
                    if ((intLocalisedRobotBearing > 3) && (intLocalisedRobotBearing < 177)) {
                        strCorrectionAngle = "LTE" + (180 - intLocalisedRobotBearing);
                    } else if ((intLocalisedRobotBearing > 183) && (intLocalisedRobotBearing < 357)) {
                        strCorrectionAngle = "RTE" + (180 - (360 - intLocalisedRobotBearing));
                    } else {
                        mintCurrentStateVuforiaLocalise5291 = stepState.STATE_COMPLETE;
                        deleteParallelStep();
                        break;
                    }
                    //double check localisation
                    fileLogger.writeEvent(3,"mintCurrentStateVuforiaLocalise5291", "Inserting Steps VFL 0 0 0 mintCurrentStep " + mintCurrentStep);
                    autonomousStepsFile.insertSteps(3, "VFL", false, false, 0, 0, 0, 0, 0, 0, 0.5, mintCurrentStep + 1);
                    //need a delay, as Vuforia is slow to update
                    autonomousStepsFile.insertSteps(2, "DEL500", false, false, 0, 0, 0, 0, 0, 0, 0, mintCurrentStep + 1);
                    //load the angle to adjust
                    autonomousStepsFile.insertSteps(3, strCorrectionAngle, false, false, 0, 0, 0, 0, 0, 0, 0.3, mintCurrentStep + 1);
                    mintCurrentStateVuforiaLocalise5291 = stepState.STATE_COMPLETE;
                    deleteParallelStep();
                    break;
                }

                if (allianceColor.equals("Blue")) {
                    if ((intLocalisedRobotBearing > 273) && (intLocalisedRobotBearing < 360)) {
                        strCorrectionAngle = "LTE" + (intLocalisedRobotBearing - 270);
                    } else if ((intLocalisedRobotBearing > 0) && (intLocalisedRobotBearing < 91)) {
                        strCorrectionAngle = "LTE" + (90 + intLocalisedRobotBearing);
                    } else if ((intLocalisedRobotBearing > 90) && (intLocalisedRobotBearing < 267)) {
                        strCorrectionAngle = "RTE" + (270 - intLocalisedRobotBearing);
                    } else {
                        mintCurrentStateVuforiaLocalise5291 = stepState.STATE_COMPLETE;
                        deleteParallelStep();
                        break;
                    }
                    autonomousStepsFile.insertSteps(3, "VFL", false, false, 0, 0, 0, 0, 0, 0, 0.5, mintCurrentStep + 1);
                    //need a delay, as Vuforia is slow to update
                    autonomousStepsFile.insertSteps(2, "DEL500", false, false, 0, 0, 0, 0, 0, 0, 0, mintCurrentStep + 1);
                    //load the angle to adjust
                    autonomousStepsFile.insertSteps(3, strCorrectionAngle, false, false, 0, 0, 0, 0, 0, 0, 0.3, mintCurrentStep + 1);
                    mintCurrentStateVuforiaLocalise5291 = stepState.STATE_COMPLETE;
                    deleteParallelStep();
                    break;
                }
            }
            //check timeout value
            if (mStateTime.seconds() > mdblStepTimeout)
            {
                fileLogger.writeEvent(1,"mintCurrentStateVuforiaLocalise5291", "Timeout:- " + mStateTime.seconds());
                //  Transition to a new state.
                mintCurrentStateVuforiaLocalise5291 = stepState.STATE_COMPLETE;
                deleteParallelStep();
            }
            break;
        }
    }

    private void VuforiaMove ()
    {
        switch (mintCurStVuforiaMove5291) {
            case STATE_INIT: {
                //ensure vision processing is enable
                mblnDisableVisionProcessing = false;  //enable vision processing

                mintCurStVuforiaMove5291 = stepState.STATE_RUNNING;
                fileLogger.writeEvent(2,"VuforiaMove()", "Initialised");
            }
            break;
            case STATE_RUNNING:
            {
                String strCorrectionAngle;
                fileLogger.writeEvent(2,"VuforiaMove()", "Running" );
                fileLogger.writeEvent(2,"VuforiaMove()", "localiseRobotPos " + localiseRobotPos );

                if (!localiseRobotPos)
                {
                    //need to do something gere to try and get localise
                    mintCurStVuforiaMove5291 = stepState.STATE_COMPLETE;
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

                fileLogger.writeEvent(2,"VuforiaMove()", "Temp Values requiredMoveAngletemp1 " + requiredMoveAngletemp1 + " requiredMoveAngletemp2 " + requiredMoveAngletemp2 + " requiredMoveAngletemp3 " + requiredMoveAngletemp3);
                fileLogger.writeEvent(2,"VuforiaMove()", "Temp Values currentX " + currentX + " currentY " + currentY);
                fileLogger.writeEvent(2,"VuforiaMove()", "Localised, determining angles....Alliancecolour= " + allianceColor + " intLocalisedRobotBearing= " + intLocalisedRobotBearing + " CurrentX= " + currentX + " CurrentY= " + currentY);
                fileLogger.writeEvent(2,"VuforiaMove()", "Localised, determining angles....requiredMoveX " + requiredMoveX + " requiredMoveY " + requiredMoveY);
                fileLogger.writeEvent(2,"VuforiaMove()", "Localised, determining angles....requiredMoveDistance " + requiredMoveDistance + " requiredMoveAngle " + requiredMoveAngle);

                if ((((int) mdblRobotParm5) > currentY) && ((int) mdblRobotParm4 > currentX)) {
                    requiredMoveAngle = 90 - requiredMoveAngle;
                } else if ((((int) mdblRobotParm5) > currentY) && ((int) mdblRobotParm4 < currentX)) {
                    requiredMoveAngle =  90 + requiredMoveAngle;
                } else if ((((int) mdblRobotParm5) < currentY) && ((int) mdblRobotParm4 > currentX)) {
                    requiredMoveAngle = 270 + requiredMoveAngle;
                } else if ((((int) mdblRobotParm5) < currentY) && ((int) mdblRobotParm4 < currentX)) {
                    requiredMoveAngle = 270 - requiredMoveAngle;
                }

                strCorrectionAngle = newAngleDirection (intLocalisedRobotBearing, requiredMoveAngle);
                autonomousStepsFile.insertSteps(3, "FWE"+requiredMoveDistance, false, false, 0, 0, 0, 0, 0, 0, 0.6, mintCurrentStep + 1);
                autonomousStepsFile.insertSteps(3, strCorrectionAngle, false, false, 0, 0, 0, 0, 0, 0, 0.4, mintCurrentStep + 1);
                mintCurStVuforiaMove5291 = stepState.STATE_COMPLETE;
                deleteParallelStep();
            }
            //check timeout value
            if (mStateTime.seconds() > mdblStepTimeout)
            {
                fileLogger.writeEvent(1,"VuforiaMove()", "Timeout:- "  + mStateTime.seconds());
                //  Transition to a new state.
                mintCurStVuforiaMove5291 = stepState.STATE_COMPLETE;
                deleteParallelStep();
            }
            break;
        }
    }

    private void VuforiaTurn ()
    {
        String strCorrectionAngle;
        switch (mintCurStVuforiaTurn5291) {
            case STATE_INIT: {
                //ensure vision processing is enabled
                mblnDisableVisionProcessing     = false;  //enable vision processing
                mintCurStVuforiaTurn5291        = stepState.STATE_RUNNING;
                fileLogger.writeEvent(2,"VuforiaTurn()", "Initialised");
            }
            break;
            case STATE_RUNNING:
            {
                fileLogger.writeEvent(2,"VuforiaTurn()", "Running" );
                fileLogger.writeEvent(2,"VuforiaTurn()", "localiseRobotPos " + localiseRobotPos );

                if (!localiseRobotPos)
                {
                    //need to do something here to try and get localised
                    mintCurStVuforiaTurn5291    = stepState.STATE_COMPLETE;
                    deleteParallelStep();
                    break;
                }

                int intLocalisedRobotBearing    = (int)localisedRobotBearing;
                double requiredMoveAngle        = mdblRobotParm1;
                strCorrectionAngle = newAngleDirection (intLocalisedRobotBearing, (int)requiredMoveAngle);
                fileLogger.writeEvent(2,"VuforiaTurn()", "Localised, determining angles....Alliancecolour= " + allianceColor + " intLocalisedRobotBearing= " + intLocalisedRobotBearing  + " requiredMoveAngle " + requiredMoveAngle);
                autonomousStepsFile.insertSteps(3, strCorrectionAngle, false, false, 0, 0, 0, 0, 0, 0, 0.4, mintCurrentStep + 1);
                mintCurStVuforiaTurn5291 = stepState.STATE_COMPLETE;
                deleteParallelStep();
            }
            //check timeout value
            if (mStateTime.seconds() > mdblStepTimeout)
            {
                fileLogger.writeEvent(1,"VuforiaTurn()", "Timeout:- " + mStateTime.seconds());
                //  Transition to a new state.
                mintCurStVuforiaTurn5291 = stepState.STATE_COMPLETE;
                deleteParallelStep();
            }
            break;
        }
    }

    private void DelayStep ()
    {
        switch (mintCurrentStepDelay) {
            case STATE_INIT: {
                mintStepDelay = Integer.parseInt(mstrRobotCommand.substring(3));
                mintCurrentStepDelay = stepState.STATE_RUNNING;
                if (debug >= 2) {
                    fileLogger.writeEvent(3,"DelayStep()", "Init Delay Time    " + mintStepDelay);
                }
            }
            break;
            case STATE_RUNNING:
            {
                if (mStateTime.milliseconds() >= mintStepDelay)
                {
                    fileLogger.writeEvent(1,"DelayStep()", "Complete         ");
                    mintCurrentStepDelay = stepState.STATE_COMPLETE;
                    deleteParallelStep();
                }
            }
            //check timeout value
            if (mStateTime.seconds() > mdblStepTimeout)
            {
                fileLogger.writeEvent(1,"DelayStep()", "Timeout:- " + mStateTime.seconds());
                //  Transition to a new state.
                mintCurrentStepDelay = stepState.STATE_COMPLETE;
                deleteParallelStep();
            }
            break;
        }
    }

    private void LedState (boolean g1, boolean r1, boolean b1, boolean g2, boolean r2, boolean b2) {
        green1LedChannel.setState(g1);
        red1LedChannel.setState(r1);
        blue1LedChannel.setState(b1);
        green2LedChannel.setState(g2);
        red2LedChannel.setState(r2);
        blue2LedChannel.setState(b2);        
    }

    private String getAngle(int angle1, int angle2)
    {
        fileLogger.writeEvent(2,TAG, "Getangle - Current Angle1:= " + angle1 + " Desired Angle2:= " + angle2);
        switch (angle1)
        {
            case 0:
                switch (angle2)
                {
                    case 45:
                        return "RTE45";
                    case 90:
                        return "RTE90";
                    case 135:
                        return "RTE135";
                    case 180:
                        return "RTE180";
                    case 225:
                        return "LTE135";
                    case 270:
                        return "LTE90";
                    case 315:
                        return "LTE45";
                }
                break;
            case 45:
                switch (angle2)
                {
                    case 0:
                        return "LTE45";
                    case 90:
                        return "RTE45";
                    case 135:
                        return "RTE90";
                    case 180:
                        return "RTE135";
                    case 225:
                        return "RTE180";
                    case 270:
                        return "LTE135";
                    case 315:
                        return "LTE90";
                }
                break;
            case 90:
                switch (angle2)
                {
                    case 0:
                        return "LTE90";
                    case 45:
                        return "LTE45";
                    case 135:
                        return "RTE45";
                    case 180:
                        return "RTE90";
                    case 225:
                        return "RTE135";
                    case 270:
                        return "RTE180";
                    case 315:
                        return "LTE135";
                }
                break;
            case 135:
                switch (angle2)
                {
                    case 0:
                        return "LTE135";
                    case 45:
                        return "LTE90";
                    case 90:
                        return "LTE45";
                    case 180:
                        return "RTE45";
                    case 225:
                        return "RTE90";
                    case 270:
                        return "RTE135";
                    case 315:
                        return "RTE180";
                }
                break;
            case 180:
                switch (angle2)
                {
                    case 0:
                        return "LTE180";
                    case 45:
                        return "LTE135";
                    case 90:
                        return "LTE90";
                    case 135:
                        return "LTE45";
                    case 225:
                        return "RTE45";
                    case 270:
                        return "RTE90";
                    case 315:
                        return "RTE135";
                }
                break;
            case 225:
                switch (angle2)
                {
                    case 0:
                        return "RTE135";
                    case 45:
                        return "LTE180";
                    case 90:
                        return "LTE135";
                    case 135:
                        return "LTE90";
                    case 180:
                        return "LTE45";
                    case 270:
                        return "RTE45";
                    case 315:
                        return "RTE90";
                }
                break;
            case 270:
                switch (angle2)
                {
                    case 0:
                        return "RTE90";
                    case 45:
                        return "RTE135";
                    case 90:
                        return "LTE180";
                    case 135:
                        return "LTE135";
                    case 180:
                        return "LTE90";
                    case 225:
                        return "LTE45";
                    case 315:
                        return "RTE45";
                }
                break;
            case 315:
                switch (angle2)
                {
                    case 0:
                        return "RTE45";
                    case 45:
                        return "RTE90";
                    case 90:
                        return "RTE135";
                    case 135:
                        return "LTE180";
                    case 180:
                        return "LTE135";
                    case 225:
                        return "LTE90";
                    case 270:
                        return "LTE45";
                }
                break;
        }
        return "ERROR";
    }

    private String newAngleDirection (int currentDirection, int newDirection)
    {
        if (currentDirection < newDirection)
            return "LTE" + (newDirection - currentDirection);
        else
            return "RTE" + (currentDirection - newDirection);
    }

    private double teamAngleAdjust ( double angle ) {

        fileLogger.writeEvent(2,"teamAngleAdjust", "teamAngleAdjust - angle " + angle + " allianceColor " + allianceColor);

        if (allianceColor.equals("Red")) {
            //angle = angle + 90;  if starting against the wall
            //angle = angle + 225; if starting at 45 to the wall facing the beacon
            angle = angle + 225;
            if (angle > 360) {
                angle = angle - 360;
            }
            fileLogger.writeEvent(2,"teamAngleAdjust", "In RED Angle " + angle);

        } else
        if (allianceColor.equals("Blue")) {
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
        //adds 100ms to scan time, try use this as little as possible
        range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
        mdblRangeSensor1 = getDistance(range1Cache[0] & 0xFF, range1Cache[1] & 0xFF, DistanceUnit.CM);
        mdblRangeSensor2 = getDistance(range2Cache[0] & 0xFF, range2Cache[1] & 0xFF, DistanceUnit.CM);
        fileLogger.writeEvent(2,"readRangeSensors()", "mdblRangeSensor1 " + mdblRangeSensor1 + ",mdblRangeSensor2 " + mdblRangeSensor2);
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    private double getDriveError(double targetAngle) {

        double robotError;
        double robotErrorIMU;
        double robotErrorGyro;
        double MRgyroHeading;
        double adafruitIMUHeading;

        adafruitIMUHeading = getAdafruitHeading();

        fileLogger.writeEvent(2,"getDriveError()", "targetAngle " + targetAngle);
        fileLogger.writeEvent(2,"getDriveError()", "Adafruit IMU Reading " + adafruitIMUHeading);
        // calculate error in -179 to +180 range  (
        robotErrorIMU = targetAngle - teamAngleAdjust(adafruitIMUHeading);
        robotError = robotErrorIMU;
        fileLogger.writeEvent(2,"getDriveError()", "USING HEADING FROM IMU=" + useAdafruitIMU);
        fileLogger.writeEvent(2,"getDriveError()", "robotErrorIMU " + robotError + ", getAdafruitHeading() " + adafruitIMUHeading + " teamAngleAdjust(adafruitIMUHeading) "  + teamAngleAdjust(adafruitIMUHeading));

        if (robotError > 180)
            robotError -= 360;
        if (robotError <= -180)
            robotError += 360;

        fileLogger.writeEvent(2,"getDriveError()", "robotError2 " + robotError);

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
}