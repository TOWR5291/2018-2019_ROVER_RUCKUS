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
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.usb.exception.RobotUsbUnspecifiedException;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.util.HashMap;

import club.towr5291.R;
import club.towr5291.functions.Constants;
import club.towr5291.functions.FileLogger;
import club.towr5291.functions.ReadStepFileRoverRuckus;
import club.towr5291.functions.RoverRuckusOCV;
import club.towr5291.functions.TOWR5291TextToSpeech;
import club.towr5291.libraries.ImageCaptureOCV;
import club.towr5291.libraries.LibraryMotorType;
import club.towr5291.libraries.LibraryStateSegAutoRoverRuckus;
import club.towr5291.libraries.LibraryTensorFlowRoverRuckus;
import club.towr5291.libraries.LibraryVuforiaRoverRuckus;
import club.towr5291.libraries.TOWRDashBoard;
import club.towr5291.libraries.TOWR5291LEDControl;
import club.towr5291.libraries.robotConfig;
import club.towr5291.libraries.robotConfigSettings;
import club.towr5291.functions.TOWR5291Utils;
import club.towr5291.opmodes.OpModeMasterLinear;
import club.towr5291.robotconfig.HardwareArmMotorsRoverRuckus;
import club.towr5291.robotconfig.HardwareDriveMotors;
import club.towr5291.robotconfig.HardwareSensorsRoverRuckus;


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

Written by Ian Haden/Wyatt Ashley October 2018
2018-10-27 - Ian Haden  - Converted to ROVER RUCKUS

*/
@Autonomous(name="5291 Autonomous Drive Rover Ruckus", group="5291")
//@Disabled
public class AutoDriveTeam5291RoverRuckus extends OpModeMasterLinear {

    private OpMode onStop = this;
    private OpModeManagerImpl opModeManager;
    private String TeleOpMode = "Base Drive 2019";

    final int LABEL_WIDTH = 200;

    //The autonomous menu settings from the sharepreferences
    private SharedPreferences sharedPreferences;
    private robotConfig ourRobotConfig;

    private ElapsedTime runtime = new ElapsedTime();

    //set up the variables for file logger and what level of debug we will log info at
    public FileLogger fileLogger;
    private int debug = 3;

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

    //Camera Webcam
    private WebcamName robotWebcam;

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
    private Constants.stepState mintCurStVuforiaMove5291;                       // Current State of Vuforia Move
    private Constants.stepState mintCurStVuforiaTurn5291;                       // Current State of Vuforia Turn
    private Constants.stepState mintCurrentStateGyroTurnEncoder5291;            // Current State of the Turn function that take the Gyro as an initial heading
    private Constants.stepState mintCurrentStateEyes5291;                       // Current State of the Eyelids
    private Constants.stepState mintCurrentStateTankTurnGyroHeading;            // Current State of Tank Turn using Gyro
    private Constants.stepState mintCurrentStateMecanumStrafe;                  // Current State of mecanum strafe
    private Constants.stepState mintCurrentStepDelay;                           // Current State of Delay (robot doing nothing)
    private Constants.stepState mintCurrentStateMoveLift;                       // Current State of the Move lift
    private Constants.stepState mintCurrentStateInTake;                       // Current State of the Move lift
    private Constants.stepState mintCurrentStateTiltMotor;                      // Current State of the Tilt Motor
    private Constants.stepState mintCurrentStateFindGold;                       // Current State of Finding Gold
    private Constants.stepState mintCurrentStateTeamMarker;                     // Current State of releaseing the team marker
    private Constants.stepState mintCurrentStateWyattsGyroDrive;                     //Wyatt Gyro Function

    private int mintFindGoldLoop = 0;                                           //there can only be 3 positions so count how many times we try
    private boolean mboolFoundGold = false;

    private double mdblTeamMarkerDrop = .8;
    private double mdblTeamMarkerHome = 0;

    private HashMap<String, Integer> mintActiveSteps = new HashMap<>();
    private HashMap<String, Integer> mintActiveStepsCopy = new HashMap<>();

    //motors
    private HardwareDriveMotors robotDrive          = new HardwareDriveMotors();   // Use 5291's hardware
    private HardwareArmMotorsRoverRuckus robotArms  = new HardwareArmMotorsRoverRuckus();   // Use 5291's hardware
    private HardwareSensorsRoverRuckus sensor       = new HardwareSensorsRoverRuckus();

    private boolean vuforiaWebcam = false;

    //variable for the state engine, declared here so they are accessible throughout the entire opmode with having to pass them through each function
    private double mdblStep;                                 //Step from the step file, probably not needed
    private double mdblStepTimeout;                          //Timeout value ofthe step, the step will abort if the timeout is reached
    private String mstrRobotCommand;                         //The command the robot will execute, such as move forward, turn right etc
    private double mdblStepDistance;                         //used when decoding the step, this will indicate how far the robot is to move in inches
    private double mdblStepSpeed;                            //When a move command is executed this is the speed the motors will run at
    private boolean mblnParallel;                            //used to determine if next step will run in parallel - at same time
    private boolean mblnRobotLastPos;                        //used to determine if next step will run from end of last step or from encoder position
    private double mdblRobotParm1;                           //First Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mdblRobotParm2;                           //Second Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mdblRobotParm3;                           //Third Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mdblRobotParm4;                           //Fourth Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mdblRobotParm5;                           //Fifth Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mdblRobotParm6;                           //Sixth Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not

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
    private boolean flipit = false;
    private int quadrant;
    private int imuStartCorrectionVar = 0;
    private int imuMountCorrectionVar = 90;

    /**
     * Variables for the lift and remembering the current position
     */
    private int mintCurrentLiftCountMotor1          = 0;
    private int mintCurrentLiftCountMotor2          = 0;
    private int mintLiftStartCountMotor1            = 0;
    private int mintLiftStartCountMotor2            = 0;
    private double mintCurrentLiftPosCountMotor1    = mintCurrentLiftCountMotor1 - mintLiftStartCountMotor1;
    private double mintCurrentLiftPosCountMotor2    = mintCurrentLiftCountMotor2 - mintLiftStartCountMotor2;

    //hashmap for the steps to be stored in.  A Hashmap is like a fancy array
    //private HashMap<String, LibraryStateSegAutoRoverRuckus> autonomousSteps = new HashMap<String, LibraryStateSegAutoRoverRuckus>();
    private HashMap<String, String> powerTable = new HashMap<String, String>();
    private ReadStepFileRoverRuckus autonomousStepsFile = new ReadStepFileRoverRuckus();

    private RoverRuckusOCV elementColour = new RoverRuckusOCV();

    private ImageCaptureOCV imageCaptureOCV = new ImageCaptureOCV();
    private LibraryTensorFlowRoverRuckus tensorFlowRoverRuckus = new LibraryTensorFlowRoverRuckus();

    private int mintNumberColourTries = 0;
    private Constants.ObjectColours mColour;
    private Constants.ObjectColours mLocation;

    private TOWR5291TextToSpeech towr5291TextToSpeech = new TOWR5291TextToSpeech(false);

    //LED Strips
    private TOWR5291LEDControl LEDs;
    private Constants.LEDState mint5291LEDStatus = Constants.LEDState.STATE_NULL;

    private static TOWRDashBoard dashboard = null;

    public static TOWRDashBoard getDashboard() {
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

        dashboard = TOWRDashBoard.createInstance(telemetry);
        dashboard = TOWRDashBoard.getInstance();

        FtcRobotControllerActivity activity = (FtcRobotControllerActivity) hardwareMap.appContext;

        dashboard.setTextView((TextView) activity.findViewById(R.id.textOpMode));
        dashboard.clearDisplay();
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

        //load menu settings and setup robot and debug level
        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        ourRobotConfig = new robotConfig();

        ourRobotConfig.setAllianceColor(sharedPreferences.getString("club.towr5291.Autonomous.Color", "Red"));// Using a Function to Store The Robot Specification
        ourRobotConfig.setTeamNumber(sharedPreferences.getString("club.towr5291.Autonomous.TeamNumber", "0000"));
        ourRobotConfig.setAllianceStartPosition(sharedPreferences.getString("club.towr5291.Autonomous.Position", "Left"));
        ourRobotConfig.setDelay(Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Delay", "0")));
        ourRobotConfig.setRobotMotorType(sharedPreferences.getString("club.towr5291.Autonomous.RobotMotorChoice", "ANDY40SPUR"));
        ourRobotConfig.setRobotConfigBase(sharedPreferences.getString("club.towr5291.Autonomous.RobotConfigBase", "TileRunner2x40"));
        debug = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", "1"));

        switch (ourRobotConfig.getAllianceStartPosition()){
            case "Left":
                imuStartCorrectionVar = -45;
                break;
            default:
                imuStartCorrectionVar = 45;
                break;
        }

        //now we have loaded the config from sharedpreferences we can setup the robot
        ourRobotConfig.initConfig();

        //adjust debug level based on saved settings
        fileLogger.setDebugLevel(debug);

        fileLogger.writeEvent(1, "robotConfigTeam #  " + ourRobotConfig.getTeamNumber());
        fileLogger.writeEvent(1, "Alliance Colour    " + ourRobotConfig.getAllianceColor());
        fileLogger.writeEvent(1, "Alliance Start Pos " + ourRobotConfig.getAllianceStartPosition());
        fileLogger.writeEvent(1, "Alliance Delay     " + ourRobotConfig.getDelay());
        fileLogger.writeEvent(1, "Robot Config Base  " + ourRobotConfig.getRobotConfigBase());
        fileLogger.writeEvent(1, "Robot Motor Type  " + ourRobotConfig.getRobotMotorType());
        fileLogger.writeEvent(3, "Configuring Robot Parameters - Finished");
        fileLogger.writeEvent(3, "Loading Autonomous Steps - Start");

        dashboard.displayPrintf(1, "initRobot Loading Steps " + ourRobotConfig.getAllianceColor() + " Team " + ourRobotConfig.getTeamNumber());
        dashboard.displayPrintf(2, "initRobot SharePreferences!");
        dashboard.displayPrintf(3, "robotConfigTeam # " + ourRobotConfig.getTeamNumber());
        dashboard.displayPrintf(4, "Alliance          " + ourRobotConfig.getAllianceColor());
        dashboard.displayPrintf(5, "Start Pos         " + ourRobotConfig.getAllianceStartPosition());
        dashboard.displayPrintf(6, "Start Del         " + ourRobotConfig.getDelay());
        dashboard.displayPrintf(7, "Robot Base        " + ourRobotConfig.getRobotConfigBase());
        dashboard.displayPrintf(8, "Robot Motor Type  " + ourRobotConfig.getRobotMotorType());
        dashboard.displayPrintf(9, "Debug Level       " + debug);

        // Set up the LEDS
        LEDs = new TOWR5291LEDControl(hardwareMap, "green1", "red1", "blue1", "green2", "red2", "blue2");
        LEDs.setLEDControlDemoMode(false);
        LEDs.setLEDColour(Constants.LEDColours.LED_MAGENTA);
        LEDs.setLEDControlAlliance(ourRobotConfig.getAllianceColor());
        dashboard.displayPrintf(10, "initRobot LED Initiated!");

        dashboard.displayPrintf(10, "initRobot Limit Switch Initiated!");
        //load the sequence based on alliance colour and team

        autonomousStepsFile.ReadStepFile(ourRobotConfig);

        //need to load initial step of a delay based on user input
        autonomousStepsFile.insertSteps(ourRobotConfig.getDelay() + 1, "DELAY", 0, 0, false, false,ourRobotConfig.getDelay() * 1000, 0, 0, 0, 0, 0, 1);

        dashboard.displayPrintf(10, "initRobot STEPS LOADED");

        fileLogger.writeEvent(3, "Loading Autonomous Steps - Finished");
        fileLogger.writeEvent(3, "Configuring Adafruit IMU - Start");
        towr5291TextToSpeech.Speak("Step File Loaded!", debug);

        dashboard.displayPrintf(10, "initRobot IMU Loading");

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        towr5291TextToSpeech.Speak("Loading IMU Gyro", debug);
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

        dashboard.displayPrintf(10, "initRobot IMU Configured");

        fileLogger.writeEvent(3, "Configuring Adafruit IMU - Finished");
        fileLogger.writeEvent(3, "Configuring Motors Base - Start");

        dashboard.displayPrintf(10, "initRobot BaseDrive Loading");

        robotDrive.init(fileLogger, hardwareMap, robotConfigSettings.robotConfigChoice.valueOf(ourRobotConfig.getRobotConfigBase()), LibraryMotorType.MotorTypes.valueOf(ourRobotConfig.getRobotMotorType()));
        robotDrive.setHardwareDriveResetEncoders();
        robotDrive.setHardwareDriveRunUsingEncoders();

        dashboard.displayPrintf(10, "initRobot BaseDrive Loaded");
        fileLogger.writeEvent(3, "Configuring Motors Base - Finish");

        dashboard.displayPrintf(10, "Configuring Arm Motors - Start");
        fileLogger.writeEvent(3, "Configuring Arm Motors - Start");

        robotArms.init(hardwareMap , dashboard);
        robotArms.setHardwareLiftMotorResetEncoders();
        robotArms.setHardwareLiftMotorRunUsingEncoders();
        robotArms.setHardwareArmDirections();
        robotArms.teamMarkerServo.setPosition(mdblTeamMarkerHome);

        fileLogger.writeEvent(3, "Configuring Arm Motors - Finish");
        dashboard.displayPrintf(10, "Configuring Arm Motors - Finish");

        sensor.init(hardwareMap);
        fileLogger.writeEvent(3, "Resetting State Engine - Start");

        initDefaultStates();

        mint5291LEDStatus = Constants.LEDState.STATE_TEAM;
        mblnNextStepLastPos = false;

        fileLogger.writeEvent(3, "Resetting State Engine - Finish");
        fileLogger.writeEvent(3, "Configuring Vuforia - Start");

        dashboard.displayPrintf(10, "initRobot VUFORIA Loading");

        towr5291TextToSpeech.Speak("Loading OpenCV & Vuforia", debug);
        //init openCV
        initOpenCv();
        dashboard.displayPrintf(1, "initRobot OpenCV!");
        fileLogger.writeEvent(3, "OpenCV Started");

        //load all the vuforia stuff
        LibraryVuforiaRoverRuckus RoverRuckusVuforia = new LibraryVuforiaRoverRuckus();
        VuforiaTrackables RoverRuckusTrackables;

        if (vuforiaWebcam) {
            robotWebcam = hardwareMap.get(WebcamName.class, "Webcam 1");
            RoverRuckusTrackables = RoverRuckusVuforia.LibraryVuforiaRoverRuckus(hardwareMap, ourRobotConfig, robotWebcam, false);
        } else{
            RoverRuckusTrackables = RoverRuckusVuforia.LibraryVuforiaRoverRuckus(hardwareMap, ourRobotConfig, false);
        }

        imageCaptureOCV.initImageCaptureOCV(RoverRuckusVuforia, dashboard, fileLogger);
        //tensorFlowRoverRuckus.initTensorFlow(RoverRuckusVuforia.getVuforiaLocalizer(), hardwareMap, fileLogger, "RoverRuckus.tflite", "GOLD", "SILVER", true);

        fileLogger.writeEvent(3,"MAIN","Configured Vuforia - About to Activate");
        dashboard.displayPrintf(10, "Configured Vuforia - About to Activate");

        //activate vuforia
        RoverRuckusTrackables.activate();

        fileLogger.writeEvent(3,"MAIN", "Activated Vuforia");

        towr5291TextToSpeech.Speak("Completed Loading, Waiting for Start", debug);
        dashboard.displayPrintf(10, "Init - Complete, Wait for Start");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        fileLogger.setEventTag("opModeIsActive()");
        dashboard.clearDisplay();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 500);

        //the main loop.  this is where the action happens
        while (opModeIsActive()) {
            //adjust the LED state
            mint5291LEDStatus = LEDs.LEDControlUpdate(mint5291LEDStatus);

            //need to capture picture and process it.
//            if (!mblnDisableVisionProcessing) {
                //start capturing frames for analysis
//                if (mblnReadyToCapture) {
//
//                    try {
//                        VuforiaLocalizer.CloseableFrame frame = RoverRuckusVuforia.getVuforia().getFrameQueue().take(); //takes the frame at the head of the queue
//                        long numImages = frame.getNumImages();
//                        fileLogger.writeEvent(3, "VISION", "Number of Images " + numImages);
//
//                        for (int i = 0; i < numImages; i++) {
//                            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
//                                rgb = frame.getImage(i);
//                                break;
//                            }
//                        }
//
//                        /*rgb is now the Image object that we’ve used in the video*/
//                        Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
//                        bm.copyPixelsFromBuffer(rgb.getPixels());
//
//                        //put the image into a MAT for OpenCV
//                        tmp = new Mat(rgb.getWidth(), rgb.getHeight(), CvType.CV_8UC4);
//                        Utils.bitmapToMat(bm, tmp);
//                        //close the frame, prevents memory leaks and crashing
//                        frame.close();
//                    } catch (InterruptedException e) {
//                        dashboard.displayPrintf(1, "VUFORIA --- ERROR ERROR ERROR");
//                        dashboard.displayPrintf(2, "VUFORIA --- ERROR ERROR ERROR");
//                    }
//                    quadrant = 6;
//
//                    mColour = elementColour.RoverRuckusOCV(fileLogger, dashboard, tmp, 0, true, 6, false);
//                    fileLogger.writeEvent(3, "VISION", "Colour Returned " + mColour);
//                    dashboard.displayPrintf(2, "Element Detected-" + mColour);
//
//                    mintCaptureLoop++;
//                }
//            }

            switch (mintCurrentStateStep) {
                case STATE_INIT:
                    fileLogger.writeEvent(1,"mintCurrentStateStep:- " + mintCurrentStateStep + " mintCurrentStateStep " + mintCurrentStateStep);
                    fileLogger.writeEvent(1,"About to check if step exists " + mintCurrentStep);

                    // get step from hashmap, send it to the initStep for decoding
                    if (autonomousStepsFile.activeSteps().containsKey(String.valueOf(mintCurrentStep))) {
                        fileLogger.writeEvent(1,"Step Exists TRUE " + mintCurrentStep + " about to get the values from the step");
                        initStep();
                        mintCurrentStateStep = Constants.stepState.STATE_RUNNING;
                    } else {
                        mintCurrentStateStep = Constants.stepState.STATE_FINISHED;
                    }
                    break;
                case STATE_START:
                    mintCurrentStateStep = Constants.stepState.STATE_RUNNING;
                    break;
                case STATE_RUNNING:

                    //load all the parallel steps so they can be evaluated for completeness
                    loadParallelSteps();

                    //Process all the parallel steps
                    for (String stKey : mintActiveStepsCopy.keySet()) {
                        fileLogger.writeEvent(1, "STATE_RUNNING", "Looping through Parallel steps, found " + stKey);
                        mintStepNumber = mintActiveStepsCopy.get(stKey);
                        loadActiveStep(mintStepNumber);
                        fileLogger.writeEvent(1, "STATE_RUNNING", "About to run " + mstrRobotCommand);
                        processSteps(mstrRobotCommand);
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
                    robotArms.setHardwareLiftPower(0);
                    robotArms.setTiltLiftPower(0);
                    //  Transition to a new state.
                    mintCurrentStateStep = Constants.stepState.STATE_FINISHED;
                    break;
                case STATE_ERROR:
                    mint5291LEDStatus = Constants.LEDState.STATE_ERROR;
                    dashboard.displayPrintf(1, LABEL_WIDTH,"STATE", "ERROR WAITING TO FINISH " + mintCurrentStep);
                    break;
                case STATE_FINISHED:
                    mint5291LEDStatus = Constants.LEDState.STATE_FINISHED;
                    robotDrive.setHardwareDrivePower(0);
                    robotArms.setHardwareLiftPower(0);
                    robotArms.setTiltLiftPower(0);
                    imu.close();
                    //stop the logging
                    if (fileLogger != null) {
                        fileLogger.writeEvent(1, "Step FINISHED - FINISHED");
                        fileLogger.writeEvent(1, "Stopped");
                        Log.d("END:-", "FileLogger Stopped");
                        fileLogger.close();
                        fileLogger = null;
                    }
                    //deactivate vuforia
                    RoverRuckusTrackables.deactivate();
                    dashboard.displayPrintf(1, LABEL_WIDTH,"STATE", "FINISHED " + mintCurrentStep);
                    break;
            }
        }
        if (fileLogger != null) {
            fileLogger.writeEvent(1, "FINISHED AUTON - TIMED OUT");
            Log.d("END:-", "FINISHED AUTON - TIMED OUT - logger stopped");
            fileLogger.close();
            fileLogger = null;
        }

        //tensorFlowRoverRuckus.shutdown();

        //switch opmode to teleop
        //opModeManager = (OpModeManagerImpl) onStop.internalOpModeServices;
        //opModeManager.initActiveOpMode(TeleOpMode);
        //opmode not active anymore
    }

    private void loadActiveStep(int step) {
        fileLogger.setEventTag("loadActiveStep()");
        LibraryStateSegAutoRoverRuckus mStateSegAuto = autonomousStepsFile.activeSteps().get(String.valueOf(step));
        fileLogger.writeEvent(1,"Got the values for step " + step + " about to decode");

        mdblStep            = mStateSegAuto.getmStep();
        mdblStepTimeout     = mStateSegAuto.getmRobotTimeOut();
        mstrRobotCommand    = mStateSegAuto.getmRobotCommand();
        mdblStepDistance    = mStateSegAuto.getmRobotDistance();
        mdblStepSpeed       = mStateSegAuto.getmRobotSpeed();
        mblnParallel        = mStateSegAuto.getmRobotParallel();
        mblnRobotLastPos    = mStateSegAuto.getmRobotLastPos();
        mdblRobotParm1      = mStateSegAuto.getmRobotParm1();
        mdblRobotParm2      = mStateSegAuto.getmRobotParm2();
        mdblRobotParm3      = mStateSegAuto.getmRobotParm3();
        mdblRobotParm4      = mStateSegAuto.getmRobotParm4();
        mdblRobotParm5      = mStateSegAuto.getmRobotParm5();
        mdblRobotParm6      = mStateSegAuto.getmRobotParm6();

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
        fileLogger.writeEvent(2,"Processing Parallel Step " + stepName);
        switch (stepName) {
            case "DELAY":
                DelayStep();
                break;
            case "TANKTURN":
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
            case "STRAFE":
                MecanumStrafe();
                break;
            case "RRE":  // Right turn with a Radius in Parm 1
            case "LRE":  // Left turn with a Radius in Parm 1
                RadiusTurnStep();
                break;
            case "DRIVE":  // Drive forward a distance in inches and power setting
                DriveStepHeading();
                break;
            case "VME":  // Move the robot using localisation from the targets
                VuforiaMove();
                break;
            case "VTE":  // Turn the Robot using information from Vuforia and Pythag
                VuforiaTurn();
                break;
            case "LIFT":    // Moves the lift up and down for the 2018-19 game
                moveLiftUpDown();
                break;
            case "INTAKE":
                SetIntake();
                break;
            case "TILT":
                tiltMotor();
                break;
            case "FINDGOLD":
                findGold();
                break;
            case "TEAMMARKER":
                releaseTeamMarker();
                break;
            case "WYATTGYRO":
                WyattsGyroDrive();
                break;
        }
    }

    //--------------------------------------------------------------------------
    //  Initialise the state.
    //--------------------------------------------------------------------------
    private void initStep() {
        fileLogger.setEventTag("initStep()");
        fileLogger.writeEvent(3,"Starting to Decode Step " + mintCurrentStep);

        if (!(mintActiveSteps.containsValue(mintCurrentStep))) {
            mintActiveSteps.put(String.valueOf(mintCurrentStep), mintCurrentStep);
            fileLogger.writeEvent(3,"Put step into hashmap mintActiveSteps " + mintCurrentStep);
        }

        loadActiveStep(mintCurrentStep);
        // Reset the state time, and then change to next state.
        mStateTime.reset();

        switch (mstrRobotCommand) {
            case "DELAY":
                mintCurrentStepDelay                = Constants.stepState.STATE_INIT;
                towr5291TextToSpeech.Speak("Running Delay", debug);
                break;
            case "GTH":
                mintCurrentStateTankTurnGyroHeading = Constants.stepState.STATE_INIT;
                towr5291TextToSpeech.Speak("Running Tank Turn Gyro Heading", debug);
                break;
            case "STRAFE":
                mintCurrentStateMecanumStrafe       = Constants.stepState.STATE_INIT;
                towr5291TextToSpeech.Speak("Running Mecanum Strafe", debug);
                break;
            case "TANKTURN":
                mintCurrentStateTankTurn            = Constants.stepState.STATE_INIT;
                towr5291TextToSpeech.Speak("Running Tank Turn", debug);
                break;
            case "LPE":
                mintCurrentStatePivotTurn           = Constants.stepState.STATE_INIT;
                towr5291TextToSpeech.Speak("Running Pivot Turn", debug);
                break;
            case "RPE":
                mintCurrentStatePivotTurn           = Constants.stepState.STATE_INIT;
                towr5291TextToSpeech.Speak("Running Pivot Turn", debug);
                break;
            case "LRE":  // Left turn with a Radius in Parm 1
                mintCurrentStateRadiusTurn          = Constants.stepState.STATE_INIT;
                towr5291TextToSpeech.Speak("Running Radius Turn", debug);
                break;
            case "RRE":  // Right turn with a Radius in Parm 1
                mintCurrentStateRadiusTurn          = Constants.stepState.STATE_INIT;
                towr5291TextToSpeech.Speak("Running Radius Turn", debug);
                break;
            case "DRIVE":  // Drive forward a distance in inches and power setting
                mintCurrentStateDriveHeading        = Constants.stepState.STATE_INIT;
                towr5291TextToSpeech.Speak("Running Drive Heading", debug);
                break;
            case "VME":  // Move the robot using localisation from the targets
                mintCurStVuforiaMove5291            = Constants.stepState.STATE_INIT;
                towr5291TextToSpeech.Speak("Running Vuforia Move", debug);
                break;
            case "VTE":  // Turn the Robot using information from Vuforia and Pythag
                mintCurStVuforiaTurn5291            = Constants.stepState.STATE_INIT;
                towr5291TextToSpeech.Speak("Running Vuforia Turn", debug);
                break;
            case "GTE":  // Special Function, 5291 Move forward until line is found
                mintCurrentStateGyroTurnEncoder5291 = Constants.stepState.STATE_INIT;
                towr5291TextToSpeech.Speak("Running Gyro Turn Encoder", debug);
                break;
            case "EYE":  // Special Function, 5291 Move forward until line is found
                mintCurrentStateEyes5291            = Constants.stepState.STATE_INIT;
                towr5291TextToSpeech.Speak("Running Move Eyes", debug);
                break;
            case "LIFT":
                mintCurrentStateMoveLift            = Constants.stepState.STATE_INIT;
                towr5291TextToSpeech.Speak("Running Move Lift", debug);
                break;
            case "INTAKE":
                mintCurrentStateInTake              = Constants.stepState.STATE_INIT;
                towr5291TextToSpeech.Speak("Running Set Intake", debug);
                break;
            case "TILT":
                mintCurrentStateTiltMotor           = Constants.stepState.STATE_INIT;
                towr5291TextToSpeech.Speak("Running Tilt Motor", debug);
                break;
            case "FINDGOLD":
                mintCurrentStateFindGold            = Constants.stepState.STATE_INIT;
                towr5291TextToSpeech.Speak("Running Find Gold", debug);
                break;
            case "WYATTGYRO":
                mintCurrentStateWyattsGyroDrive     = Constants.stepState.STATE_INIT;
                towr5291TextToSpeech.Speak("Running Wyatt's Gyro Drive", debug);
                break;
            case "FNC":  //  Run a special Function with Parms

                break;
            case "TEAMMARKER":
                mintCurrentStateTeamMarker          = Constants.stepState.STATE_INIT;
                towr5291TextToSpeech.Speak("Running Set Team Marker Servo Position", debug);
                break;
        }

        fileLogger.writeEvent(2,"Current Step          :- " + mintCurrentStep);
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
                fileLogger.writeEvent(2,"mdblStepDistance   :- " + mdblStepDistance);
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

                mintStepLeftTarget1 = mintStartPositionLeft1 + (int) (mdblStepDistance * ourRobotConfig.getCOUNTS_PER_INCH());
                mintStepLeftTarget2 = mintStartPositionLeft2 + (int) (mdblStepDistance * ourRobotConfig.getCOUNTS_PER_INCH());
                mintStepRightTarget1 = mintStartPositionRight1 + (int) (mdblStepDistance * ourRobotConfig.getCOUNTS_PER_INCH());
                mintStepRightTarget2 = mintStartPositionRight2 + (int) (mdblStepDistance * ourRobotConfig.getCOUNTS_PER_INCH());

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
                if ((mdblRobotParm1 == 1) || (mdblRobotParm4 == 1)) {
                    //use Gyro to run heading
                    // adjust relative speed based on heading error.
                    dblError = getDriveError(mdblRobotParm1);
                    dblSteer = getDriveSteer(dblError, mdblRobotParm1);
                    fileLogger.writeEvent(3,"dblError " + dblError);
                    fileLogger.writeEvent(3,"dblSteer " + dblSteer);
                    fileLogger.writeEvent(3, "runningDriveHeadingStep", "Heading " + mdblRobotParm2);

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (mdblStepDistance < 0)
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
                if (((robotDrive.getHardwareDriveIsBusy() & (robotConfig.motors.leftMotor1.toInt() | robotConfig.motors.rightMotor1.toInt())) == (robotConfig.motors.leftMotor1.toInt() | robotConfig.motors.rightMotor1.toInt()))) {
                    fileLogger.writeEvent(3,"Encoder counts per inch = " + ourRobotConfig.getCOUNTS_PER_INCH() + " dblDistanceFromStart " + dblDistanceFromStart + " dblDistanceToEnd " + dblDistanceToEnd + " Power Level Left " + dblStepSpeedTempLeft + " Power Level Right " + dblStepSpeedTempRight + " Running to target  L1, L2, R1, R2  " + mintStepLeftTarget1 + ", " + mintStepLeftTarget2 + ", " + mintStepRightTarget1 + ",  " + mintStepRightTarget2 + ", " + " Running at position L1 " + intLeft1MotorEncoderPosition + " L2 " + intLeft2MotorEncoderPosition + " R1 " + intRight1MotorEncoderPosition + " R2 " + intRight2MotorEncoderPosition);
                    dashboard.displayPrintf(3, LABEL_WIDTH,"Path1", "Running to " + mintStepLeftTarget1 + ":" + mintStepRightTarget1);
                    dashboard.displayPrintf(4, LABEL_WIDTH,"Path2", "Running at " +intLeft1MotorEncoderPosition + ":" + intRight1MotorEncoderPosition);
                    dashboard.displayPrintf(5, LABEL_WIDTH,"Path3", "Running at " + intLeft2MotorEncoderPosition + ":" + intRight2MotorEncoderPosition);
                    // set power on motor controller to update speeds
                    robotDrive.setHardwareDriveLeftMotorPower(dblStepSpeedTempLeft);
                    robotDrive.setHardwareDriveRightMotorPower(dblStepSpeedTempRight);
                } else {
                    robotDrive.setHardwareDrivePower(0);
                    fileLogger.writeEvent(2,"Complete         ");
                    mblnDisableVisionProcessing = false;  //enable vision processing
                    mintCurrentStateDriveHeading = Constants.stepState.STATE_COMPLETE;
                    deleteParallelStep();
                }

                //check timeout value
                if (mStateTime.seconds() > mdblStepTimeout) {// Stop all motion;
                    robotDrive.setHardwareDrivePower(0);
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

                if (mdblStepDistance > 0) {
                    mdblStepTurnL = mdblStepDistance;
                    mdblStepTurnR = 0;
                } else {
                    mdblStepTurnL = 0;
                    mdblStepTurnR = mdblStepDistance;
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
                    dashboard.displayPrintf(3, LABEL_WIDTH, "Target", "Running to %7d :%7d", mintStepLeftTarget1, mintStepRightTarget1);
                    dashboard.displayPrintf(4, LABEL_WIDTH,"Actual_Left", "Running at %7d :%7d", intLeft1MotorEncoderPosition, intLeft2MotorEncoderPosition);
                    dashboard.displayPrintf(5, LABEL_WIDTH,"ActualRight", "Running at %7d :%7d", intRight1MotorEncoderPosition, intRight2MotorEncoderPosition);

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
                    if (((robotDrive.getHardwareDriveIsBusy() & robotConfig.motors.leftMotor1.toInt()) == robotConfig.motors.leftMotor1.toInt())) {
                        fileLogger.writeEvent(1,"Complete........");
                        mblnDisableVisionProcessing = false;  //enable vision processing
                        mintCurrentStatePivotTurn = Constants.stepState.STATE_COMPLETE;
                        deleteParallelStep();
                    }
                } else if (mdblStepTurnL == 0) {
                    fileLogger.writeEvent(3,"Running.......");
                    dashboard.displayPrintf(3, LABEL_WIDTH,"Target", "Running to %7d :%7d", mintStepLeftTarget1, mintStepRightTarget1);
                    dashboard.displayPrintf(4, LABEL_WIDTH,"Actual_Left", "Running at %7d :%7d", intLeft1MotorEncoderPosition, intLeft2MotorEncoderPosition);
                    dashboard.displayPrintf(5, LABEL_WIDTH,"ActualRight", "Running at %7d :%7d", intRight1MotorEncoderPosition, intRight2MotorEncoderPosition);

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
                    if (((robotDrive.getHardwareDriveIsBusy() & robotConfig.motors.rightMotor1.toInt()) == robotConfig.motors.rightMotor1.toInt())) {
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
                if (mdblStepDistance > 0) {
                    mintStepLeftTarget1 = mintStartPositionLeft1 + (int) (ourRobotConfig.getMecanumTurnOffset() * 0.5 * Math.abs(mdblStepDistance) * ourRobotConfig.getCOUNTS_PER_DEGREE());
                    mintStepLeftTarget2 = mintStartPositionLeft2 + (int) (ourRobotConfig.getMecanumTurnOffset() * 0.5 * Math.abs(mdblStepDistance) * ourRobotConfig.getCOUNTS_PER_DEGREE());
                    mintStepRightTarget1 = mintStartPositionRight1 - (int) (ourRobotConfig.getMecanumTurnOffset() * 0.5 * Math.abs(mdblStepDistance) * ourRobotConfig.getCOUNTS_PER_DEGREE());
                    mintStepRightTarget2 = mintStartPositionRight2 - (int) (ourRobotConfig.getMecanumTurnOffset() * 0.5 * Math.abs(mdblStepDistance) * ourRobotConfig.getCOUNTS_PER_DEGREE());
                } else {
                    mintStepLeftTarget1 = mintStartPositionLeft1 - (int) (ourRobotConfig.getMecanumTurnOffset() * 0.5 * Math.abs(mdblStepDistance) * ourRobotConfig.getCOUNTS_PER_DEGREE());
                    mintStepLeftTarget2 = mintStartPositionLeft2 - (int) (ourRobotConfig.getMecanumTurnOffset() * 0.5 * Math.abs(mdblStepDistance) * ourRobotConfig.getCOUNTS_PER_DEGREE());
                    mintStepRightTarget1 = mintStartPositionRight1 + (int) (ourRobotConfig.getMecanumTurnOffset() * 0.5 * Math.abs(mdblStepDistance) * ourRobotConfig.getCOUNTS_PER_DEGREE());
                    mintStepRightTarget2 = mintStartPositionRight2 + (int) (ourRobotConfig.getMecanumTurnOffset() * 0.5 * Math.abs(mdblStepDistance) * ourRobotConfig.getCOUNTS_PER_DEGREE());
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

                mdblRobotTurnAngle = mdblStepDistance;
                fileLogger.writeEvent(3,"mdblRobotTurnAngle" + mdblRobotTurnAngle);

                //calculate the distance to travel based on the angle we are turning
                // length = radius x angle (in radians)
                rdblArcLengthRadiusTurnOuter = ((mdblStepDistance / 180) * Math.PI) * mdblRobotParm1;
                dblArcLengthRadiusTurnInner = ((mdblStepDistance / 180) * Math.PI) * (mdblRobotParm1 - (ourRobotConfig.getROBOT_TRACK()));
                //rdblArcLengthRadiusTurnOuter = (mdblStepDistance / 180) *  Math.PI) * (mdblRobotParm1 + (0.5 * ROBOT_TRACK));

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
                if (mdblStepDistance > 0) {
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
                } else {
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
                dashboard.displayPrintf(3, LABEL_WIDTH, "Target: ", "Running to %7d :%7d", mintStepLeftTarget1, mintStepRightTarget1);
                dashboard.displayPrintf(4, LABEL_WIDTH, "Actual_Left: ", "Running at %7d :%7d", intLeft1MotorEncoderPosition, intLeft2MotorEncoderPosition);
                dashboard.displayPrintf(5, LABEL_WIDTH, "ActualRight: ", "Running at %7d :%7d", intRight1MotorEncoderPosition, intRight2MotorEncoderPosition);

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

                robotDrive.setHardwareDriveRunUsingEncoders();
                mblnDisableVisionProcessing = true;  //disable vision processing

                adafruitIMUHeading = getAdafruitHeading();
                currentHeading = adafruitIMUHeading;
                //mdblRobotTurnAngle = Double.parseDouble(mdblStepDistance);
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

                mintStepLeftTarget1 = mintStartPositionLeft1 - (int) (mdblStepDistance * ourRobotConfig.getCOUNTS_PER_INCH_STRAFE_LEFT_OFFSET() * ourRobotConfig.getCOUNTS_PER_INCH_STRAFE() * ourRobotConfig.getCOUNTS_PER_INCH_STRAFE_FRONT_OFFSET());
                mintStepLeftTarget2 = mintStartPositionLeft2 + (int) (mdblStepDistance * ourRobotConfig.getCOUNTS_PER_INCH_STRAFE_LEFT_OFFSET() * ourRobotConfig.getCOUNTS_PER_INCH_STRAFE() * ourRobotConfig.getCOUNTS_PER_INCH_STRAFE_REAR_OFFSET());
                mintStepRightTarget1 = mintStartPositionRight1 + (int) (mdblStepDistance * ourRobotConfig.getCOUNTS_PER_INCH_STRAFE_RIGHT_OFFSET() * ourRobotConfig.getCOUNTS_PER_INCH_STRAFE() * ourRobotConfig.getCOUNTS_PER_INCH_STRAFE_FRONT_OFFSET());
                mintStepRightTarget2 = mintStartPositionRight2 - (int) (mdblStepDistance * ourRobotConfig.getCOUNTS_PER_INCH_STRAFE_RIGHT_OFFSET() * ourRobotConfig.getCOUNTS_PER_INCH_STRAFE() * ourRobotConfig.getCOUNTS_PER_INCH_STRAFE_REAR_OFFSET());

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
                dashboard.displayPrintf(4,  "Mecanum Strafe Positions moving " + mdblStepDistance);
                dashboard.displayPrintf(5, LABEL_WIDTH, "Left  Target: ", "Running to %7d :%7d", mintStepLeftTarget1, mintStepLeftTarget2);
                dashboard.displayPrintf(6, LABEL_WIDTH, "Left  Actual: ", "Running at %7d :%7d", intLeft1MotorEncoderPosition, intLeft2MotorEncoderPosition);
                dashboard.displayPrintf(7, LABEL_WIDTH, "Right Target: ", "Running to %7d :%7d", mintStepRightTarget1, mintStepRightTarget2);
                dashboard.displayPrintf(8, LABEL_WIDTH, "Right Actual: ", "Running at %7d :%7d", intRight1MotorEncoderPosition, intRight2MotorEncoderPosition);

                if (mblnRobotLastPos) {
                    if ((((dblDistanceToEndRight1 + dblDistanceToEndRight2) / 2) < 2) && (((dblDistanceToEndLeft1 + dblDistanceToEndLeft2) / 2) < 2)) {
                        mblnNextStepLastPos = true;
                        mintCurrentStateMecanumStrafe = Constants.stepState.STATE_COMPLETE;
                        deleteParallelStep();
                    }
                }

                if (!robotDrive.baseMotor1.isBusy() || (!robotDrive.baseMotor3.isBusy()) || (!robotDrive.baseMotor2.isBusy()) || (!robotDrive.baseMotor4.isBusy())) {
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

    double mdblWyattsGyroDriveStartAngle = 0;

    private void WyattsGyroDrive(){
        switch(mintCurrentStateWyattsGyroDrive){
            case STATE_INIT:
                fileLogger.setEventTag("WyattsGyroDrive");

                mdblWyattsGyroDriveStartAngle = getAdafruitHeading();
                fileLogger.writeEvent(5,"Starting Gyro Value is: " + mdblWyattsGyroDriveStartAngle);

                // Determine new target position, and pass to motor controller
                int mintWyattsGyroDriveMoveCounts = (int)(mdblStepDistance * ourRobotConfig.getCOUNTS_PER_INCH());
                fileLogger.writeEvent(3, "move counts in Wyatt Gyro Drive is: " + mintWyattsGyroDriveMoveCounts);

                robotDrive.baseMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robotDrive.baseMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robotDrive.baseMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robotDrive.baseMotor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // Set Target and Turn On RUN_TO_POSITION
                robotDrive.baseMotor1.setTargetPosition(robotDrive.baseMotor1.getCurrentPosition() + mintWyattsGyroDriveMoveCounts);
                robotDrive.baseMotor2.setTargetPosition(robotDrive.baseMotor2.getCurrentPosition() + mintWyattsGyroDriveMoveCounts);
                robotDrive.baseMotor3.setTargetPosition(robotDrive.baseMotor3.getCurrentPosition() + mintWyattsGyroDriveMoveCounts);
                robotDrive.baseMotor4.setTargetPosition(robotDrive.baseMotor4.getCurrentPosition() + mintWyattsGyroDriveMoveCounts);

                robotDrive.setHardwareDrivePower(mdblStepSpeed);
                break;

            case STATE_RUNNING:
                // adjust relative speed based on heading error.
                double driveError = getDriveError(mdblWyattsGyroDriveStartAngle);
                fileLogger.writeEvent(3, "Drive Error in Wyatt Gryo Drive is: " + driveError);
                fileLogger.writeEvent(3, "Current Gyro Value: " + getAdafruitHeading());
                double steer = getDriveSteer(driveError, mdblRobotParm1);
                fileLogger.writeEvent(3, "Drive Steer in Wyatt Gyro Drive is" + steer);

                // if driving in reverse, the motor correction also needs to be reversed
                if (mdblStepDistance < 0)
                    steer *= -1.0;

                double leftSpeed = mdblStepSpeed - steer;
                fileLogger.writeEvent(3, "Left Motor Speed in Wyatts Gyro Drive is: " + leftSpeed);
                double rightSpeed = mdblStepSpeed + steer;
                fileLogger.writeEvent(3, "Right Motor Speed in Wyatts Gyro Drive is: " + rightSpeed);

                // Normalize speeds if either one exceeds +/- 1.0;
                double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robotDrive.setHardwareDriveLeftMotorPower(leftSpeed);
                robotDrive.setHardwareDriveRightMotorPower(rightSpeed);

                if (!robotDrive.baseMotor1.isBusy() && !robotDrive.baseMotor2.isBusy() &&
                    !robotDrive.baseMotor3.isBusy() && !robotDrive.baseMotor4.isBusy()){

                    mintCurrentStateWyattsGyroDrive = Constants.stepState.STATE_COMPLETE;

                    robotDrive.setHardwareDrivePower(0);

                    // Turn off RUN_TO_POSITION
                    robotDrive.baseMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robotDrive.baseMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robotDrive.baseMotor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robotDrive.baseMotor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    fileLogger.writeEvent(2, "Basemotor turned off Run To Position in Wyatts Gyro Drive");
                }

                if (mStateTime.seconds() > mdblStepTimeout) {
                    robotDrive.setHardwareDrivePower(0);
                    fileLogger.writeEvent(1,"Timeout:- " + mStateTime.seconds());
                    //  Transition to a new state.
                    mintCurrentStateMoveLift = Constants.stepState.STATE_COMPLETE;
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
                mdblRobotTurnAngle = mdblStepDistance;
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
                mdblRobotTurnAngle = mdblStepDistance;
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
                autonomousStepsFile.insertSteps(3,  newAngleDirectionGyro ((int)mdblGyrozAccumulated, (int)mdblRobotTurnAngle),(int)mdblRobotTurnAngle,mdblStepSpeed,false, false, 0, 0, 0, 0, 0, 0,  mintCurrentStep + 1);
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

    private void moveLiftUpDown(){

        double dblDistanceToMoveLift1;
        double dblDistanceToMoveLift2;

        fileLogger.setEventTag("moveLiftUpDown()");

        switch (mintCurrentStateMoveLift){
            case STATE_INIT:
                fileLogger.writeEvent(2, "Initialised");
                robotArms.setHardwareLiftMotorRunUsingEncoders();
                fileLogger.writeEvent(5, "Using Encoders");
                robotArms.setHardwareLiftMotorResetEncoders();
                fileLogger.writeEvent(5, "Resetting Encoders");

                //check timeout value
                if (mStateTime.milliseconds() > mdblRobotParm1) {
                    if (mdblRobotParm2 == 0){
                        dblDistanceToMoveLift1 = robotArms.getLiftMotor1Encoder() + (mdblStepDistance * ourRobotConfig.getLIFTMAIN_COUNTS_PER_INCH());
                        dblDistanceToMoveLift2 = robotArms.getLiftMotor2Encoder() + (mdblStepDistance * ourRobotConfig.getLIFTMAIN_COUNTS_PER_INCH());
                    } else if (mdblRobotParm2 == 1){
                        double distaceInCounts = (mdblStepDistance * ourRobotConfig.getLIFTMAIN_COUNTS_PER_INCH());
                        dblDistanceToMoveLift1 = (robotArms.getLiftMotor1Encoder() - mintLiftStartCountMotor1) + distaceInCounts;
                        dblDistanceToMoveLift2 = (robotArms.getLiftMotor2Encoder() - mintLiftStartCountMotor2) + distaceInCounts;
                    } else {
                        dblDistanceToMoveLift1 = 0;
                        dblDistanceToMoveLift2 = 0;
                        fileLogger.writeEvent("ERROR ERROR ERROR ERROR ERROR ERROR ERROR");
                        fileLogger.writeEvent("ERROR ERROR ERROR ERROR ERROR ERROR ERROR");
                        fileLogger.writeEvent("ERROR MOVING LIFT PARM 1 IS THE MODE CSN BE 0 OR 1");
                        fileLogger.writeEvent("Mode 1 is to move a certain distance so like move out 1 more inch");
                        fileLogger.writeEvent("Mode 2 is to move to a spot so move to 5 inches on the lift");
                    }
                    robotArms.liftMotor1.setTargetPosition((int)dblDistanceToMoveLift1);
                    robotArms.liftMotor2.setTargetPosition((int)dblDistanceToMoveLift2);

                    robotArms.setHardwareLiftPower(mdblStepSpeed);

                    robotArms.setHardwareLiftMotorRunToPosition();

                    mintCurrentStateMoveLift = Constants.stepState.STATE_RUNNING;
                }
                break;
            case STATE_RUNNING:
                fileLogger.writeEvent(2, "Running");

                robotArms.setHardwareLiftPower(mdblStepSpeed);
                fileLogger.writeEvent(2, "Motor Speed Set: Busy 1 " + robotArms.liftMotor1.isBusy() + " Busy 2 " + robotArms.liftMotor2.isBusy());

                if ((!(robotArms.liftMotor1.isBusy())) || (!(robotArms.liftMotor2.isBusy()))){
                    fileLogger.writeEvent(2, "Motor 1 is not busy");
                    fileLogger.writeEvent(2, "Motor 2 is not busy");
                    robotArms.setHardwareLiftPower(0);
                    fileLogger.writeEvent(5, "LIFT MOTOR POWER IS 0");
                    mintCurrentLiftCountMotor1 = robotArms.liftMotor1.getCurrentPosition();
                    mintCurrentLiftCountMotor2 = robotArms.liftMotor2.getCurrentPosition();
                    fileLogger.writeEvent(5, "Lift Motor 1 Current Encoder Count: " + String.valueOf(mintCurrentLiftCountMotor1));
                    fileLogger.writeEvent(5, "Lift Motor 2 Current Encoder Count: " + String.valueOf(mintCurrentLiftCountMotor2));
                    fileLogger.writeEvent(2, "Finished");
                    mintCurrentStateMoveLift = Constants.stepState.STATE_COMPLETE;
                    deleteParallelStep();
                }
                //check timeout value
                if (mStateTime.seconds() > mdblStepTimeout) {
                    robotArms.setHardwareLiftPower(0);
                    fileLogger.writeEvent(1,"Timeout:- " + mStateTime.seconds());
                    mintCurrentLiftCountMotor1 = robotArms.liftMotor1.getCurrentPosition();
                    mintCurrentLiftCountMotor2 = robotArms.liftMotor2.getCurrentPosition();
                    //  Transition to a new state.
                    mintCurrentStateMoveLift = Constants.stepState.STATE_COMPLETE;
                    deleteParallelStep();
                }
                break;
        }
    }

    private void SetIntake(){
        fileLogger.setEventTag("SetIntake()");

        switch (mintCurrentStateInTake){
            case STATE_INIT:
                fileLogger.writeEvent(2,"Initialised");
                fileLogger.writeEvent(2,"Power: " + String.valueOf(mdblStepSpeed));
                robotArms.intakeServo1.setPosition(mdblStepSpeed);
                robotArms.intakeServo2.setPosition(mdblStepSpeed);
                mintCurrentStateInTake = Constants.stepState.STATE_RUNNING;
                break;
            case STATE_RUNNING:
                fileLogger.writeEvent(2,"Running");
                fileLogger.writeEvent(2,"Power: " + String.valueOf(mdblStepSpeed));
                //robotArms.intakeMotor.setPower(mdblStepSpeed);
                if (mdblStepSpeed == 0) {
                    fileLogger.writeEvent(1,"Complete.......");
                    mintCurrentStateInTake = Constants.stepState.STATE_COMPLETE;
                    deleteParallelStep();
                    break;
                }

                if (mStateTime.milliseconds() >= mdblRobotParm1)
                {
                    fileLogger.writeEvent(1,"Timer Complete.......");
                    mintCurrentStateInTake = Constants.stepState.STATE_COMPLETE;
                    deleteParallelStep();
                    break;
                }//check timeout value

                if (mStateTime.seconds() > mdblStepTimeout) {
                    robotArms.intakeServo1.setPosition(0);
                    robotArms.intakeServo2.setPosition(0);
                    fileLogger.writeEvent(1,"Timeout:- " + mStateTime.seconds());
                    //  Transition to a new state.
                    mintCurrentStateInTake = Constants.stepState.STATE_COMPLETE;
                    deleteParallelStep();
                }
                break;
        }
    }

    private void tiltMotor(){
        double dblDistanceToMoveTilt1;
        double dblDistanceToMoveTilt2;

        fileLogger.setEventTag("tiltMotor()");

        switch (mintCurrentStateTiltMotor){
            case STATE_INIT:
                fileLogger.writeEvent(2, "Initialised");
                robotArms.tiltMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robotArms.tiltMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                dblDistanceToMoveTilt1 = robotArms.tiltMotor1.getCurrentPosition() + (mdblStepDistance * ourRobotConfig.getCOUNTS_PER_DEGREE_TILT());
                dblDistanceToMoveTilt2 = robotArms.tiltMotor2.getCurrentPosition() + (mdblStepDistance * ourRobotConfig.getCOUNTS_PER_DEGREE_TILT());

                fileLogger.writeEvent(2, "Distance to move = " + mdblStepDistance + ", dblDistanceToMoveTilt1 " + dblDistanceToMoveTilt1);
                fileLogger.writeEvent(2, "Distance to move = " + mdblStepDistance + ", dblDistanceToMoveTilt2 " + dblDistanceToMoveTilt2);

                robotArms.tiltMotor1.setTargetPosition((int)dblDistanceToMoveTilt1);
                robotArms.tiltMotor2.setTargetPosition((int)dblDistanceToMoveTilt2);

                robotArms.tiltMotor1.setPower(mdblStepSpeed);
                robotArms.tiltMotor2.setPower(mdblStepSpeed);

                robotArms.tiltMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robotArms.tiltMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                mintCurrentStateTiltMotor = Constants.stepState.STATE_RUNNING;
                break;
            case STATE_RUNNING:
                fileLogger.writeEvent(2, "Running");

                robotArms.tiltMotor1.setPower(mdblStepSpeed);
                robotArms.tiltMotor2.setPower(mdblStepSpeed);

                fileLogger.writeEvent(2, "Motor Speed Set: Busy 1 " + robotArms.tiltMotor1.isBusy());
                fileLogger.writeEvent(2, "Motor Speed Set: Busy 2 " + robotArms.tiltMotor2.isBusy());

                fileLogger.writeEvent(3, "Current Encoder 1 = " + robotArms.tiltMotor1.getCurrentPosition() + " Expected Encoder " + robotArms.tiltMotor1.getTargetPosition());
                fileLogger.writeEvent(3, "Current Encoder 2 = " + robotArms.tiltMotor1.getCurrentPosition() + " Expected Encoder " + robotArms.tiltMotor2.getTargetPosition());

                if (mdblRobotParm1 > 0){
                    if ((robotArms.tiltMotor1.getCurrentPosition() < (robotArms.tiltMotor1.getTargetPosition() + mdblRobotParm1))
                        && (robotArms.tiltMotor1.getCurrentPosition() > (robotArms.tiltMotor1.getTargetPosition() - mdblRobotParm1))
                        && (robotArms.tiltMotor2.getCurrentPosition() < (robotArms.tiltMotor2.getTargetPosition() + mdblRobotParm1))
                        && (robotArms.tiltMotor2.getCurrentPosition() > (robotArms.tiltMotor2.getTargetPosition() - mdblRobotParm1))) {

                        fileLogger.writeEvent(2, "Motor 1 is near the target Stopping Motor");
                        fileLogger.writeEvent(2, "Motor 2 is near the target Stopping Motor");

                        fileLogger.writeEvent(2, "Finished");
                        mintCurrentStateTiltMotor = Constants.stepState.STATE_COMPLETE;
                        deleteParallelStep();
                        break;
                    }
                } else {
                    if (!(robotArms.tiltMotor1.isBusy()) && !(robotArms.tiltMotor2.isBusy())) {
                        fileLogger.writeEvent(2, "Motor 1 is not busy");
                        fileLogger.writeEvent(2, "Motor 2 is not busy");
                        //robotArms.tiltMotor1.setPower(0);
                        //robotArms.tiltMotor2.setPower(0);
                        fileLogger.writeEvent(2, "Finished");
                        mintCurrentStateTiltMotor = Constants.stepState.STATE_COMPLETE;
                        deleteParallelStep();
                        break;
                    }
                }
                //check timeout value
                if (mStateTime.seconds() > mdblStepTimeout) {
                    robotArms.tiltMotor1.setPower(0);
                    robotArms.tiltMotor2.setPower(0);
                    fileLogger.writeEvent(1,"Timeout:- " + mStateTime.seconds());
                    //  Transition to a new state.
                    mintCurrentStateTiltMotor = Constants.stepState.STATE_COMPLETE;
                    deleteParallelStep();
                }
                break;
        }
    }

    private void findGold(){

        fileLogger.setEventTag("findGold()");

        switch (mintCurrentStateFindGold){
            case STATE_INIT:
                if (mStateTime.milliseconds() > mdblRobotParm1) {
                    fileLogger.writeEvent(3, "Initialised");
                    mintFindGoldLoop = (int) mdblRobotParm6;
                    mboolFoundGold = false;

                    imageCaptureOCV.takeImage(new ImageCaptureOCV.OnImageCapture() {
                        @Override
                        public void OnImageCaptureVoid(Mat mat) {
                            //mColour = elementColour.RoverRuckusOCV(fileLogger, dashboard, mat, 0, true, 6, false);
                            //check if gold is middle
                            if (elementColour.RoverRuckusOCV(fileLogger, dashboard, mat, 0, true, 7, false) == Constants.ObjectColours.OBJECT_RED){
                                mColour = Constants.ObjectColours.OBJECT_RED;
                                mLocation = Constants.ObjectColours.OBJECT_RED_CENTER;
                            } else if (elementColour.RoverRuckusOCV(fileLogger, dashboard, mat, 0, true, 4, false) == Constants.ObjectColours.OBJECT_RED){
                                mColour = Constants.ObjectColours.OBJECT_RED;
                                mLocation = Constants.ObjectColours.OBJECT_RED_LEFT;
                            } else if (elementColour.RoverRuckusOCV(fileLogger, dashboard, mat, 0, true, 3, false) == Constants.ObjectColours.OBJECT_RED){
                                mColour = Constants.ObjectColours.OBJECT_RED;
                                mLocation = Constants.ObjectColours.OBJECT_RED_RIGHT;
                            }
                        }
                    });
                    mintNumberColourTries = 0;
                    //if mintFindGoldLoop is 1 then we look ahead, 2 look right, 3 look left
                    mintCurrentStateFindGold = Constants.stepState.STATE_RUNNING;
                }
                break;
            case STATE_RUNNING:
                fileLogger.writeEvent(3, "Running");

                //check to see if we found
                if ((mColour == Constants.ObjectColours.OBJECT_RED) || (mColour == Constants.ObjectColours.OBJECT_NONE)) {
                    fileLogger.writeEvent(1,"Image Processed:- " + mColour.toString());

                    switch (mLocation){
                        case OBJECT_RED_LEFT:
                            autonomousStepsFile.insertSteps(3, "DRIVE", -14,mdblStepSpeed, false, false, 0, 0, 0, 0, 0, 0,  mintCurrentStep + 1);
                            autonomousStepsFile.insertSteps(3, "TANKTURN", -135,mdblStepSpeed, false, false, 0, 0, 0, 0, 0, 0,  mintCurrentStep + 1);
                            autonomousStepsFile.insertSteps(3, "DRIVE", 24,mdblStepSpeed, false, false, 0, 0, 0, 0, 0, 0,  mintCurrentStep + 1);
                            autonomousStepsFile.insertSteps(3, "TANKTURN", 45,mdblStepSpeed, false, false, 0, 0, 0, 0, 0, 0,  mintCurrentStep + 1);
                            autonomousStepsFile.insertSteps(3, "TILT", -85,mdblStepSpeed, false, false, 50, 0, 0, 0, 0, 0,  mintCurrentStep + 1);
                            break;
                        case OBJECT_RED_RIGHT:
                            autonomousStepsFile.insertSteps(3, "DRIVE", -14,mdblStepSpeed, false, false, 0, 0, 0, 0, 0, 0,  mintCurrentStep + 1);
                            autonomousStepsFile.insertSteps(3, "TANKTURN", 135,mdblStepSpeed, false, false, 0, 0, 0, 0, 0, 0,  mintCurrentStep + 1);
                            autonomousStepsFile.insertSteps(3, "DRIVE", 24,mdblStepSpeed, false, false, 0, 0, 0, 0, 0, 0,  mintCurrentStep + 1);
                            autonomousStepsFile.insertSteps(3, "TANKTURN", -45,mdblStepSpeed, false, false, 0, 0, 0, 0, 0, 0,  mintCurrentStep + 1);
                            autonomousStepsFile.insertSteps(3, "TILT", -85,mdblStepSpeed, false, false, 50, 0, 0, 0, 0, 0,  mintCurrentStep + 1);
                            break;
                        default:
                            autonomousStepsFile.insertSteps(3, "DRIVE", 24,mdblStepSpeed, true, false, 0, 0, 0, 0, 0, 0,  mintCurrentStep + 1);
                            autonomousStepsFile.insertSteps(3, "TILT", -85,mdblStepSpeed, false, false, 50, 0, 0, 0, 0, 0,  mintCurrentStep + 1);
                            autonomousStepsFile.insertSteps(3, "LIFT", -23,mdblStepSpeed, false, false, 0, 0, 0, 0, 0, 0,  mintCurrentStep + 1);
                            autonomousStepsFile.insertSteps(3, "TANKTURN", -180,mdblStepSpeed, false, false, 0, 0, 0, 0, 0, 0,  mintCurrentStep + 1);
                            autonomousStepsFile.insertSteps(3, "DRIVE", -14,mdblStepSpeed, false, false, 0, 0, 0, 0, 0, 0,  mintCurrentStep + 1);
                            break;
                    }

                    //autonomousStepsFile.insertSteps(3, "DRIVE", -14,mdblStepSpeed, true, false, 0, 0, 0, 0, 0, 0,  mintCurrentStep + 1);

                    //need to insert the next steps
                    //if we found gold we need to move forward and knock it off
                    //need to check findloopgold value to see if we have turned
//                    if (mColour == Constants.ObjectColours.OBJECT_RED) {
//                        fileLogger.writeEvent(3,"Found GOLD - Current Bearing " + getAdafruitHeading());
//                        mboolFoundGold = true;
//                        switch (mintFindGoldLoop) {
//                            case 1:
//                                autonomousStepsFile.insertSteps(3, "TANKTURN", -45,mdblStepSpeed, false, false, 0, 0, 0, 0, 0, 0,  mintCurrentStep + 1);
//                                break;
//                            case 2:
//                                autonomousStepsFile.insertSteps(3, "TANKTURN", 45,mdblStepSpeed, false, false, 0, 0, 0, 0, 0, 0,  mintCurrentStep + 1);
//                                break;
//                        }
//                        //autonomousStepsFile.insertSteps(3, "DRIVE", 18,mdblStepSpeed, false, false, 0, 0, 0, 0, 0, 0,  mintCurrentStep + 1);
//                        //autonomousStepsFile.insertSteps(3, "DRIVE", -18,mdblStepSpeed, false, true, 0, 0, 0, 0, 0, 0,  mintCurrentStep + 1);
//                        autonomousStepsFile.insertSteps(3, "TANKTURN", 180, mdblStepSpeed, false, false, 0,0,0,0,0,0,mintCurrentStep + 1);
//                    } else {
//                        fileLogger.writeEvent(3,"Found NOTHING ");
//                        switch (mintFindGoldLoop) {
//                            case 0:
//                                fileLogger.writeEvent(3,"Case 0 : Turn to see if we can find it - Current Bearing " + getAdafruitHeading());
//                                autonomousStepsFile.insertSteps(3, "FINDGOLD", 0,mdblStepSpeed, false, false, 0, 0, 0, 0, 0, 1,  mintCurrentStep + 1);
//                                autonomousStepsFile.insertSteps(3, "TANKTURN", 45,mdblStepSpeed, false, false, 0, 0, 0, 0, 0, 0,  mintCurrentStep + 1);
//                                break;
//                            case 1:
//                                fileLogger.writeEvent(3,"Case 1 : Turn other way to see if we can find it - Current Bearing " + getAdafruitHeading());
//                                autonomousStepsFile.insertSteps(3, "FINDGOLD", 0,mdblStepSpeed, false, false, 0, 0, 0, 0, 0, 2,  mintCurrentStep + 1);
//                                autonomousStepsFile.insertSteps(3, "TANKTURN", -90,mdblStepSpeed, false, false, 0, 0, 0, 0, 0, 0,  mintCurrentStep + 1);
//                                break;
//                            case 2:   //no gold found, just turn around and leave
//                                fileLogger.writeEvent(3,"Case 2 : We Found Nothing, so get back straight and finish ");
//                                autonomousStepsFile.insertSteps(3, "TANKTURN", 45,mdblStepSpeed, false, false, 0, 0, 0, 0, 0, 0,  mintCurrentStep + 1);
//                                break;
//                        }
//                    }

                    //  Transition to a new state.
                    mintCurrentStateFindGold = Constants.stepState.STATE_COMPLETE;
                    deleteParallelStep();
                }
//                else {
//                    imageCaptureOCV.takeImage(new ImageCaptureOCV.OnImageCapture() {
//                        @Override
//                        public void OnImageCaptureVoid(Mat mat) {
//                            mColour = elementColour.RoverRuckusOCV(fileLogger, dashboard, mat, 0, true, 6, false);
//                        }
//                    });
//                }
                //check timeout value
                if (mStateTime.seconds() > mdblStepTimeout) {
                    fileLogger.writeEvent(1,"Timeout:- " + mStateTime.seconds());
                    //  Transition to a new state.
                    mintCurrentStateFindGold = Constants.stepState.STATE_COMPLETE;
                    deleteParallelStep();
                }
                break;
        }
    }

    private void releaseTeamMarker(){

        fileLogger.setEventTag("releaseTeamMarker()");

        switch (mintCurrentStateTeamMarker){
            case STATE_INIT:
                fileLogger.writeEvent(2, "Initialised");
                robotArms.teamMarkerServo.setPosition(mdblTeamMarkerDrop);

                if (mdblRobotParm1 == 1){
                    robotArms.teamMarkerServo.setPosition(mdblTeamMarkerDrop);
                } else if (mdblRobotParm1 == 0){
                    robotArms.teamMarkerServo.setPosition(mdblTeamMarkerHome);
                }

                mintCurrentStateTeamMarker = Constants.stepState.STATE_RUNNING;

                break;
            case STATE_RUNNING:
                fileLogger.writeEvent(2, "Running");

                fileLogger.writeEvent(2, "Open for " + mdblRobotParm1 + "ms, now closing");
                //  Transition to a new state.
                mintCurrentStateTeamMarker = Constants.stepState.STATE_COMPLETE;
                deleteParallelStep();

                //check timeout value
                if (mStateTime.seconds() > mdblStepTimeout) {
                    robotArms.teamMarkerServo.setPosition(mdblTeamMarkerHome);
                    fileLogger.writeEvent(1,"Timeout:- " + mStateTime.seconds());
                    //  Transition to a new state.
                    mintCurrentStateTeamMarker = Constants.stepState.STATE_COMPLETE;
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

                autonomousStepsFile.insertSteps(3, "DRIVE", requiredMoveDistance,0.6, false, false, 0, 0, 0, 0, 0, 0,  mintCurrentStep + 1);
                if (intCorrectionAngle < 0) {
                    autonomousStepsFile.insertSteps(3, "LEFTTURN",  intCorrectionAngle,.5, false, false, 0, 0, 0, 0, 0, 0, mintCurrentStep + 1);

                } else if (intCorrectionAngle > 0) {
                    autonomousStepsFile.insertSteps(3, "RIGHTTURN", intCorrectionAngle,.5, false, false, 0, 0, 0, 0, 0, 0, mintCurrentStep + 1);
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
                    autonomousStepsFile.insertSteps(3, "LEFTTURN", intCorrectionAngle,.5, false, false, 0, 0, 0, 0, 0, 0, mintCurrentStep + 1);

                } else if (intCorrectionAngle > 0) {
                    autonomousStepsFile.insertSteps(3, "RIGHTTURN", intCorrectionAngle,.5, false, false, 0, 0, 0, 0, 0, 0, mintCurrentStep + 1);
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


    private Double getAdafruitPitch ()
    {
        Orientation angles;
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);

        if (imuMountCorrectionVar == 90) {
            return formatAngle(angles.angleUnit, angles.thirdAngle);
        } else {
            return formatAngle(angles.angleUnit, angles.secondAngle);
        }
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
        double angle = -z + imuStartCorrectionVar + imuMountCorrectionVar;
        if (angle < 0)
            return angle + 360;
        else if (angle > 360)
            return angle - 360;
        else
            return angle;
    }

    private boolean checkAllStatesComplete () {
        if ((mintCurrentStepDelay                   == Constants.stepState.STATE_COMPLETE) &&
            (mintCurStVuforiaTurn5291               == Constants.stepState.STATE_COMPLETE) &&
            (mintCurStVuforiaMove5291               == Constants.stepState.STATE_COMPLETE) &&
            (mintCurrentStateDrive                  == Constants.stepState.STATE_COMPLETE) &&
            (mintCurrentStateDriveHeading           == Constants.stepState.STATE_COMPLETE) &&
            (mintCurrentStatePivotTurn              == Constants.stepState.STATE_COMPLETE) &&
            (mintCurrentStateTankTurn               == Constants.stepState.STATE_COMPLETE) &&
            (mintCurrentStateEyes5291               == Constants.stepState.STATE_COMPLETE) &&
            (mintCurrentStateGyroTurnEncoder5291    == Constants.stepState.STATE_COMPLETE) &&
            (mintCurrentStateTankTurnGyroHeading    == Constants.stepState.STATE_COMPLETE) &&
            (mintCurrentStateMecanumStrafe          == Constants.stepState.STATE_COMPLETE) &&
            (mintCurrentStateMoveLift               == Constants.stepState.STATE_COMPLETE) &&
            (mintCurrentStateInTake                 == Constants.stepState.STATE_COMPLETE) &&
            (mintCurrentStateTiltMotor              == Constants.stepState.STATE_COMPLETE) &&
            (mintCurrentStateFindGold               == Constants.stepState.STATE_COMPLETE) &&
            (mintCurrentStateTeamMarker             == Constants.stepState.STATE_COMPLETE) &&
            (mintCurrentStateRadiusTurn             == Constants.stepState.STATE_COMPLETE) &&
            (mintCurrentStateWyattsGyroDrive        == Constants.stepState.STATE_COMPLETE)) {
            return true;
        }
        return false;
    }

    private void initDefaultStates() {
        mintCurrentStateStep                = Constants.stepState.STATE_INIT;
        mintCurrentStepDelay                = Constants.stepState.STATE_COMPLETE;
        mintCurStVuforiaMove5291            = Constants.stepState.STATE_COMPLETE;
        mintCurStVuforiaTurn5291            = Constants.stepState.STATE_COMPLETE;
        mintCurrentStateDrive               = Constants.stepState.STATE_COMPLETE;
        mintCurrentStateDriveHeading        = Constants.stepState.STATE_COMPLETE;
        mintCurrentStatePivotTurn           = Constants.stepState.STATE_COMPLETE;
        mintCurrentStateTankTurn            = Constants.stepState.STATE_COMPLETE;
        mintCurrentStateEyes5291            = Constants.stepState.STATE_COMPLETE;
        mintCurrentStateGyroTurnEncoder5291 = Constants.stepState.STATE_COMPLETE;
        mintCurrentStateTankTurnGyroHeading = Constants.stepState.STATE_COMPLETE;
        mintCurrentStateMecanumStrafe       = Constants.stepState.STATE_COMPLETE;
        mintCurrentStateMoveLift            = Constants.stepState.STATE_COMPLETE;
        mintCurrentStateInTake              = Constants.stepState.STATE_COMPLETE;
        mintCurrentStateTiltMotor           = Constants.stepState.STATE_COMPLETE;
        mintCurrentStateFindGold            = Constants.stepState.STATE_COMPLETE;
        mintCurrentStateTeamMarker          = Constants.stepState.STATE_COMPLETE;
        mintCurrentStateRadiusTurn          = Constants.stepState.STATE_COMPLETE;
        mintCurrentStateWyattsGyroDrive     = Constants.stepState.STATE_COMPLETE;
    }

}