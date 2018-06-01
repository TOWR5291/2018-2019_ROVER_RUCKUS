package club.towr5291.opmodes;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

import com.qualcomm.robotcore.util.ElapsedTime;

import club.towr5291.functions.Constants;
import club.towr5291.functions.FileLogger;


@TeleOp(name = "LED Show", group = "5291")
//@Disabled
public class BaseDriveLEDDisplay extends LinearOpMode {
    //set up TAG for logging prefic, this info will appear first in every log statemend
    private static final String TAG = "AutoDriveTeam5291";

    //The autonomous menu settings from the sharepreferences
    private SharedPreferences sharedPreferences;
    private String teamNumber;
    private String allianceColor;
    private String alliancePosition;
    private int delay;
    private String numBeacons;
    private String robotConfig;

    private ElapsedTime     runtime = new ElapsedTime();

    //set up the variables for file logger and what level of debug we will log info at
    private FileLogger fileLogger;
    private int debug = 3;

    private LEDState mint5291LEDStatus;                                                   // Flash the LED based on teh status

    private enum LEDState {
        STATE_ERROR,
        STATE_TEAM,
        STATE_MOVING,
        STATE_BEACON,
        STATE_SUCCESS,
        STATE_FINISHED
    }

    //variable for the state engine, declared here so they are accessible throughout the entire opmode with having to pass them through each function
    private ElapsedTime mStateTime = new ElapsedTime();     // Time into current state, used for the timeout

    //LED Strips
    DeviceInterfaceModule dim;                  // Device Object
    final int GREEN1_LED_CHANNEL = 0;
    final int RED1_LED_CHANNEL = 1;
    final int BLUE1_LED_CHANNEL = 2;
    final int GREEN2_LED_CHANNEL = 3;
    final int RED2_LED_CHANNEL = 4;
    final int BLUE2_LED_CHANNEL = 5;
    final boolean LedOn = false;
    final boolean LedOff = true;
    private Constants.ObjectColours mColour;
    private double mdblLastOn;
    private double mdblLastState;
    private boolean mblnLEDON;
    private int mintCounts = 0;


    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.addData("Init1     ",  "Starting!");
        telemetry.update();
        //
        // get a reference to a Modern Robotics DIM, and IO channels.
        dim = hardwareMap.get(DeviceInterfaceModule.class, "dim");   //  Use generic form of device mapping

        dim.setDigitalChannelMode(GREEN1_LED_CHANNEL, DigitalChannelController.Mode.OUTPUT); // Set the direction of each channel
        dim.setDigitalChannelMode(RED1_LED_CHANNEL, DigitalChannelController.Mode.OUTPUT); // Set the direction of each channel
        dim.setDigitalChannelMode(BLUE1_LED_CHANNEL, DigitalChannelController.Mode.OUTPUT); // Set the direction of each channel
        dim.setDigitalChannelMode(GREEN2_LED_CHANNEL, DigitalChannelController.Mode.OUTPUT); // Set the direction of each channel
        dim.setDigitalChannelMode(RED2_LED_CHANNEL, DigitalChannelController.Mode.OUTPUT); // Set the direction of each channel
        dim.setDigitalChannelMode(BLUE2_LED_CHANNEL, DigitalChannelController.Mode.OUTPUT); // Set the direction of each channel

        LedState(LedOn, LedOn, LedOn, LedOn, LedOn, LedOn);

        //load variables
        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        teamNumber = sharedPreferences.getString("club.towr5291.Autonomous.TeamNumber", "0000");
        allianceColor = sharedPreferences.getString("club.towr5291.Autonomous.Color", "Red");
        alliancePosition = sharedPreferences.getString("club.towr5291.Autonomous.Position", "Left");
        delay = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Delay", "0"));
        numBeacons = sharedPreferences.getString("club.towr5291.Autonomous.Beacons", "One");
        robotConfig = sharedPreferences.getString("club.towr5291.Autonomous.RobotConfig", "TileRunner-2x40");

        if (debug >= 1)
        {
            fileLogger = new FileLogger(runtime);
            fileLogger.open();
            fileLogger.write("Time,SysMS,Thread,Event,Desc");
            fileLogger.writeEvent(TAG, "Log Started");
            Log.d(TAG, "Log Started");
            runtime.reset();
            telemetry.addData("FileLogger: ", runtime.toString());
            telemetry.addData("FileLogger Op Out File: ", fileLogger.getFilename());
        }

        mint5291LEDStatus = LEDState.STATE_FINISHED;
        mColour = Constants.ObjectColours.OBJECT_RED_BLUE;

        if (debug >= 1)
        {
            fileLogger.writeEvent(TAG, "Init Complete");
            Log.d(TAG, "Init Complete");
        }

        //show options on the driver station phone
        telemetry.addData("Init11     ",  "Complete");
        telemetry.addData("robotConfigTeam #     ",  teamNumber);
        telemetry.addData("Alliance   ",  allianceColor);
        telemetry.addData("Start Pos  ",  alliancePosition);
        telemetry.addData("Start Del  ",  delay);
        telemetry.addData("# Beacons  ",  numBeacons);
        telemetry.addData("Robot      ",  robotConfig);
        telemetry.update();

        mdblLastState = mStateTime.milliseconds();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //the main loop.  this is where the action happens
        while (opModeIsActive())
        {
            //process LED status
            //ERROR - FLASH RED 3 TIMES
            switch (mint5291LEDStatus) {
                case STATE_TEAM:        //FLASH Alliance Colour
                    if ((!mblnLEDON)) {
                        mblnLEDON = true;
                        mdblLastOn = mStateTime.milliseconds();
                        if (allianceColor.equals("Red"))
                            LedState(LedOff, LedOn, LedOff, LedOff, LedOn, LedOff);
                        else if (allianceColor.equals("Blue"))
                            LedState(LedOff, LedOff, LedOn, LedOff, LedOff, LedOn);
                        else
                            LedState(LedOn, LedOn, LedOn, LedOn, LedOn, LedOn);
                    }
                    else if (mStateTime.milliseconds() > (mdblLastOn + 10000))
                    {
                        mdblLastState = mStateTime.milliseconds();
                        mblnLEDON = false;
                        mint5291LEDStatus = LEDState.STATE_ERROR;
                    }
                    break;
                case STATE_ERROR:       //Flash RED 3 times Rapidly
                    if ((!mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOn + 250))) {
                        mdblLastOn = mStateTime.milliseconds();
                        mblnLEDON = true;
                        LedState(LedOff, LedOn, LedOff, LedOff, LedOn, LedOff);
                    } else  if ((mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOn + 750))) {
                        mdblLastOn = mStateTime.milliseconds();
                        mblnLEDON = false;
                        LedState(LedOff, LedOff, LedOff, LedOff, LedOff, LedOff);
                    }
                    if (mStateTime.milliseconds() > (mdblLastState + 10000))
                    {
                        mdblLastState = mStateTime.milliseconds();
                        mblnLEDON = false;
                        mint5291LEDStatus = LEDState.STATE_SUCCESS;
                    }
                    break;
                case STATE_SUCCESS:       //Flash GREEN 3 times Rapidly
                    if ((!mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOn + 250))) {
                        mdblLastOn = mStateTime.milliseconds();
                        mblnLEDON = true;
                        LedState(LedOn, LedOff, LedOff, LedOn, LedOff, LedOff);
                    } else  if ((mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOn + 250))) {
                        mdblLastOn = mStateTime.milliseconds();
                        mblnLEDON = false;
                        LedState(LedOff, LedOff, LedOff, LedOff, LedOff, LedOff);
                        mintCounts ++;
                    }
                    if (mStateTime.milliseconds() > (mdblLastState + 10000))
                    {
                        mdblLastState = mStateTime.milliseconds();
                        mblnLEDON = false;
                        mint5291LEDStatus = LEDState.STATE_BEACON;
                    }
                    break;
                case STATE_BEACON:       //

                    if ((!mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOn + 500))) {
                        mdblLastOn = mStateTime.milliseconds();
                        mblnLEDON = true;
                        if (mColour == Constants.ObjectColours.OBJECT_BLUE_RED) {    //means red is to the right
                            LedState(LedOff, LedOff, LedOn, LedOff, LedOff, LedOff);
                        } else if (mColour == Constants.ObjectColours.OBJECT_RED_BLUE) {
                            LedState(LedOff, LedOn, LedOff, LedOff, LedOff, LedOff);
                        } else if (mColour == Constants.ObjectColours.OBJECT_BLUE) {
                            LedState(LedOn, LedOff, LedOff, LedOff, LedOff, LedOff);
                        } else if (mColour == Constants.ObjectColours.OBJECT_BLUE) {
                            LedState(LedOff, LedOn, LedOff, LedOff, LedOff, LedOff);
                        }
                    } else  if ((mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOn + 500))) {
                        mdblLastOn = mStateTime.milliseconds();
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
                    }
                    if (mStateTime.milliseconds() > (mdblLastState + 10000))
                    {
                        mdblLastState = mStateTime.milliseconds();
                        mblnLEDON = false;
                        mint5291LEDStatus = LEDState.STATE_FINISHED;
                    }
                    break;
                case STATE_FINISHED:      //Solid Green
                    LedState(LedOn, LedOff, LedOff, LedOn, LedOff, LedOff);
                    if (mStateTime.milliseconds() > (mdblLastState + 10000))
                    {
                        mdblLastState = mStateTime.milliseconds();
                        mblnLEDON = false;
                        mint5291LEDStatus = LEDState.STATE_TEAM;

                        if ((mColour == Constants.ObjectColours.OBJECT_BLUE_RED))
                            mColour = Constants.ObjectColours.OBJECT_RED_BLUE;
                        else
                            mColour = Constants.ObjectColours.OBJECT_BLUE_RED;

                        if (allianceColor.equals("Red"))
                            allianceColor = "Blue";
                        else
                            allianceColor = "Red";
                    }
                    break;

            }

            telemetry.update();
        }

        //opmode not active anymore

    }

    private void LedState (boolean g1, boolean r1, boolean b1, boolean g2, boolean r2, boolean b2) {

        dim.setDigitalChannelState(GREEN1_LED_CHANNEL, g1);   //turn LED ON
        dim.setDigitalChannelState(RED1_LED_CHANNEL, r1);
        dim.setDigitalChannelState(BLUE1_LED_CHANNEL, b1);
        dim.setDigitalChannelState(GREEN2_LED_CHANNEL, g2);   //turn LED ON
        dim.setDigitalChannelState(RED2_LED_CHANNEL, r2);
        dim.setDigitalChannelState(BLUE2_LED_CHANNEL, b2);

    }



}
