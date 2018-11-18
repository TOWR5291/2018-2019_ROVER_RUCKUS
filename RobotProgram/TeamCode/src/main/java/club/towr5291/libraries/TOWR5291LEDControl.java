package club.towr5291.libraries;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import club.towr5291.functions.Constants;

/**
 * Created by Ian Haden TOWR5291 on 5/26/2018.
 * This class was created as a demonstration on how to create and use a class especially when code is re used
 *
 *
 * Modification history
 * Edited by:
 * Ian Haden 05/26/2018 -> Initial creation
 * Ian Haden 07/18/2018 -> Added comments, renamed some variables so they represent their respective functions
 */

public class TOWR5291LEDControl {

    private HardwareMap hardwareMap;
    private DigitalChannel greenLeftLedChannel;
    private DigitalChannel redLeftLedChannel;
    private DigitalChannel blueLeftLedChannel;
    private DigitalChannel greenRightLedChannel;
    private DigitalChannel redRightLedChannel;
    private DigitalChannel blueRightLedChannel;

    private double mdblLastOn;
    private double mdblLastOff;
    private double mdblLastState;
    private boolean mblnLEDON;
    private int mintCounts = 0;
    private boolean demo = false;
    private int demotimer = 10000;
    private String mAllianceColour;
    private Constants.ObjectColours mObjectColour;
    private Constants.LEDColours mLEDLeftColour;
    private Constants.LEDColours mLEDRightColour;
    private final boolean LedOn = false;
    private final boolean LedOff = true;
    private boolean greenLeftLastState = true;
    private boolean redLeftLastState = true;
    private boolean blueLeftLastState = true;
    private boolean greenRightLastState = true;
    private boolean redRightLastState = true;
    private boolean blueRightLastState = true;
    private Constants.RobotSide side = Constants.RobotSide.BOTH;
    private int flashOnTime;
    private int flashOffTime;
    private int flashOnVariableTime;
    private int flashOffVariableTime;
    private boolean reverseVariableTime;
    private int maxFlashRate = 100;
    private int minFlashRate = 1500;
    private double maxFlashRateIncrease = 1.25;
    private double minFlashRateDecrease = 0.75;

    private Constants.LEDState LEDStatusPrevious = Constants.LEDState.STATE_TEAM;

    private ElapsedTime mStateTime = new ElapsedTime();     // Time into current state, used for the timeout

    /**
     * Constructor
     * This method scales the given value from the source range to the target range.
     *
     * @param hardwareMapIn Hardware Map of the robot.
     * @param LeftGreenTag specifies the tag programmed into the module.
     * @param LeftRedTag specifies the tag programmed into the module.
     * @param LeftBlueTag specifies the tag programmed into the module.
     * @param RightGreenTag specifies the tag programmed into the module.
     * @param RightRedTag specifies the tag programmed into the module.
     * @param RightBlueTag specifies the tag programmed into the module.
     */
    public TOWR5291LEDControl(HardwareMap hardwareMapIn, String LeftGreenTag, String LeftRedTag, String LeftBlueTag, String RightGreenTag, String RightRedTag, String RightBlueTag) {
        this.hardwareMap = hardwareMapIn;
        this.greenLeftLedChannel = hardwareMap.get(DigitalChannel.class, LeftGreenTag);    //  Use generic form of device mapping
        this.redLeftLedChannel = hardwareMap.get(DigitalChannel.class, LeftRedTag);    //  Use generic form of device mapping
        this.blueLeftLedChannel = hardwareMap.get(DigitalChannel.class, LeftBlueTag);    //  Use generic form of device mapping
        this.greenRightLedChannel = hardwareMap.get(DigitalChannel.class, RightGreenTag);    //  Use generic form of device mapping
        this.redRightLedChannel = hardwareMap.get(DigitalChannel.class, RightRedTag);    //  Use generic form of device mapping
        this.blueRightLedChannel = hardwareMap.get(DigitalChannel.class, RightBlueTag);    //  Use generic form of device mapping
        initCoreVariables();
    }

    /**
     * Constructor
     * This method scales the given value from the source range to the target range.
     *
     * @param hardwareMapIn Hardware Map of the robot.
     */
    public TOWR5291LEDControl(HardwareMap hardwareMapIn) {
        this.hardwareMap = hardwareMapIn;
        this.greenLeftLedChannel = hardwareMap.get(DigitalChannel.class, robotConfig.LEDnames.leftGreen.toString());    //  Use generic form of device mapping
        this.redLeftLedChannel = hardwareMap.get(DigitalChannel.class, robotConfig.LEDnames.leftRed.toString());    //  Use generic form of device mapping
        this.blueLeftLedChannel = hardwareMap.get(DigitalChannel.class, robotConfig.LEDnames.leftBlue.toString());    //  Use generic form of device mapping
        this.greenRightLedChannel = hardwareMap.get(DigitalChannel.class, robotConfig.LEDnames.rightGreen.toString());    //  Use generic form of device mapping
        this.redRightLedChannel = hardwareMap.get(DigitalChannel.class, robotConfig.LEDnames.rightRed.toString());    //  Use generic form of device mapping
        this.blueRightLedChannel = hardwareMap.get(DigitalChannel.class, robotConfig.LEDnames.rightBlue.toString());    //  Use generic form of device mapping
        initCoreVariables();
    }

    /**
     * init the core variables needed when the class is constructed for all constructors
     */
    private void initCoreVariables() {
        this.greenLeftLedChannel.setMode(DigitalChannel.Mode.OUTPUT);
        this.redLeftLedChannel.setMode(DigitalChannel.Mode.OUTPUT);
        this.blueLeftLedChannel.setMode(DigitalChannel.Mode.OUTPUT);
        this.greenRightLedChannel.setMode(DigitalChannel.Mode.OUTPUT);
        this.redRightLedChannel.setMode(DigitalChannel.Mode.OUTPUT);
        this.blueRightLedChannel.setMode(DigitalChannel.Mode.OUTPUT);
        greenLeftLastState = this.greenLeftLedChannel.getState();
        redLeftLastState = this.redLeftLedChannel.getState();
        blueLeftLastState = this.blueLeftLedChannel.getState();
        greenRightLastState = this.greenRightLedChannel.getState();
        redRightLastState = this.redRightLedChannel.getState();
        blueRightLastState = this.blueRightLedChannel.getState();
        LedStateEnforce(LedOn, LedOff, LedOff, LedOn, LedOff, LedOff);
        this.flashOnTime = 500;
        this.flashOffTime = 500;
        this.flashOnVariableTime = 200;
        this.flashOffVariableTime = 300;
        this.reverseVariableTime = true;  //0 is speed up, 1 is slow down
        this.side = Constants.RobotSide.BOTH;
    }

    /**
     * Set the LED control DEMO Mode on or off with an optional timer
     */
    public void setLEDControlDemoMode(boolean demo) {
        setLEDControlDemoMode(demo, this.demotimer);
    }
    public void setLEDControlDemoMode(boolean demo, int timer) {
        mdblLastState = mStateTime.milliseconds();
        if (timer > 200)
            this.demotimer = timer;
        this.demo = demo;
    }

    /**
     * Get whether the LED control is in DEMO mode or in control mode
     */
    public boolean getLEDControlDemoMode() {
        return this.demo;
    }

    /**
     * Get the value of the demo timer
     */
    public int getLEDControlDemoTimer() {
        return this.demotimer;
    }

    /**
     * Set the LED Left Colour
     */
    public void setLEDLeftColour(Constants.LEDColours colour) {
        this.mLEDLeftColour = colour;
        this.side = Constants.RobotSide.LEFT;  //Left Side Only
    }

    /**
     * Set the LED Right Colour
     */
    public void setLEDRightColour(Constants.LEDColours colour) {
        this.mLEDRightColour = colour;
        this.side = Constants.RobotSide.RIGHT;  //Right Side Only
    }

    /**
     * Set the LED Colour for ALL LEDS
     */
    public void setLEDColour(Constants.LEDColours colour) {
        setLEDLeftColour(colour);
        setLEDRightColour(colour);
        this.side = Constants.RobotSide.BOTH;  //Both Side
    }

    /**
     * Set the LED Colour to the Alliance Colour
     */
    public void setLEDControlAlliance(String allianceColour) {
        this.mAllianceColour = allianceColour;
    }

    /**
     * Set the LED Colour to the Vision Detected Colour
     */
    public void setLEDControlObjectColour(Constants.ObjectColours objectColour) {
        this.mObjectColour = objectColour;
    }

    public Constants.LEDState LEDControlUpdate(Constants.LEDState LEDStatus) {

        switch (LEDStatus) {
            case STATE_UPDATE:
                LedStateLeft(this.mLEDLeftColour);
                LedStateRight(this.mLEDRightColour);
                break;
            case STATE_NULL:
                if ((!mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOff + flashOnTime))) {
                    mdblLastOn = mStateTime.milliseconds();
                    mblnLEDON = true;
                    switch (this.side) {
                        case LEFT:
                            LedStateLeft(Constants.LEDColours.LED_WHITE);
                            break;
                        case RIGHT:
                            LedStateRight(Constants.LEDColours.LED_WHITE);
                            break;
                        case BOTH:
                            LedStateLeft(Constants.LEDColours.LED_WHITE);
                            LedStateRight(Constants.LEDColours.LED_OFF);
                            break;
                    }
                } else if ((mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOn + flashOffTime))) {
                    mdblLastOff = mStateTime.milliseconds();
                    mblnLEDON = false;
                    switch (this.side) {
                        case LEFT:
                            LedStateLeft(Constants.LEDColours.LED_OFF);
                            break;
                        case RIGHT:
                            LedStateRight(Constants.LEDColours.LED_OFF);
                            break;
                        case BOTH:
                            LedStateLeft(Constants.LEDColours.LED_OFF);
                            LedStateRight(Constants.LEDColours.LED_WHITE);
                            break;
                    }
                }
                if ((this.demo) && (mStateTime.milliseconds() > (mdblLastState + this.demotimer)))
                {
                    mdblLastState = mStateTime.milliseconds();
                    mblnLEDON = false;
                    return Constants.LEDState.STATE_TEAM;
                }
                break;
            case STATE_TEAM:        //FLASH Alliance Colour
                if ((!mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOff + this.flashOnVariableTime))) {
                    mdblLastOn = mStateTime.milliseconds();

                    if (!mblnLEDON)  {
                        switch (this.mAllianceColour) {
                            case "Red":
                                LedStateBoth(Constants.LEDColours.LED_RED);
                                break;
                            case "Blue":
                                LedStateBoth(Constants.LEDColours.LED_BLUE);
                                break;
                            default:
                                LedStateBoth(Constants.LEDColours.LED_WHITE);
                                break;
                        }
                        this.flashOffVariableTime = (int)(this.flashOnVariableTime * (this.reverseVariableTime ? maxFlashRateIncrease : minFlashRateDecrease));
                    }
                    mblnLEDON = true;
                } else if ((mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOn + this.flashOffVariableTime))) {
                    mdblLastOff = mStateTime.milliseconds();
                    if (mblnLEDON) {
                        switch (this.mAllianceColour) {
                            case "Red":
                                LedStateBoth(Constants.LEDColours.LED_OFF);
                                break;
                            case "Blue":
                                LedStateBoth(Constants.LEDColours.LED_OFF);
                                break;
                            default:
                                LedStateBoth(Constants.LEDColours.LED_OFF);
                                break;
                        }
                        this.flashOnVariableTime = (int)(this.flashOnVariableTime * (this.reverseVariableTime ? maxFlashRateIncrease : minFlashRateDecrease));
                        if (this.flashOnVariableTime > minFlashRate) this.reverseVariableTime = false;
                        if (this.flashOnVariableTime < maxFlashRate) this.reverseVariableTime = true;
                    }
                    mblnLEDON = false;
                }
                if ((this.demo) && (mStateTime.milliseconds() > (mdblLastState + this.demotimer)))
                {
                    mdblLastState = mStateTime.milliseconds();
                    mblnLEDON = false;
                    return Constants.LEDState.STATE_ERROR;
                }
                break;
            case STATE_ERROR:       //Flash RED Rapidly
                if ((!mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOff + 300))) {
                    mdblLastOn = mStateTime.milliseconds();
                    mblnLEDON = true;
                    LedStateBoth(Constants.LEDColours.LED_RED);
                } else if ((mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOn + 500))) {
                    mdblLastOff = mStateTime.milliseconds();
                    mblnLEDON = false;
                    LedStateBoth(Constants.LEDColours.LED_OFF);
                }
                if ((this.demo) && (mStateTime.milliseconds() > (mdblLastState + this.demotimer)))
                {
                    mdblLastState = mStateTime.milliseconds();
                    mblnLEDON = false;
                    return Constants.LEDState.STATE_SUCCESS;
                }
                break;
            case STATE_SUCCESS:       //Flash GREEN x times Rapidly
                if ((!mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOff + 200))) {
                    mdblLastOn = mStateTime.milliseconds();
                    mblnLEDON = true;
                    LedStateBoth(Constants.LEDColours.LED_GREEN);
                } else if ((mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOn + 200))) {
                    mdblLastOff = mStateTime.milliseconds();
                    mblnLEDON = false;
                    LedStateBoth(Constants.LEDColours.LED_OFF);
                    mintCounts++;
                }
                if (this.demo) {
                    if (mStateTime.milliseconds() > (mdblLastState + this.demotimer))
                    {
                        mintCounts = 0;
                        mdblLastState = mStateTime.milliseconds();
                        mblnLEDON = false;
                        return Constants.LEDState.STATE_OBJECT;
                    }
                } else if (mintCounts >= 5) {
                    mintCounts = 0;
                    return Constants.LEDState.STATE_TEAM;
                }
                break;
            case STATE_OBJECT:       //
                if ((!mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOff + 500))) {
                    mdblLastOn = mStateTime.milliseconds();
                    mblnLEDON = true;
                    if (this.mObjectColour == Constants.ObjectColours.OBJECT_BLUE_RED) {
                        LedStateLeft(Constants.LEDColours.LED_BLUE);
                        LedStateRight(Constants.LEDColours.LED_OFF);
                    } else if (this.mObjectColour == Constants.ObjectColours.OBJECT_RED_BLUE) {
                        LedStateLeft(Constants.LEDColours.LED_RED);
                        LedStateRight(Constants.LEDColours.LED_OFF);
                    } else if (this.mObjectColour == Constants.ObjectColours.OBJECT_BLUE) {
                        LedStateLeft(Constants.LEDColours.LED_BLUE);
                        LedStateRight(Constants.LEDColours.LED_OFF);
                    } else if (this.mObjectColour == Constants.ObjectColours.OBJECT_RED) {
                        LedStateLeft(Constants.LEDColours.LED_RED);
                        LedStateRight(Constants.LEDColours.LED_OFF);
                    }
                } else if ((mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOn + 500))) {
                    mdblLastOff = mStateTime.milliseconds();
                    mblnLEDON = false;
                    if (this.mObjectColour == Constants.ObjectColours.OBJECT_BLUE_RED) {
                        LedStateLeft(Constants.LEDColours.LED_OFF);
                        LedStateRight(Constants.LEDColours.LED_RED);
                    } else if (this.mObjectColour == Constants.ObjectColours.OBJECT_RED_BLUE) {
                        LedStateLeft(Constants.LEDColours.LED_OFF);
                        LedStateRight(Constants.LEDColours.LED_BLUE);
                    } else if (this.mObjectColour == Constants.ObjectColours.OBJECT_BLUE) {
                        LedStateLeft(Constants.LEDColours.LED_OFF);
                        LedStateRight(Constants.LEDColours.LED_BLUE);
                    } else if (this.mObjectColour == Constants.ObjectColours.OBJECT_RED) {
                        LedStateLeft(Constants.LEDColours.LED_OFF);
                        LedStateRight(Constants.LEDColours.LED_RED);
                    }
                    mintCounts++;
                }
                if (this.demo) {
                    if (mStateTime.milliseconds() > (mdblLastState + this.demotimer))
                    {
                        mintCounts = 0;
                        mdblLastState = mStateTime.milliseconds();
                        mblnLEDON = false;
                        return Constants.LEDState.STATE_FINISHED;
                    }
                } else if (mintCounts >= 10) {
                    mintCounts = 0;
                    return Constants.LEDState.STATE_TEAM;
                }
                break;
            case STATE_FINISHED:      //Solid Green
                LedStateBoth(Constants.LEDColours.LED_OFF);
                if (this.demo) {
                    if (mStateTime.milliseconds() > (mdblLastState + this.demotimer/2))
                    {
                        mdblLastState = mStateTime.milliseconds();
                        mblnLEDON = false;
                        return Constants.LEDState.STATE_NULL;
                    }
                }
                break;
            case STATE_FLASHV_COLOUR:
            case STATE_FLASHC_COLOUR:        //FLASH Alliance Colour
                double rate;
                if (LEDStatus == Constants.LEDState.STATE_FLASHV_COLOUR)
                    rate = this.flashOnVariableTime;
                else
                    rate = 250;
                if ((!mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOff + rate))) {
                    mdblLastOn = mStateTime.milliseconds();
                    if (!mblnLEDON)  {
                        switch (this.side) {
                            case LEFT:
                                LedStateLeft(this.mLEDLeftColour);
                                break;
                            case RIGHT:
                                LedStateRight(this.mLEDLeftColour);
                                break;
                            default:
                                LedStateBoth(this.mLEDLeftColour);
                                break;
                        }
                        if (LEDStatus == Constants.LEDState.STATE_FLASHV_COLOUR)
                            rate = this.flashOffVariableTime = (int)(this.flashOnVariableTime * (this.reverseVariableTime ? maxFlashRateIncrease : minFlashRateDecrease));
                        else
                            rate = 250;
                    }
                    mblnLEDON = true;
                } else if ((mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOn + rate))) {
                    mdblLastOff = mStateTime.milliseconds();
                    if (mblnLEDON) {
                        LedStateBoth(Constants.LEDColours.LED_OFF);
                        this.flashOnVariableTime = (int)(this.flashOnVariableTime * (this.reverseVariableTime ? maxFlashRateIncrease : minFlashRateDecrease));
                        if (this.flashOnVariableTime > minFlashRate) this.reverseVariableTime = false;
                        if (this.flashOnVariableTime < maxFlashRate) this.reverseVariableTime = true;
                    }
                    mblnLEDON = false;
                }
                if ((this.demo) && (mStateTime.milliseconds() > (mdblLastState + this.demotimer)))
                {
                    mdblLastState = mStateTime.milliseconds();
                    mblnLEDON = false;
                    return Constants.LEDState.STATE_ERROR;
                }
                break;
        }
        return LEDStatus;
    }

    private void LedStateBoth (Constants.LEDColours Colour) {
        LedStateLeft(Colour);
        LedStateRight(Colour);
    }

    private void LedStateLeft (Constants.LEDColours Colour) {
        switch (Colour) {
            case LED_WHITE:
                if (this.greenLeftLastState)
                    this.greenLeftLedChannel.setState(LedOn);
                if (this.redLeftLastState)
                    this.redLeftLedChannel.setState(LedOn);
                if (this.blueLeftLastState)
                    this.blueLeftLedChannel.setState(LedOn);
                this.greenLeftLastState = LedOn;
                this.redLeftLastState = LedOn;
                this.blueLeftLastState = LedOn;
                break;
            case LED_RED:
                if (!this.greenLeftLastState)
                    this.greenLeftLedChannel.setState(LedOff);
                if (this.redLeftLastState)
                    this.redLeftLedChannel.setState(LedOn);
                if (!this.blueLeftLastState)
                    this.blueLeftLedChannel.setState(LedOff);
                this.greenLeftLastState = LedOff;
                this.redLeftLastState = LedOn;
                this.blueLeftLastState = LedOff;
                break;
            case LED_BLUE:
                if (!this.greenLeftLastState)
                    this.greenLeftLedChannel.setState(LedOff);
                if (!this.redLeftLastState)
                    this.redLeftLedChannel.setState(LedOff);
                if (this.blueLeftLastState)
                    this.blueLeftLedChannel.setState(LedOn);
                this.greenLeftLastState = LedOff;
                this.redLeftLastState = LedOff;
                this.blueLeftLastState = LedOn;
                break;
            case LED_CYAN:
                if (this.greenLeftLastState)
                    this.greenLeftLedChannel.setState(LedOn);
                if (!this.redLeftLastState)
                    this.redLeftLedChannel.setState(LedOff);
                if (this.blueLeftLastState)
                    this.blueLeftLedChannel.setState(LedOn);
                this.greenLeftLastState = LedOn;
                this.redLeftLastState = LedOff;
                this.blueLeftLastState = LedOn;
                break;
            case LED_GREEN:
                if (this.greenLeftLastState)
                    this.greenLeftLedChannel.setState(LedOn);
                if (!this.redLeftLastState)
                    this.redLeftLedChannel.setState(LedOff);
                if (!this.blueLeftLastState)
                    this.blueLeftLedChannel.setState(LedOff);
                this.greenLeftLastState = LedOn;
                this.redLeftLastState = LedOff;
                this.blueLeftLastState = LedOff;
                break;
            case LED_MAGENTA:
                if (!this.greenLeftLastState)
                    this.greenLeftLedChannel.setState(LedOff);
                if (this.redLeftLastState)
                    this.redLeftLedChannel.setState(LedOn);
                if (this.blueLeftLastState)
                    this.blueLeftLedChannel.setState(LedOn);
                this.greenLeftLastState = LedOff;
                this.redLeftLastState = LedOn;
                this.blueLeftLastState = LedOn;
                break;
            case LED_YELLOW:
                if (this.greenLeftLastState)
                    this.greenLeftLedChannel.setState(LedOn);
                if (this.redLeftLastState)
                    this.redLeftLedChannel.setState(LedOn);
                if (!this.blueLeftLastState)
                    this.blueLeftLedChannel.setState(LedOff);
                this.greenLeftLastState = LedOn;
                this.redLeftLastState = LedOn;
                this.blueLeftLastState = LedOff;
                break;
            case LED_OFF:
                if (!this.greenLeftLastState)
                    this.greenLeftLedChannel.setState(LedOff);
                if (!this.redLeftLastState)
                    this.redLeftLedChannel.setState(LedOff);
                if (!this.blueLeftLastState)
                    this.blueLeftLedChannel.setState(LedOff);
                this.greenLeftLastState = LedOff;
                this.redLeftLastState = LedOff;
                this.blueLeftLastState = LedOff;
                break;
            default:
                break;
        }
    }
    
    private void LedStateRight (Constants.LEDColours Colour) {
        switch (Colour) {
            case LED_WHITE:
                if (this.greenRightLastState)
                    this.greenRightLedChannel.setState(LedOn);
                if (this.redRightLastState)
                    this.redRightLedChannel.setState(LedOn);
                if (this.blueRightLastState)
                    this.blueRightLedChannel.setState(LedOn);
                this.greenRightLastState = LedOn;
                this.redRightLastState = LedOn;
                this.blueRightLastState = LedOn;
                break;
            case LED_RED:
                if (!this.greenRightLastState)
                    this.greenRightLedChannel.setState(LedOff);
                if (this.redRightLastState)
                    this.redRightLedChannel.setState(LedOn);
                if (!this.blueRightLastState)
                    this.blueRightLedChannel.setState(LedOff);
                this.greenRightLastState = LedOff;
                this.redRightLastState = LedOn;
                this.blueRightLastState = LedOff;
                break;
            case LED_BLUE:
                if (!this.greenRightLastState)
                    this.greenRightLedChannel.setState(LedOff);
                if (!this.redRightLastState)
                    this.redRightLedChannel.setState(LedOff);
                if (this.blueRightLastState)
                    this.blueRightLedChannel.setState(LedOn);
                this.greenRightLastState = LedOff;
                this.redRightLastState = LedOff;
                this.blueRightLastState = LedOn;
                break;
            case LED_CYAN:
                if (this.greenRightLastState)
                    this.greenRightLedChannel.setState(LedOn);
                if (!this.redRightLastState)
                    this.redRightLedChannel.setState(LedOff);
                if (this.blueRightLastState)
                    this.blueRightLedChannel.setState(LedOn);
                this.greenRightLedChannel.setState(LedOn);
                this.redRightLedChannel.setState(LedOff);
                this.blueRightLedChannel.setState(LedOn);
                this.greenRightLastState = LedOn;
                this.redRightLastState = LedOff;
                this.blueRightLastState = LedOn;
                break;
            case LED_GREEN:
                if (this.greenRightLastState)
                    this.greenRightLedChannel.setState(LedOn);
                if (!this.redRightLastState)
                    this.redRightLedChannel.setState(LedOff);
                if (!this.blueRightLastState)
                    this.blueRightLedChannel.setState(LedOff);
                this.greenRightLastState = LedOn;
                this.redRightLastState = LedOff;
                this.blueRightLastState = LedOff;
                break;
            case LED_MAGENTA:
                if (!this.greenRightLastState)
                    this.greenRightLedChannel.setState(LedOff);
                if (this.redRightLastState)
                    this.redRightLedChannel.setState(LedOn);
                if (this.blueRightLastState)
                    this.blueRightLedChannel.setState(LedOn);
                this.greenRightLastState = LedOff;
                this.redRightLastState = LedOn;
                this.blueRightLastState = LedOn;
                break;
            case LED_YELLOW:
                if (this.greenRightLastState)
                    this.greenRightLedChannel.setState(LedOn);
                if (this.redRightLastState)
                    this.redRightLedChannel.setState(LedOn);
                if (!this.blueRightLastState)
                    this.blueRightLedChannel.setState(LedOff);
                this.greenRightLastState = LedOn;
                this.redRightLastState = LedOn;
                this.blueRightLastState = LedOff;
                break;
            case LED_OFF:
                if (!this.greenRightLastState)
                    this.greenRightLedChannel.setState(LedOff);
                if (!this.redRightLastState)
                    this.redRightLedChannel.setState(LedOff);
                if (!this.blueRightLastState)
                    this.blueRightLedChannel.setState(LedOff);
                this.greenRightLastState = LedOff;
                this.redRightLastState = LedOff;
                this.blueRightLastState = LedOff;
                break;
            default:
                break;
        }
    }

    private void LedStateEnforce (boolean g1, boolean r1, boolean b1, boolean g2, boolean r2, boolean b2) {
        while (g1 != this.greenLeftLedChannel.getState())
            this.greenLeftLedChannel.setState(g1);
        while (r1 != this.redLeftLedChannel.getState())
            this.redLeftLedChannel.setState(r1);
        while (b1 != this.blueLeftLedChannel.getState())
            this.blueLeftLedChannel.setState(b1);
        while (g2 != this.greenRightLedChannel.getState())
            this.greenRightLedChannel.setState(g2);
        while (r2 != this.redRightLedChannel.getState())
            this.redRightLedChannel.setState(r2);
        while (b2 != this.blueRightLedChannel.getState())
            this.blueRightLedChannel.setState(b2);

        this.greenLeftLastState  = g1;
        this.redLeftLastState    = r1;
        this.blueLeftLastState   = b1;
        this.greenRightLastState = g2;
        this.redRightLastState   = r2;
        this.blueRightLastState  = b2;
    }
}
