package club.towr5291.libraries;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import club.towr5291.functions.Constants;
import club.towr5291.functions.TOWR5291Utils;

/**
 * Created by Ian Haden TOWR5291 on 07/27/2018.
 * This class was created as a demonstration on how to create and use a class especially when code is re used
 *
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Modification history
 * Edited by:
 * Ian Haden 07/27/2018 -> Initial creation
 */

public class TOWR5291EYEControl {

    private HardwareMap hardwareMap;
    private Constants.TOWR5291Servo leftEYEServo;
    private Constants.TOWR5291Servo rightEYEServo;

    private double mdblLastOn;
    private double mdblLastOff;
    private double mdblLastState;
    private boolean mblnEYEMove;
    private int mintCounts = 0;
    private boolean demo = false;
    private int demotimer = 10000;
    private int winkOnTime;
    private int winkOffTime;
    private int winkOnVariableTime;
    private int winkOffVariableTime;
    private boolean reverseVariableTime;
    private int maxWinkRate = 5000;
    private int minwinkRate = 500;
    private double maxWinkhRateIncrease = 1.25;
    private double minwinkRateDecrease = 0.75;
    private Constants.ObjectColours mObjectColour;
    private int mintEYERightOpen = 20;
    private int mintEYELeftOpen = 20;
    private int mintEYERightHalfOpen = mintEYERightOpen/2;
    private int mintEYELeftHalfOpen = mintEYELeftOpen/2;

    private double SERVOEYERIGHT_MIN_RANGE  = 0;
    private double SERVOEYERIGHT_MAX_RANGE  = 1.0;
    private double SERVOEYELEFT_MIN_RANGE  = 0;
    private double SERVOEYELEFT_MAX_RANGE  = 1.0;
    private double SERVOEYELEFT_HOME = 7;
    private double SERVOEYERIGHT_HOME = 2;

    private Constants.LEDState LEDStatusPrevious = Constants.LEDState.STATE_TEAM;

    private ElapsedTime mStateTime = new ElapsedTime();     // Time into current state, used for the timeout


    /**
     * Constructor
     * This method scales the given value from the source range to the target range.
     *
     * @param hardwareMapIn Hardware Map of the robot.
     * @param LeftEyeTag specifies the tag programmed into the module.
     * @param RightEYETag specifies the tag programmed into the module.
     */
    public TOWR5291EYEControl(HardwareMap hardwareMapIn, String LeftEyeTag, String RightEYETag) {
        this.hardwareMap            = hardwareMapIn;
        this.rightEYEServo          = new Constants.TOWR5291Servo(hardwareMap.servo.get(RightEYETag),SERVOEYERIGHT_MIN_RANGE, SERVOEYERIGHT_MAX_RANGE, SERVOEYERIGHT_HOME);
        this.leftEYEServo           = new Constants.TOWR5291Servo(hardwareMap.servo.get(LeftEyeTag),SERVOEYERIGHT_MIN_RANGE, SERVOEYERIGHT_MAX_RANGE, SERVOEYELEFT_HOME);
        initCoreVariables();
    }

    /**
     * Constructor
     * This method scales the given value from the source range to the target range.
     *
     * @param hardwareMapIn Hardware Map of the robot.
     */
    public TOWR5291EYEControl(HardwareMap hardwareMapIn) {
        this.hardwareMap            = hardwareMapIn;
        SERVOEYERIGHT_MIN_RANGE     = robotConfig.eyeServos.rightEYE.minPos();
        SERVOEYERIGHT_MAX_RANGE     = robotConfig.eyeServos.rightEYE.maxPos();
        SERVOEYERIGHT_HOME          = robotConfig.eyeServos.rightEYE.homePos();
        SERVOEYELEFT_MIN_RANGE      = robotConfig.eyeServos.leftEYE.minPos();
        SERVOEYELEFT_MAX_RANGE      = robotConfig.eyeServos.leftEYE.maxPos();
        SERVOEYELEFT_HOME           = robotConfig.eyeServos.leftEYE.homePos();
        this.rightEYEServo          = new Constants.TOWR5291Servo(this.hardwareMap.servo.get(robotConfig.eyeServos.rightEYE.toString()),SERVOEYERIGHT_MIN_RANGE, SERVOEYERIGHT_MAX_RANGE, SERVOEYERIGHT_HOME);
        this.leftEYEServo           = new Constants.TOWR5291Servo(this.hardwareMap.servo.get(robotConfig.eyeServos.leftEYE.toString()),SERVOEYERIGHT_MIN_RANGE, SERVOEYERIGHT_MAX_RANGE, SERVOEYELEFT_HOME);
        initCoreVariables();
    }

    /**
     * init the core variables needed when the class is constructed for all constructors
     */
    private void initCoreVariables() {
        this.leftEYEServo.servo.setDirection(Servo.Direction.FORWARD);
        this.rightEYEServo.servo.setDirection(Servo.Direction.REVERSE);

        //send the servos to home position....
        TOWR5291Utils.moveServo(this.rightEYEServo);
        TOWR5291Utils.moveServo(this.leftEYEServo);

        this.winkOnTime = 1000;
        this.winkOffTime = 1000;
        this.winkOnVariableTime = 500;
        this.winkOffVariableTime = 600;
        this.reverseVariableTime = true;  //0 is speed up, 1 is slow down
        this.mintEYERightHalfOpen = this.mintEYERightOpen / 2 + (int)robotConfig.eyeServos.rightEYE.homePos();
        this.mintEYELeftHalfOpen = this.mintEYELeftOpen / 2 + (int)robotConfig.eyeServos.leftEYE.homePos();
        this.mintEYERightOpen = this.mintEYERightOpen + (int)robotConfig.eyeServos.rightEYE.homePos();
        this.mintEYELeftOpen = this.mintEYELeftOpen + (int)robotConfig.eyeServos.leftEYE.homePos();
    }

    /**
     * Set the EYE control DEMO Mode on or off with an optional timer
     */
    public void setEYEControlDemoMode(boolean demo) {
        setEYEControlDemoMode(demo, this.demotimer);
    }
    public void setEYEControlDemoMode(boolean demo, int timer) {
        mdblLastState = mStateTime.milliseconds();
        if (timer > 200)
            this.demotimer = timer;
        this.demo = demo;
    }

    /**
     * Get whether the EYE control is in DEMO mode or in control mode
     */
    public boolean getEYEControlDemoMode() {
        return this.demo;
    }
    public int getEYEControlDemoTimer() {
        return this.demotimer;
    }

    public void setEYELeftPosition(Constants.LEDColours position) {

    }

    public void setEYERightPosition(Constants.LEDColours position) {

    }
    public void setEYEPosition(Constants.LEDColours position) {
        setEYELeftPosition(position);
        setEYERightPosition(position);
    }

    public void setEYEControlObjectColour(Constants.ObjectColours objectColour) {
        this.mObjectColour = objectColour;
    }

    public Constants.EYEState EYEControlUpdate(Constants.EYEState EYEStatus) {

        switch (EYEStatus) {
            case STATE_ERROR:
                if ((!mblnEYEMove) && (mStateTime.milliseconds() > (mdblLastOff + 500))) {
                    mdblLastOn = mStateTime.milliseconds();
                    mblnEYEMove = true;
                    TOWR5291Utils.moveServoOffset(this.rightEYEServo, mintEYERightOpen);
                    TOWR5291Utils.moveServo(this.leftEYEServo);
                } else if ((mblnEYEMove) && (mStateTime.milliseconds() > (mdblLastOn + 500))) {
                    mdblLastOff = mStateTime.milliseconds();
                    mblnEYEMove = false;
                    TOWR5291Utils.moveServo(this.rightEYEServo);
                    TOWR5291Utils.moveServo(this.leftEYEServo, mintEYELeftOpen);
                }
                if ((this.demo) && (mStateTime.milliseconds() > (mdblLastState + this.demotimer)))
                {
                    mdblLastState = mStateTime.milliseconds();
                    mblnEYEMove = false;
                    TOWR5291Utils.moveServo(this.rightEYEServo);
                    TOWR5291Utils.moveServo(this.leftEYEServo);
                    return Constants.EYEState.STATE_SUCCESS;
                }
                break;
            case STATE_MOVING:
                if ((!mblnEYEMove) && (mStateTime.milliseconds() > (mdblLastOff + 1000))) {
                    mdblLastOn = mStateTime.milliseconds();
                    mblnEYEMove = true;
                    TOWR5291Utils.moveServoOffset(this.rightEYEServo, mintEYERightOpen);
                    TOWR5291Utils.moveServo(this.leftEYEServo);
                } else if ((mblnEYEMove) && (mStateTime.milliseconds() > (mdblLastOn + 200))) {
                    mdblLastOff = mStateTime.milliseconds();
                    mblnEYEMove = false;
                    TOWR5291Utils.moveServo(this.rightEYEServo);
                    TOWR5291Utils.moveServo(this.leftEYEServo, mintEYELeftOpen);
                }
                if ((this.demo) && (mStateTime.milliseconds() > (mdblLastState + this.demotimer)))
                {
                    mdblLastState = mStateTime.milliseconds();
                    mblnEYEMove = false;
                    TOWR5291Utils.moveServo(this.rightEYEServo);
                    TOWR5291Utils.moveServo(this.leftEYEServo);
                    return Constants.EYEState.STATE_SUCCESS;
                }
                break;
            case STATE_OBJECT:
                if ((!mblnEYEMove) && (mStateTime.milliseconds() > (mdblLastOff + 500))) {
                    mdblLastOn = mStateTime.milliseconds();
                    mblnEYEMove = true;
                    if (this.mObjectColour == Constants.ObjectColours.OBJECT_BLUE_RED) {

                    } else if (this.mObjectColour == Constants.ObjectColours.OBJECT_RED_BLUE) {

                    } else if (this.mObjectColour == Constants.ObjectColours.OBJECT_BLUE) {

                    } else if (this.mObjectColour == Constants.ObjectColours.OBJECT_RED) {

                    }
                } else if ((mblnEYEMove) && (mStateTime.milliseconds() > (mdblLastOn + 500))) {
                    mdblLastOff = mStateTime.milliseconds();
                    mblnEYEMove = false;
                    if (this.mObjectColour == Constants.ObjectColours.OBJECT_BLUE_RED) {

                    } else if (this.mObjectColour == Constants.ObjectColours.OBJECT_RED_BLUE) {

                    } else if (this.mObjectColour == Constants.ObjectColours.OBJECT_BLUE) {

                    } else if (this.mObjectColour == Constants.ObjectColours.OBJECT_RED) {

                    }
                    mintCounts++;
                }
                if (this.demo) {
                    if (mStateTime.milliseconds() > (mdblLastState + this.demotimer))
                    {
                        mintCounts = 0;
                        mdblLastState = mStateTime.milliseconds();
                        mblnEYEMove = false;
                        return Constants.EYEState.STATE_FINISHED;
                    }
                } else if (mintCounts >= 10) {
                    mintCounts = 0;
                    return Constants.EYEState.STATE_FINISHED;
                }
                break;
            case STATE_SUCCESS:
                if ((!mblnEYEMove) && (mStateTime.milliseconds() > (mdblLastOff + 300))) {
                    mdblLastOn = mStateTime.milliseconds();
                    mblnEYEMove = true;
                    TOWR5291Utils.moveServo(this.rightEYEServo, mintEYERightOpen);
                    TOWR5291Utils.moveServo(this.leftEYEServo, mintEYELeftOpen);
                } else if ((mblnEYEMove) && (mStateTime.milliseconds() > (mdblLastOn + 1000))) {
                    mdblLastOff = mStateTime.milliseconds();
                    mblnEYEMove = false;
                    TOWR5291Utils.moveServo(this.rightEYEServo);
                    TOWR5291Utils.moveServo(this.leftEYEServo);
                    mintCounts++;
                }
                if (this.demo) {
                    if (mStateTime.milliseconds() > (mdblLastState + this.demotimer))
                    {
                        mintCounts = 0;
                        mdblLastState = mStateTime.milliseconds();
                        mblnEYEMove = false;
                        return Constants.EYEState.STATE_BLINK;
                    }
                } else if (mintCounts >= 5) {
                    mintCounts = 0;
                    return Constants.EYEState.STATE_BLINK;
                }
                break;
            case STATE_WINKLEFT:
                if ((!mblnEYEMove) && (mStateTime.milliseconds() > (mdblLastOff + winkOnTime))) {
                    mdblLastOn = mStateTime.milliseconds();
                    mblnEYEMove = true;
                    TOWR5291Utils.moveServo(this.rightEYEServo);
                    TOWR5291Utils.moveServo(this.leftEYEServo, mintEYELeftOpen);
                } else if ((mblnEYEMove) && (mStateTime.milliseconds() > (mdblLastOn + winkOffTime))) {
                    mdblLastOff = mStateTime.milliseconds();
                    mblnEYEMove = false;
                    TOWR5291Utils.moveServo(this.rightEYEServo);
                    TOWR5291Utils.moveServo(this.leftEYEServo);
                }
                if ((this.demo) && (mStateTime.milliseconds() > (mdblLastState + this.demotimer)))
                {
                    mdblLastState = mStateTime.milliseconds();
                    mblnEYEMove = false;
                    TOWR5291Utils.moveServo(this.rightEYEServo);
                    TOWR5291Utils.moveServo(this.leftEYEServo);
                    return Constants.EYEState.STATE_ERROR;
                }
                break;
            case STATE_WINKRIGHT:
                if ((!mblnEYEMove) && (mStateTime.milliseconds() > (mdblLastOff + winkOnTime))) {
                    mdblLastOn = mStateTime.milliseconds();
                    mblnEYEMove = true;
                    TOWR5291Utils.moveServo(this.rightEYEServo, mintEYERightOpen);
                    TOWR5291Utils.moveServo(this.leftEYEServo);
                } else if ((mblnEYEMove) && (mStateTime.milliseconds() > (mdblLastOn + winkOffTime))) {
                    mdblLastOff = mStateTime.milliseconds();
                    mblnEYEMove = false;
                    TOWR5291Utils.moveServo(this.rightEYEServo);
                    TOWR5291Utils.moveServo(this.leftEYEServo);
                }
                if ((this.demo) && (mStateTime.milliseconds() > (mdblLastState + this.demotimer)))
                {
                    mdblLastState = mStateTime.milliseconds();
                    mblnEYEMove = false;
                    TOWR5291Utils.moveServo(this.rightEYEServo);
                    TOWR5291Utils.moveServo(this.leftEYEServo);
                    return Constants.EYEState.STATE_ERROR;
                }
                break;
            case STATE_BLINK:
                if ((!mblnEYEMove) && (mStateTime.milliseconds() > (mdblLastOff + winkOnTime))) {
                    mdblLastOn = mStateTime.milliseconds();
                    mblnEYEMove = true;
                    TOWR5291Utils.moveServo(this.rightEYEServo, mintEYERightOpen);
                    TOWR5291Utils.moveServo(this.leftEYEServo, mintEYELeftOpen);
                } else if ((mblnEYEMove) && (mStateTime.milliseconds() > (mdblLastOn + winkOffTime))) {
                    mdblLastOff = mStateTime.milliseconds();
                    mblnEYEMove = false;
                    TOWR5291Utils.moveServo(this.rightEYEServo);
                    TOWR5291Utils.moveServo(this.leftEYEServo);
                }
                if ((this.demo) && (mStateTime.milliseconds() > (mdblLastState + this.demotimer)))
                {
                    mdblLastState = mStateTime.milliseconds();
                    mblnEYEMove = false;
                    TOWR5291Utils.moveServo(this.rightEYEServo);
                    TOWR5291Utils.moveServo(this.leftEYEServo);
                    return Constants.EYEState.STATE_ERROR;
                }
                break;
            case STATE_OPEN:
                TOWR5291Utils.moveServo(this.rightEYEServo, mintEYERightOpen);
                TOWR5291Utils.moveServo(this.leftEYEServo, mintEYELeftOpen);
                if (this.demo) {
                    if (mStateTime.milliseconds() > (mdblLastState + this.demotimer))
                    {
                        mintCounts = 0;
                        mdblLastState = mStateTime.milliseconds();
                        mblnEYEMove = false;
                        return Constants.EYEState.STATE_BLINK;
                    }
                } else if (mintCounts >= 5) {
                    mintCounts = 0;
                    return Constants.EYEState.STATE_BLINK;
                }
                break;
            case STATE_ANGRY:
                TOWR5291Utils.moveServo(this.rightEYEServo, mintEYERightHalfOpen);
                TOWR5291Utils.moveServo(this.leftEYEServo, mintEYELeftHalfOpen);
                if (this.demo) {
                    if (mStateTime.milliseconds() > (mdblLastState + this.demotimer))
                    {
                        mintCounts = 0;
                        mdblLastState = mStateTime.milliseconds();
                        mblnEYEMove = false;
                        return Constants.EYEState.STATE_BLINK;
                    }
                } else if (mintCounts >= 5) {
                    mintCounts = 0;
                    return Constants.EYEState.STATE_BLINK;
                }
                break;
            case STATE_CLOSED:
                TOWR5291Utils.moveServo(this.rightEYEServo);
                TOWR5291Utils.moveServo(this.leftEYEServo);
                if (this.demo) {
                    if (mStateTime.milliseconds() > (mdblLastState + this.demotimer))
                    {
                        mintCounts = 0;
                        mdblLastState = mStateTime.milliseconds();
                        mblnEYEMove = false;
                        return Constants.EYEState.STATE_BLINK;
                    }
                } else if (mintCounts >= 5) {
                    mintCounts = 0;
                    return Constants.EYEState.STATE_BLINK;
                }
                break;
            case STATE_FINISHED:
                if ((!mblnEYEMove) && (mStateTime.milliseconds() > (mdblLastOff + winkOnTime))) {
                    mdblLastOn = mStateTime.milliseconds();
                    mblnEYEMove = true;
                    TOWR5291Utils.moveServo(this.rightEYEServo, mintEYERightOpen);if ((!mblnEYEMove) && (mStateTime.milliseconds() > (mdblLastOff + winkOnTime))) {
                    mdblLastOn = mStateTime.milliseconds();
                    mblnEYEMove = true;
                    TOWR5291Utils.moveServo(this.rightEYEServo, mintEYERightOpen);
                    TOWR5291Utils.moveServo(this.leftEYEServo, mintEYELeftOpen);
                } else if ((mblnEYEMove) && (mStateTime.milliseconds() > (mdblLastOn + winkOffTime))) {
                    mdblLastOff = mStateTime.milliseconds();
                    mblnEYEMove = false;
                    TOWR5291Utils.moveServo(this.rightEYEServo);
                    TOWR5291Utils.moveServo(this.leftEYEServo);
                }
                    TOWR5291Utils.moveServo(this.leftEYEServo, mintEYELeftOpen);
                } else if ((mblnEYEMove) && (mStateTime.milliseconds() > (mdblLastOn + winkOffTime))) {
                    mdblLastOff = mStateTime.milliseconds();
                    mblnEYEMove = false;
                    TOWR5291Utils.moveServo(this.rightEYEServo);
                    TOWR5291Utils.moveServo(this.leftEYEServo);
                }
                if (this.demo) {
                    if (mStateTime.milliseconds() > (mdblLastState + this.demotimer/2))
                    {
                        mdblLastState = mStateTime.milliseconds();
                        mblnEYEMove = false;
                        return Constants.EYEState.STATE_BLINK;
                    }
                }
                break;
        }
        return EYEStatus;
    }

}
