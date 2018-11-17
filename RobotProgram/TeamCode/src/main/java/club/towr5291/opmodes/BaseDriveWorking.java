package club.towr5291.opmodes;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import club.towr5291.robotconfig.HardwareArmMotors;
import club.towr5291.robotconfig.HardwareArmMotorsVelocityVortex;
import club.towr5291.robotconfig.HardwareDriveMotorsBaseConfig;

/**
 * Created by kids on 11/4/2016 at 7:54 PM.
 */
@TeleOp(name = "Base Drive", group = "5291")
@Disabled
public class BaseDriveWorking extends OpMode {

    // Declare OpMode members.
    private HardwareDriveMotorsBaseConfig robotDrive   = new HardwareDriveMotorsBaseConfig();   // Use a Pushbot's hardware
    private HardwareArmMotorsVelocityVortex armDrive   = new HardwareArmMotorsVelocityVortex();   // Use a Pushbot's hardware

    private double mdblLeftPow;
    private double mdblRightPow;
    private double mdblLifterPower;
    private boolean leftNegative;
    private boolean rightNegative;
    private double mdblMax;
    private boolean mblnReverse = false;
    private boolean mblnSlowDownNoSteering = false;
    private boolean mblnSlowDown = false;
    private boolean mblnSlowDownReleased = true;
    private boolean mblnIntakeOn = true;
    private boolean mblnIntakeFlip = false;
    private boolean mblnLaunch = false;
    private boolean blnRightTriggerPressed = false;
    private boolean blnGamePadA;
    private boolean blnGamePadAPressed;

    private boolean mblnReleaseArm = false;

    //Servo Pusher Strips
    private enum SERVOPusherState {
        STATE_LEFT_UP,
        STATE_RIGHT_UP,
        STATE_LEFT_RIGHT_UP,
        STATE_LEFT_DOWN,
        STATE_RIGHT_DOWN,
        STATE_LEFT_RIGHT_DOWN,
        STATE_LEFT_WINK,
        STATE_RIGHT_WINK,
        STATE_BLINK,
        STATE_DROWSY
    }
    private SERVOPusherState mint5291PUSHERStatus;
    private boolean mblnPUSHERStatus;
    private double mdblPUSHERTimer;

    //servos
    // the servos are on the servo controller
    final static double SERVOLIFTRIGHT_MIN_RANGE  = 0;
    final static double SERVOLIFTRIGHT_MAX_RANGE  = 1.0;
    final static double SERVOLIFTLEFT_MIN_RANGE  = 0;
    final static double SERVOLIFTLEFT_MAX_RANGE  = 1.0;

    final static double SERVOBEACONRIGHT_MIN_RANGE  = 0;
    final static double SERVOBEACONRIGHT_MAX_RANGE  = 1.0;
    final static double SERVOBEACONLEFT_MIN_RANGE  = 0;
    final static double SERVOBEACONLEFT_MAX_RANGE  = 1.0;

    final static int SERVOBEACONLEFT_HOME = 7;
    final static int SERVOBEACONRIGHT_HOME = 4;

    private Servo servoLifterRight;
    private Servo servoLifterLeft;
    private Servo servoBeaconLeft;
    private Servo servoBeaconRight;

    // servo controller device
    private ServoController servodevice;


    //LED Strips
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

    private LEDState mint5291LEDStatus;                                                   // Flash the LED based on the status
    DeviceInterfaceModule dim;                  // Device Object
    final int GREEN1_LED_CHANNEL = 0;
    final int RED1_LED_CHANNEL = 1;
    final int BLUE1_LED_CHANNEL = 2;
    final int GREEN2_LED_CHANNEL = 3;
    final int RED2_LED_CHANNEL = 4;
    final int BLUE2_LED_CHANNEL = 5;
    final boolean LedOn = false;
    final boolean LedOff = true;
    private double mdblLastOn;
    private double mdblLastOff;
    private boolean mblnLEDON;
    private int mintCounts = 0;
    private ElapsedTime mStateTime = new ElapsedTime();     // Time into current state, used for the timeout

    //The autonomous menu settings from the sharepreferences
    private SharedPreferences sharedPreferences;
    private String teamNumber;
    private String allianceColor;
    private String alliancePosition;
    private int delay;
    private String numBeacons;
    private String robotConfig;


    @Override
    public void init() {
        //load variables
        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        teamNumber = sharedPreferences.getString("club.towr5291.Autonomous.TeamNumber", "0000");
        allianceColor = sharedPreferences.getString("club.towr5291.Autonomous.Color", "Red");
        alliancePosition = sharedPreferences.getString("club.towr5291.Autonomous.Position", "Left");
        delay = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Delay", "0"));
        numBeacons = sharedPreferences.getString("club.towr5291.Autonomous.Beacons", "One");
        robotConfig = sharedPreferences.getString("club.towr5291.Autonomous.RobotConfig", "TileRunner-2x40");

        /*
        * Initialize the drive system variables.
        * The init() method of the hardware class does all the work here
        */
        robotDrive.init(hardwareMap);
        armDrive.init(hardwareMap);

        robotDrive.leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotDrive.leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotDrive.rightMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotDrive.rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Set all motors to zero power
        robotDrive.setDriveMotorPower(0);

        armDrive.sweeper.setPower(0);
        armDrive.flicker.setPower(0);

        servodevice = hardwareMap.servoController.get("servo");

        servoBeaconRight = hardwareMap.servo.get("servobeaconright");
        servoBeaconLeft = hardwareMap.servo.get("servobeaconleft");
        servoBeaconRight.setDirection(Servo.Direction.REVERSE);

        servoLifterRight = hardwareMap.servo.get("servoliftright");
        servoLifterLeft = hardwareMap.servo.get("servoliftleft");
        servoLifterRight.setDirection(Servo.Direction.REVERSE);
        //lock the arms up
        moveServo(servoLifterRight, 135, SERVOLIFTRIGHT_MIN_RANGE, SERVOLIFTRIGHT_MAX_RANGE);
        moveServo(servoLifterLeft, 135, SERVOLIFTLEFT_MIN_RANGE, SERVOLIFTLEFT_MAX_RANGE);

// Move the beacon pushers to home
        moveServo(servoBeaconRight, SERVOBEACONRIGHT_HOME, SERVOBEACONRIGHT_MIN_RANGE, SERVOBEACONRIGHT_MAX_RANGE);
        moveServo(servoBeaconLeft, SERVOBEACONLEFT_HOME, SERVOBEACONLEFT_MIN_RANGE, SERVOBEACONLEFT_MAX_RANGE);

        // get a reference to a Modern Robotics DIM, and IO channels.
        dim = hardwareMap.get(DeviceInterfaceModule.class, "dim");   //  Use generic form of device mapping
        dim.setDigitalChannelMode(GREEN1_LED_CHANNEL, DigitalChannelController.Mode.OUTPUT); // Set the direction of each channel
        dim.setDigitalChannelMode(RED1_LED_CHANNEL, DigitalChannelController.Mode.OUTPUT); // Set the direction of each channel
        dim.setDigitalChannelMode(BLUE1_LED_CHANNEL, DigitalChannelController.Mode.OUTPUT); // Set the direction of each channel
        dim.setDigitalChannelMode(GREEN2_LED_CHANNEL, DigitalChannelController.Mode.OUTPUT); // Set the direction of each channel
        dim.setDigitalChannelMode(RED2_LED_CHANNEL, DigitalChannelController.Mode.OUTPUT); // Set the direction of each channel
        dim.setDigitalChannelMode(BLUE2_LED_CHANNEL, DigitalChannelController.Mode.OUTPUT); // Set the direction of each channel

        LedState(LedOn, LedOn, LedOn, LedOn, LedOn, LedOn);
        mint5291LEDStatus = LEDState.STATE_TEAM;
        mint5291PUSHERStatus = SERVOPusherState.STATE_LEFT_RIGHT_DOWN;

        moveServo(servoBeaconRight, SERVOBEACONRIGHT_HOME, SERVOBEACONRIGHT_MIN_RANGE, SERVOBEACONRIGHT_MAX_RANGE);
        moveServo(servoBeaconLeft, SERVOBEACONLEFT_HOME, SERVOBEACONLEFT_MIN_RANGE, SERVOBEACONLEFT_MAX_RANGE);

        mdblMax = 1;
        mblnSlowDown = false;

    }

    /*
 * Code to run ONCE when the driver hits PLAY
 */
    @Override
    public void start() {
        mint5291PUSHERStatus = SERVOPusherState.STATE_LEFT_RIGHT_DOWN;
    }

    @Override
    public void loop() {

        //arms controls, - all gamepad 2
        if (gamepad2.right_trigger > 0) {
            mblnIntakeFlip = true;
            armDrive.sweeper.setDirection(DcMotor.Direction.REVERSE);
        }
        else
        {
            mblnIntakeFlip = false;
            armDrive.sweeper.setDirection(DcMotor.Direction.FORWARD);
        }

        if (gamepad2.right_bumper)
        {
            mblnIntakeOn = true;
            armDrive.sweeper.setPower(1);
        }
        else
        {
            mblnIntakeOn = false;
            armDrive.sweeper.setPower(0);
        }

        if (gamepad2.left_bumper) {
            armDrive.flicker.setDirection(DcMotor.Direction.FORWARD);
        } else {
            armDrive.flicker.setDirection(DcMotor.Direction.REVERSE);
        }

        if (gamepad2.left_trigger > 0)
        {
            mblnLaunch = true;
            armDrive.flicker.setPower(1);
            //set the LEDS, Reset the counter so we get the right number of flashes
            mint5291LEDStatus = LEDState.STATE_SHOOTING;
            mintCounts = 0;
        }
        else
        {
            mblnLaunch = false;
            armDrive.flicker.setPower(0);
        }

        if (gamepad2.a)
        {
            mblnReleaseArm = true;
            moveServo(servoLifterRight, 180, SERVOLIFTRIGHT_MIN_RANGE, SERVOLIFTRIGHT_MAX_RANGE);
            moveServo(servoLifterLeft, 180, SERVOLIFTLEFT_MIN_RANGE, SERVOLIFTLEFT_MAX_RANGE);
            mint5291LEDStatus = LEDState.STATE_FORKS;
        }
        else
        {
            moveServo(servoLifterRight, 135, SERVOLIFTRIGHT_MIN_RANGE, SERVOLIFTRIGHT_MAX_RANGE);
            moveServo(servoLifterLeft, 135, SERVOLIFTLEFT_MIN_RANGE, SERVOLIFTLEFT_MAX_RANGE);
        }

        //lift the extension for capping the ball
        if (mblnReleaseArm) {
            mdblLifterPower = -gamepad2.left_stick_y;
            armDrive.lifter.setPower(mdblLifterPower);
            armDrive.lifter2.setPower(mdblLifterPower);
            if (mdblLifterPower != 0)
                mint5291LEDStatus = LEDState.STATE_LIFT;

        }

        //driver controls - All gamepad1
        if (gamepad1.left_bumper)
        {
            mdblRightPow =  gamepad1.left_stick_y;
            mdblLeftPow =  gamepad1.right_stick_y;
            robotDrive.leftMotor1.setDirection(DcMotor.Direction.REVERSE);
            robotDrive.leftMotor2.setDirection(DcMotor.Direction.REVERSE);
            robotDrive.rightMotor1.setDirection(DcMotor.Direction.FORWARD);
            robotDrive.rightMotor2.setDirection(DcMotor.Direction.FORWARD);
            mblnReverse = true;
            mint5291PUSHERStatus = SERVOPusherState.STATE_LEFT_RIGHT_DOWN;
        }
        else
        {
            mdblLeftPow =  gamepad1.left_stick_y;
            mdblRightPow =  gamepad1.right_stick_y;
            robotDrive.leftMotor1.setDirection(DcMotor.Direction.FORWARD);
            robotDrive.leftMotor2.setDirection(DcMotor.Direction.FORWARD);
            robotDrive.rightMotor1.setDirection(DcMotor.Direction.REVERSE);
            robotDrive.rightMotor2.setDirection(DcMotor.Direction.REVERSE);
            mblnReverse = false;
        }
        if(gamepad1.y) {
            mdblLeftPow =  gamepad1.left_stick_y;
            mdblRightPow =  gamepad1.right_stick_y;
            robotDrive.leftMotor1.setDirection(DcMotor.Direction.FORWARD);
            robotDrive.leftMotor2.setDirection(DcMotor.Direction.FORWARD);
            robotDrive.rightMotor1.setDirection(DcMotor.Direction.REVERSE);
            robotDrive.rightMotor2.setDirection(DcMotor.Direction.REVERSE);
            mblnReverse = false;
        }
        if(!mblnReverse) {
            mdblLeftPow =  gamepad1.left_stick_y;
            mdblRightPow =  gamepad1.right_stick_y;
            robotDrive.leftMotor1.setDirection(DcMotor.Direction.FORWARD);
            robotDrive.leftMotor2.setDirection(DcMotor.Direction.FORWARD);
            robotDrive.rightMotor1.setDirection(DcMotor.Direction.REVERSE);
            robotDrive.rightMotor2.setDirection(DcMotor.Direction.REVERSE);
            mblnReverse = false;
        }

        //slow robot down to enable better control when capping the ball
        if (gamepad1.right_trigger > 0 ) {
            if (!blnRightTriggerPressed) {
                if (mdblMax >= 1) {
                    mblnSlowDown = true;
                    mdblMax = 0.25;
                    mint5291PUSHERStatus = SERVOPusherState.STATE_DROWSY;
                } else {
                    mdblMax = 1;
                    mblnSlowDown = false;
                    mint5291PUSHERStatus = SERVOPusherState.STATE_LEFT_RIGHT_UP;
                }
            }
            blnRightTriggerPressed = true;
        }
        else
        {
            //used to prevent multi button detections (debouncing)
            blnRightTriggerPressed = false;
        }

        //disable steering
        if(gamepad1.a) {
            if (!blnGamePadAPressed) {
                blnGamePadA = !blnGamePadA;
            }
            blnGamePadAPressed = true;
        } else {
            //used to prevent multi button detections (debouncing)
            blnGamePadAPressed = false;
        }

        if (blnGamePadA)
        {
            mdblMax = 0.25;
            mdblRightPow = mdblLeftPow;
        } else {
            mdblMax = 0.99;
        }

        if (mdblLeftPow < 0) {
            leftNegative = true;
        } else {
            leftNegative = false;
        }
        if (mdblRightPow < 0) {
            rightNegative = true;
        } else {
            rightNegative = false;
        }

        if (mdblLeftPow >= mdblMax && !leftNegative)
        {
            mdblLeftPow = mdblMax;
        }
        else if (mdblLeftPow <= -mdblMax && leftNegative)
        {
            mdblLeftPow = -mdblMax;
        }

        if (mdblRightPow >= mdblMax && !rightNegative)
        {
            mdblRightPow = mdblMax;
        }
        else if (mdblRightPow <= -mdblMax && rightNegative)
        {
            mdblRightPow = -mdblMax;
        }

        //Send the Pushers down
        if(gamepad1.b) {
            mint5291PUSHERStatus = SERVOPusherState.STATE_LEFT_RIGHT_DOWN;
        }

        robotDrive.setDriveRightMotorPower(mdblRightPow);
        robotDrive.setDriveLeftMotorPower(mdblLeftPow);

        telemetry.addData("Left Speed Raw", -gamepad1.left_stick_y);
        telemetry.addData("Right Speed Raw", -gamepad1.right_stick_y);
        telemetry.addData("Left Speed", -mdblLeftPow);
        telemetry.addData("Right Speed", -mdblRightPow);
        telemetry.addData("Reverse", mblnReverse);
        telemetry.addData("Slowdown", mblnSlowDown);
        telemetry.addData("Sweeper", mblnIntakeOn);
        telemetry.addData("Sweeper Rev", mblnIntakeFlip);
        telemetry.addData("Flicker", mblnLaunch);

        //LED STUFF
        //process LED status
        //ERROR - FLASH RED 3 TIMES

        switch (mint5291LEDStatus) {
            case STATE_TEAM:        //FLASH Alliance Colour
                if (allianceColor.equals("Red"))
                    LedState(LedOff, LedOn, LedOff, LedOff, LedOn, LedOff);
                else if (allianceColor.equals("Blue"))
                    LedState(LedOff, LedOff, LedOn, LedOff, LedOff, LedOn);
                else
                    LedState(LedOn, LedOn, LedOn, LedOn, LedOn, LedOn);
                break;
            case STATE_ERROR:       //Flash RED 3 times Rapidly
                if ((!mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOff + 250))) {
                    mdblLastOn = mStateTime.milliseconds();
                    mblnLEDON = true;
                    LedState(LedOff, LedOn, LedOff, LedOff, LedOn, LedOff);
                } else  if ((mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOn + 750))) {
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
                } else  if ((mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOn + 250))) {
                    mdblLastOff = mStateTime.milliseconds();
                    mblnLEDON = false;
                    LedState(LedOff, LedOff, LedOff, LedOff, LedOff, LedOff);
                    mintCounts ++;
                }
                if (mintCounts >= 5) {
                    mintCounts = 0;
                    mint5291LEDStatus = LEDState.STATE_TEAM;
                }
                break;
            case STATE_SHOOTING:       //Flash GREEN 3 times Rapidly
                if ((!mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOff + 250))) {
                    mdblLastOn = mStateTime.milliseconds();
                    mblnLEDON = true;
                    LedState(LedOn, LedOff, LedOff, LedOn, LedOff, LedOff);
                } else  if ((mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOn + 250))) {
                    mdblLastOff = mStateTime.milliseconds();
                    mblnLEDON = false;
                    LedState(LedOff, LedOff, LedOn, LedOff, LedOff, LedOn);
                    mintCounts ++;
                }
                if (mintCounts >= 5) {
                    mintCounts = 0;
                    mint5291LEDStatus = LEDState.STATE_TEAM;
                }
                break;
            case STATE_FORKS:       //Flash GREEN 3 times Rapidly
                if ((!mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOff + 250))) {
                    mdblLastOn = mStateTime.milliseconds();
                    mblnLEDON = true;
                    LedState(LedOff, LedOn, LedOn, LedOff, LedOn, LedOn);
                } else  if ((mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOn + 250))) {
                    mdblLastOff = mStateTime.milliseconds();
                    mblnLEDON = false;
                    LedState(LedOff, LedOff, LedOff, LedOff, LedOff, LedOff);
                    mintCounts ++;
                }
                if (mintCounts >= 3) {
                    mintCounts = 0;
                    mint5291LEDStatus = LEDState.STATE_TEAM;
                }
                break;
            case STATE_LIFT:
                LedState(LedOn, LedOn, LedOff, LedOn, LedOn, LedOff);
                break;
            case STATE_FINISHED:      //Solid Green
                LedState(LedOn, LedOff, LedOff, LedOn, LedOff, LedOff);
                break;
            case STATE_CELEBRATION:        //Flash red + blue for after capping
                if ((!mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOff + 250))) {
                    mdblLastOn = mStateTime.milliseconds();
                    mblnLEDON = true;
                    if (allianceColor.equals("Red"))
                        LedState(LedOff, LedOn, LedOff, LedOff, LedOn, LedOff);
                    else if (allianceColor.equals("Blue"))
                        LedState(LedOff, LedOff, LedOn, LedOff, LedOff, LedOn);
                    else
                        LedState(LedOn, LedOn, LedOn, LedOn, LedOn, LedOn);
                } else  if ((mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOn + 250))) {
                    mdblLastOff = mStateTime.milliseconds();
                    mblnLEDON = false;
                    if (allianceColor.equals("Red"))
                        LedState(LedOff, LedOn, LedOff, LedOff, LedOn, LedOff);
                    else if (allianceColor.equals("Blue"))
                        LedState(LedOff, LedOff, LedOn, LedOff, LedOff, LedOn);
                    else
                        LedState(LedOn, LedOn, LedOn, LedOn, LedOn, LedOn);
                    mintCounts ++;
                }
                if (mintCounts >= 500) {
                    mintCounts = 0;
                    mint5291LEDStatus = LEDState.STATE_TEAM;
                }
                break;
        }


        //PUSHER SERVO STUFF
        switch (mint5291PUSHERStatus) {
            case STATE_LEFT_RIGHT_DOWN:
                moveServo(servoBeaconRight, SERVOBEACONRIGHT_HOME, SERVOBEACONRIGHT_MIN_RANGE, SERVOBEACONRIGHT_MAX_RANGE);
                moveServo(servoBeaconLeft, SERVOBEACONLEFT_HOME, SERVOBEACONLEFT_MIN_RANGE, SERVOBEACONLEFT_MAX_RANGE);
                break;
            case STATE_LEFT_DOWN:
                moveServo(servoBeaconLeft, SERVOBEACONLEFT_HOME, SERVOBEACONLEFT_MIN_RANGE, SERVOBEACONLEFT_MAX_RANGE);
                break;
            case STATE_RIGHT_DOWN:
                moveServo(servoBeaconRight, SERVOBEACONRIGHT_HOME, SERVOBEACONRIGHT_MIN_RANGE, SERVOBEACONRIGHT_MAX_RANGE);
                break;
            case STATE_LEFT_UP:
                moveServo(servoBeaconLeft, SERVOBEACONLEFT_HOME + 90, SERVOBEACONLEFT_MIN_RANGE, SERVOBEACONLEFT_MAX_RANGE);
                break;
            case STATE_RIGHT_UP:
                moveServo(servoBeaconRight, SERVOBEACONRIGHT_HOME + 90, SERVOBEACONRIGHT_MIN_RANGE, SERVOBEACONRIGHT_MAX_RANGE);
                break;
            case STATE_LEFT_RIGHT_UP:
                if ((!mblnPUSHERStatus) && (mStateTime.milliseconds() > (mdblPUSHERTimer + 500))) {
                    mdblPUSHERTimer = mStateTime.milliseconds();
                    mblnPUSHERStatus = true;
                    moveServo(servoBeaconRight, SERVOBEACONRIGHT_HOME + 90, SERVOBEACONRIGHT_MIN_RANGE, SERVOBEACONRIGHT_MAX_RANGE);
                    moveServo(servoBeaconLeft, SERVOBEACONLEFT_HOME + 90, SERVOBEACONLEFT_MIN_RANGE, SERVOBEACONLEFT_MAX_RANGE);
                } else if ((mblnPUSHERStatus) && (mStateTime.milliseconds() > (mdblPUSHERTimer + 3000))) {
                    mdblPUSHERTimer = mStateTime.milliseconds();
                    mblnPUSHERStatus = false;
                    moveServo(servoBeaconRight, SERVOBEACONRIGHT_HOME, SERVOBEACONRIGHT_MIN_RANGE, SERVOBEACONRIGHT_MAX_RANGE);
                    moveServo(servoBeaconLeft, SERVOBEACONLEFT_HOME, SERVOBEACONLEFT_MIN_RANGE, SERVOBEACONLEFT_MAX_RANGE);
                }
                break;
            case STATE_LEFT_WINK:
                if ((!mblnPUSHERStatus)) {
                    mdblPUSHERTimer = mStateTime.milliseconds();
                    mblnPUSHERStatus = true;
                    moveServo(servoBeaconLeft, SERVOBEACONLEFT_HOME, SERVOBEACONLEFT_MIN_RANGE, SERVOBEACONLEFT_MAX_RANGE);
                } else if ((mblnPUSHERStatus) && (mStateTime.milliseconds() > (mdblPUSHERTimer + 750))) {
                    mdblPUSHERTimer = mStateTime.milliseconds();
                    mblnPUSHERStatus = false;
                    moveServo(servoBeaconLeft, SERVOBEACONLEFT_HOME + 90, SERVOBEACONLEFT_MIN_RANGE, SERVOBEACONLEFT_MAX_RANGE);
                    mint5291PUSHERStatus = SERVOPusherState.STATE_LEFT_RIGHT_UP;
                }
                break;
            case STATE_RIGHT_WINK:
                if ((!mblnPUSHERStatus)) {
                    mdblPUSHERTimer = mStateTime.milliseconds();
                    mblnPUSHERStatus = true;
                    moveServo(servoBeaconRight, SERVOBEACONRIGHT_HOME, SERVOBEACONRIGHT_MIN_RANGE, SERVOBEACONRIGHT_MAX_RANGE);
                } else if ((mblnPUSHERStatus) && (mStateTime.milliseconds() > (mdblPUSHERTimer + 750))) {
                    mdblPUSHERTimer = mStateTime.milliseconds();
                    mblnPUSHERStatus = false;
                    moveServo(servoBeaconRight, SERVOBEACONRIGHT_HOME + 90, SERVOBEACONRIGHT_MIN_RANGE, SERVOBEACONRIGHT_MAX_RANGE);
                    mint5291PUSHERStatus = SERVOPusherState.STATE_LEFT_RIGHT_UP;
                }
                break;
            case STATE_DROWSY:
                if ((!mblnPUSHERStatus) && (mStateTime.milliseconds() > (mdblPUSHERTimer + 500))) {
                    mdblPUSHERTimer = mStateTime.milliseconds();
                    mblnPUSHERStatus = true;
                    moveServo(servoBeaconRight, SERVOBEACONRIGHT_HOME + 45, SERVOBEACONRIGHT_MIN_RANGE, SERVOBEACONRIGHT_MAX_RANGE);
                    moveServo(servoBeaconLeft, SERVOBEACONLEFT_HOME + 45, SERVOBEACONLEFT_MIN_RANGE, SERVOBEACONLEFT_MAX_RANGE);
                } else if ((mblnPUSHERStatus) && (mStateTime.milliseconds() > (mdblPUSHERTimer + 3000))) {
                    mdblPUSHERTimer = mStateTime.milliseconds();
                    mblnPUSHERStatus = false;
                    moveServo(servoBeaconRight, SERVOBEACONRIGHT_HOME, SERVOBEACONRIGHT_MIN_RANGE, SERVOBEACONRIGHT_MAX_RANGE);
                    moveServo(servoBeaconLeft, SERVOBEACONLEFT_HOME, SERVOBEACONLEFT_MIN_RANGE, SERVOBEACONLEFT_MAX_RANGE);
                }
                break;
        }

    }

    public boolean moveServo (Servo Servo, double Position, double RangeMin, double RangeMax ) {
        boolean OKToMove = true;

        //set right position
        if ((Range.scale(Position, 0, 180, 0, 1) < RangeMin ) || (Range.scale(Position, 0, 180, 0, 1) > RangeMax )) {
            return false;
        }

        Servo.setPosition(Range.scale(Position, 0, 180, 0, 1));

        return true;
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