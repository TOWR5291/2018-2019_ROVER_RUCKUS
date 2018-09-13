package club.towr5291.opmodes;

/**
 * Created by kids on 9/30/2017 at 9:55 AM.
 */

import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import android.util.Log;
import android.widget.TextView;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

import club.towr5291.R;
import club.towr5291.functions.FileLogger;
import club.towr5291.libraries.TOWRDashBoard;
import club.towr5291.libraries.robotConfigSettings;
import club.towr5291.robotconfig.HardwareDriveMotors;

import android.app.Activity;

//import club.towr5291.robotconfig.HardwareArmMotorsRR;

@TeleOp(name="RR Mecanum Test IAN", group="RRTest")
//@Disabled
public class RRMecanumTestIan extends OpModeMasterLinear
{

    private class MyThread implements Runnable {
        @Override
        public void run() {

        }
    }



    //set up TAG for logging prefic, this info will appear first in every log statemend
    private static final String TAG = "BaseDriveTest";

    //motors
    // Declare OpMode members.
    private HardwareDriveMotors robotDrive  = new HardwareDriveMotors();   // Use a Pushbot's hardware
    private HardwareDriveMotors armDrive    = new HardwareDriveMotors();   // Use a Pushbot's hardware

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

    //set up the variables for file logger and what level of debug we will log info at
    private FileLogger fileLogger;
    private int debug = 3;

    //servos
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
    private final static double SERVOJEWELLEFT_HOME             = 147;
    private final static double SERVOJEWELRIGHT_MIN_RANGE       = 4;
    private final static double SERVOJEWELRIGHT_MAX_RANGE       = 180;
    private final static double SERVOJEWELRIGHT_HOME            = 150;
    private Servo servoGlyphGripTopLeft;
    private Servo servoGlyphGripBotLeft;
    private Servo servoGlyphGripTopRight;
    private Servo servoGlyphGripBotRight;
    private Servo servoJewelLeft;
    private Servo servoJewelRight;

    private DigitalChannel green1LedChannel;
    private DigitalChannel red1LedChannel;
    private DigitalChannel blue1LedChannel;
    private DigitalChannel green2LedChannel;
    private DigitalChannel red2LedChannel;
    private DigitalChannel blue2LedChannel;
    private DigitalChannel limitswitch1;  // Hardware Device Object
    private DigitalChannel limitswitch2;  // Hardware Device Object
    private final boolean LedOn = false;
    private boolean LedOff = true;

    private static TOWRDashBoard dashboard = null;

    public static TOWRDashBoard getDashboard()
    {
        return dashboard;
    }

    private void LedState (boolean g1, boolean r1, boolean b1, boolean g2, boolean r2, boolean b2) {
        green1LedChannel.setState(g1);
        red1LedChannel.setState(r1);
        blue1LedChannel.setState(b1);
        green2LedChannel.setState(g2);
        red2LedChannel.setState(r2);
        blue2LedChannel.setState(b2);
    }

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    @Override
    public void runOpMode() throws InterruptedException
    {

        dashboard = TOWRDashBoard.createInstance(telemetry);

        FtcRobotControllerActivity act = (FtcRobotControllerActivity)(hardwareMap.appContext);

        dashboard.setTextView((TextView)act.findViewById(R.id.textOpMode));
        dashboard.displayPrintf(0, "Starting Menu System");

        if (debug >= 1)
        {
            //create logging based on initial settings, sharepreferences will adjust levels
            fileLogger = new FileLogger(runtime, debug,true);
            fileLogger.writeEvent("START", "-------------------------------------------------------------------------");
            fileLogger.write("Time,SysMS,Thread,Event,Desc");
            fileLogger.writeEvent(TAG, "Loading sharePreferences");
            runtime.reset();
            dashboard.displayPrintf(1, "FileLogger - " + runtime.toString());
            dashboard.displayPrintf(2, "FileLogger - " + fileLogger.getFilename());
        }

        //load menu settings and setup robot and debug level
        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        teamNumber = sharedPreferences.getString("club.towr5291.Autonomous.TeamNumber", "0000");
        allianceColor = sharedPreferences.getString("club.towr5291.Autonomous.Color", "Red");
        allianceStartPosition = sharedPreferences.getString("club.towr5291.Autonomous.StartPosition", "Left");
        delay = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Delay", "0"));
        robotConfig = sharedPreferences.getString("club.towr5291.Autonomous.RobotConfig", "TileRunnerMecanum2x40");
        debug = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", "1"));

        dashboard.displayPrintf(3, "Team          - " + teamNumber);
        dashboard.displayPrintf(4, "alliance      - " + allianceColor);
        dashboard.displayPrintf(5, "StartPosition - " + allianceStartPosition);
        dashboard.displayPrintf(6, "Delay         - " + delay);
        dashboard.displayPrintf(7, "Config        - " + robotConfig);
        dashboard.displayPrintf(8, "Debug         - " + debug);

        if (debug >= 1)
        {
            //adjust debug level based on shared preferences
            fileLogger.setDebugLevel(debug);
            fileLogger.writeEvent(TAG, "Loaded sharePreferences");
            fileLogger.writeEvent(TAG, "Loading LED Settings");
        }

        // get a reference to a Modern Robotics DIM, and IO channels.
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

        if (debug >= 1)
        {
            fileLogger.writeEvent(TAG, "Loaded LED Settings");
            fileLogger.writeEvent(TAG, "Loading baseHardware");
        }

        robotDrive.init(fileLogger, hardwareMap, robotConfigSettings.robotConfigChoice.valueOf(robotConfig));
        armDrive.init(fileLogger, hardwareMap, robotConfigSettings.robotConfigChoice.valueOf(robotConfig), "lifttop", "liftbot", null, null);

        if (debug >= 1)
        {
            fileLogger.writeEvent(TAG, "Loaded baseHardware");
            fileLogger.writeEvent(TAG, "Setting setHardwareDriveRunWithoutEncoders");
        }

        robotDrive.setHardwareDriveRunWithoutEncoders();

        //config the servos
        servoGlyphGripTopLeft = hardwareMap.servo.get("griptopleft");
        servoGlyphGripBotLeft = hardwareMap.servo.get("gripbotleft");
        servoGlyphGripTopLeft.setDirection(Servo.Direction.REVERSE);
        servoGlyphGripBotLeft.setDirection(Servo.Direction.REVERSE);
        servoGlyphGripTopRight = hardwareMap.servo.get("griptopright");
        servoGlyphGripBotRight = hardwareMap.servo.get("gripbotright");
        servoJewelLeft = hardwareMap.servo.get("jewelleft");
        servoJewelRight = hardwareMap.servo.get("jewelright");

        if (debug >= 1)
        {
            fileLogger.writeEvent(TAG, "Set setHardwareDriveRunWithoutEncoders");
        }

        // get a reference to our digitalTouch object.
        limitswitch1 = hardwareMap.get(DigitalChannel.class, "limittop");
        limitswitch2 = hardwareMap.get(DigitalChannel.class, "limitbot");

        // set the digital channel to input.
        limitswitch1.setMode(DigitalChannel.Mode.INPUT);
        limitswitch2.setMode(DigitalChannel.Mode.INPUT);

        if (debug >= 1) fileLogger.writeEvent(TAG, "Set Limit Switches");

        //lock the jewel arms home
        sendServosHome(servoGlyphGripTopLeft, servoGlyphGripBotLeft, servoGlyphGripTopRight, servoGlyphGripBotRight, servoJewelLeft, servoJewelRight);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        dashboard.clearDisplay();

        while (opModeIsActive()) {

            if (limitswitch1.getState() == true) {
                dashboard.displayPrintf(3, "Limit Switch Top Is Not Pressed");
            } else {
                dashboard.displayPrintf(3, "Limit Switch Top Is Pressed");
            }
            if (limitswitch2.getState() == true) {
                dashboard.displayPrintf(4, "Limit Switch Bot Is Not Pressed");
            } else {
                dashboard.displayPrintf(4, "Limit Switch Bot Is Pressed");
            }

            if ((limitswitch1.getState() == false) && (gamepad2.right_stick_y > 0)){
                armDrive.setHardwareDriveLeft1MotorPower(0);
                armDrive.baseMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armDrive.baseMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                armDrive.setHardwareDriveLeft1MotorPower(gamepad2.right_stick_y);
            }
            if ((limitswitch2.getState() == false) && (gamepad2.left_stick_y > 0)){
                armDrive.setHardwareDriveLeft2MotorPower(0);
                armDrive.baseMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armDrive.baseMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                armDrive.setHardwareDriveLeft2MotorPower(-gamepad2.left_stick_y);
            }
            dashboard.displayPrintf(5, "Main Position " + armDrive.baseMotor2.getCurrentPosition());
            dashboard.displayPrintf(6, "Top Position " + armDrive.baseMotor1.getCurrentPosition());

            if ((gamepad2.left_trigger != 0) || (gamepad1.left_trigger != 0)){
                LedState(LedOff, LedOff, LedOn, LedOff, LedOff, LedOn);
                moveTopServos(servoGlyphGripTopLeft, servoGlyphGripTopRight, SERVOLIFTLEFTTOP_GLYPH_GRAB, SERVOLIFTRIGHTTOP_GLYPH_GRAB);
            } else if ((gamepad2.left_bumper) || (gamepad1.left_bumper)) {
                LedState(LedOff, LedOff, LedOn, LedOff, LedOff, LedOn);
                moveTopServos(servoGlyphGripTopLeft, servoGlyphGripTopRight, SERVOLIFTLEFTTOP_GLYPH_START, SERVOLIFTRIGHTTOP_GLYPH_START);
            } else {
                LedState(LedOff, LedOff, LedOff, LedOff, LedOff, LedOff);
                moveTopServos(servoGlyphGripTopLeft, servoGlyphGripTopRight, SERVOLIFTLEFTTOP_GLYPH_RELEASE, SERVOLIFTRIGHTTOP_GLYPH_RELEASE);
            }

            if ((gamepad2.right_trigger != 0) || (gamepad1.right_trigger != 0) ) {
                LedState(LedOff, LedOff, LedOn, LedOff, LedOff, LedOn);
                moveTopServos(servoGlyphGripBotLeft, servoGlyphGripBotRight, SERVOLIFTLEFTBOT_GLYPH_GRAB, SERVOLIFTRIGHTBOT_GLYPH_GRAB);
            } else if ((gamepad2.right_bumper) || (gamepad1.right_bumper)) {
                LedState(LedOff, LedOn, LedOn, LedOff, LedOn, LedOn);
                moveTopServos(servoGlyphGripBotLeft, servoGlyphGripBotRight, SERVOLIFTLEFTBOT_GLYPH_START, SERVOLIFTRIGHTBOT_GLYPH_START);
            } else {
                LedState(LedOff, LedOff, LedOff, LedOff, LedOff, LedOff);
                moveTopServos(servoGlyphGripBotLeft, servoGlyphGripBotRight, SERVOLIFTLEFTBOT_GLYPH_RELEASE, SERVOLIFTRIGHTBOT_GLYPH_RELEASE);
            }


            dblLeftMotor1 = Range.clip(-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x, -.6, .6);
            dblLeftMotor2 = Range.clip(-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x, -.6, .6);
            dblRightMotor1 = Range.clip(-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x, -.6, .6);
            dblRightMotor2 = Range.clip(-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x, -.6, .6);

            if ((dblLeftMotor1 < 0) || (dblRightMotor1 < 0)) LedState(LedOff, LedOn, LedOff, LedOff, LedOn, LedOff);
            if ((dblLeftMotor1 > 0) || (dblRightMotor1 > 0)) LedState(LedOn, LedOff, LedOff, LedOn, LedOff, LedOff);

            robotDrive.setHardwareDrivePower(dblLeftMotor1, dblLeftMotor2, dblRightMotor1, dblRightMotor2);
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

    public void strafe(handed direction, double speed) {

        switch (direction.toString()) {
            case "Left":
                dblLeftMotor1  =  speed;
                dblLeftMotor2  =  -speed;
                dblRightMotor1 =  -speed;
                dblRightMotor2 =  speed;
                robotDrive.setHardwareDrivePower(dblLeftMotor1, dblLeftMotor2, dblRightMotor1, dblRightMotor2);
                break;
            case "Right":
                dblLeftMotor1  =  -speed;
                dblLeftMotor2  =  speed;
                dblRightMotor1 =  speed;
                dblRightMotor2 =  -speed;
                robotDrive.setHardwareDrivePower(dblLeftMotor1, dblLeftMotor2, dblRightMotor1, dblRightMotor2);
                break;
        }
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

