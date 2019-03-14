package club.towr5291.Concepts;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import club.towr5291.functions.FileLogger;
import club.towr5291.libraries.LibraryMotorType;
import club.towr5291.libraries.robotConfigSettings;
import club.towr5291.robotconfig.HardwareArmMotors;
import club.towr5291.robotconfig.HardwareDriveMotors;

/**
 * Created by lztdd0 on 11/5/17.
 */


/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

@TeleOp(name = "Concept: Robot Tests", group = "Sensor")
@Disabled
public class ConceptRobotBasicTests extends LinearOpMode {
     /*
     * The REV Robotics Touch Sensor
     * is treated as a digital channel.  It is HIGH if the button is unpressed.
     * It pulls LOW if the button is pressed.
     *
     * Also, when you connect a REV Robotics Touch Sensor to the digital I/O port on the
     * Expansion Hub using a 4-wire JST cable, the second pin gets connected to the Touch Sensor.
     * The lower (first) pin stays unconnected.*
     */

    //motors
    // Declare OpMode members.
    private HardwareDriveMotors robotDrive      = new HardwareDriveMotors();   // Use a Pushbot's hardware
    private HardwareArmMotors armDrive          = new HardwareArmMotors();   // Use a Pushbot's hardware

    //mode selection stuff
    public int mode = 0;

    //all modes variables
    public double dblLeftMotor1;
    public double dblLeftMotor2;
    public double dblRightMotor1;
    public double dblRightMotor2;


    //The autonomous menu settings from the sharepreferences
    private SharedPreferences sharedPreferences;
    private String teamNumber;
    private String allianceColor;
    private String allianceStartPosition;
    private int delay;
    private String robotConfig;
    private ElapsedTime runtime = new ElapsedTime();
    private String motorType;

    //set up the variables for file logger and what level of debug we will log info at
    private FileLogger fileLogger;
    private int debug = 3;

    //servos
    // the servos are on the servo controller
    private final static double SERVOLIFTLEFTTOP_MIN_RANGE      = 0;
    private final static double SERVOLIFTLEFTTOP_MAX_RANGE      = 180;
    private final static double SERVOLIFTLEFTTOP_HOME           = 180; //90
    private final static double SERVOLIFTLEFTTOP_GLYPH_START    = 60;  //need to work this out
    private final static double SERVOLIFTLEFTTOP_GLYPH_RELEASE  = 60;
    private final static double SERVOLIFTLEFTTOP_GLYPH_GRAB     = 30;

    private final static double SERVOLIFTRIGHTTOP_MIN_RANGE     = 0;
    private final static double SERVOLIFTRIGHTTOP_MAX_RANGE     = 180;
    private final static double SERVOLIFTRIGHTTOP_HOME          = 180; //90
    private final static double SERVOLIFTRIGHTTOP_GLYPH_START   = 60;  //need to work this out
    private final static double SERVOLIFTRIGHTTOP_GLYPH_RELEASE = 60;
    private final static double SERVOLIFTRIGHTTOP_GLYPH_GRAB    = 30;

    private final static double SERVOLIFTLEFTBOT_MIN_RANGE      = 0;
    private final static double SERVOLIFTLEFTBOT_MAX_RANGE      = 180;
    private final static double SERVOLIFTLEFTBOT_HOME           = 180;  //90
    private final static double SERVOLIFTLEFTBOT_GLYPH_START    = 60;  //need to work this out
    private final static double SERVOLIFTLEFTBOT_GLYPH_RELEASE  = 60;
    private final static double SERVOLIFTLEFTBOT_GLYPH_GRAB     = 30;

    private final static double SERVOLIFTRIGHTBOT_MIN_RANGE     = 0;
    private final static double SERVOLIFTRIGHTBOT_MAX_RANGE     = 180;
    private final static double SERVOLIFTRIGHTBOT_HOME          = 180;  //90
    private final static double SERVOLIFTRIGHTBOT_GLYPH_START   = 60;  //need to work this out
    private final static double SERVOLIFTRIGHTBOT_GLYPH_RELEASE = 60;
    private final static double SERVOLIFTRIGHTBOT_GLYPH_GRAB    = 30;

    private final static double SERVOJEWELLEFT_MIN_RANGE        = 0;
    private final static double SERVOJEWELLEFT_MAX_RANGE        = 1.0;
    private final static double SERVOJEWELLEFT_HOME             = 147;
    private final static double SERVOJEWELRIGHT_MIN_RANGE       = 4;
    private final static double SERVOJEWELRIGHT_MAX_RANGE       = 1.0;
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

    private void LedState (boolean g1, boolean r1, boolean b1, boolean g2, boolean r2, boolean b2) {
        green1LedChannel.setState(g1);
        red1LedChannel.setState(r1);
        blue1LedChannel.setState(b1);
        green2LedChannel.setState(g2);
        red2LedChannel.setState(r2);
        blue2LedChannel.setState(b2);
    }

    @Override
    public void runOpMode() {

        int servoLeftpos = 0;
        int servoRightpos = 0;
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

        // get a reference to our digitalTouch object.
        limitswitch1 = hardwareMap.get(DigitalChannel.class, "limittop");
        limitswitch2 = hardwareMap.get(DigitalChannel.class, "limitbot");

        // set the digital channel to input.
        limitswitch1.setMode(DigitalChannel.Mode.INPUT);
        limitswitch2.setMode(DigitalChannel.Mode.INPUT);

        //load menu settings and setup robot and debug level
        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        teamNumber = sharedPreferences.getString("club.towr5291.Autonomous.TeamNumber", "0000");
        allianceColor = sharedPreferences.getString("club.towr5291.Autonomous.Color", "Red");
        allianceStartPosition = sharedPreferences.getString("club.towr5291.Autonomous.StartPosition", "Left");
        delay = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Delay", "0"));
        robotConfig = sharedPreferences.getString("club.towr5291.Autonomous.RobotConfig", "TileRunnerMecanum2x40");
        motorType = sharedPreferences.getString("club.towr5291.Autonomous.RobotMotorChoice", "ANDY40SPUR");
        debug = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", "1"));

        if (debug >= 1)
        {
            fileLogger = new FileLogger(runtime, debug,true);
            fileLogger.writeEvent("START", "-------------------------------------------------------------------------");
            fileLogger.write("Time,SysMS,Thread,Event,Desc");
            fileLogger.writeEvent("ConceptRobotBasicTests", "Loading sharePreferences");
            Log.d("ConceptRobotBasicTests", "Log Started");
            runtime.reset();
            telemetry.addData("FileLogger: ", runtime.toString());
            telemetry.addData("FileLogger Op Out File: ", fileLogger.getFilename());
            fileLogger.writeEvent("ConceptRobotBasicTests", "Loaded sharePreferences");
            fileLogger.writeEvent("ConceptRobotBasicTests", "Loading baseHardware");
        }

        robotDrive.init(hardwareMap, robotConfigSettings.robotConfigChoice.valueOf(robotConfig), LibraryMotorType.MotorTypes.valueOf(motorType));
        armDrive.init(hardwareMap, robotConfigSettings.robotConfigChoice.valueOf(robotConfig));

        if (debug >= 1)
        {
            fileLogger.writeEvent("ConceptRobotBasicTests", "Loaded baseHardware");
            fileLogger.writeEvent("ConceptRobotBasicTests", "Setting setHardwareDriveRunWithoutEncoders");
        }

        robotDrive.setHardwareDriveRunWithoutEncoders();

        if (debug >= 1) { fileLogger.writeEvent("ConceptRobotBasicTests", "Set setHardwareDriveRunWithoutEncoders"); }

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

        if (debug >= 1) { fileLogger.writeEvent("ConceptRobotBasicTests", "Set Servos Configs");}

        // wait for the start button to be pressed.
        waitForStart();

        // while the op mode is active, loop and read the light levels.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {

            // send the info back to driver station using telemetry function.
            // if the digital channel returns true it's HIGH and the button is unpressed.
            if (limitswitch1.getState() == true) {
                telemetry.addData("Limit Switch Top", "Is Not Pressed");
            } else {
                telemetry.addData("Limit Switch Top", "Is Pressed");
            }
            if (limitswitch2.getState() == true) {
                telemetry.addData("Limit Switch Bot", "Is Not Pressed");
            } else {
                telemetry.addData("Limit Switch Bot", "Is Pressed");
            }

            dblLeftMotor1 = Range.clip(+gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x, -1.0, 1.0);
            dblLeftMotor2 = Range.clip(-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x, -1.0, 1.0);
            dblRightMotor1 = Range.clip(-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x, -1.0, 1.0);
            dblRightMotor2 = Range.clip(+gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x, -1.0, 1.0);

            robotDrive.setHardwareDrivePower(dblLeftMotor1, dblLeftMotor2, dblRightMotor1, dblRightMotor2);

            armDrive.setHardwareArmLift1MotorPower(gamepad2.left_stick_y);
            armDrive.setHardwareArmLift2MotorPower(gamepad2.right_stick_y);

            if (gamepad1.dpad_down) {
                servoLeftpos = servoLeftpos - 1;
                if (servoLeftpos < 0) {
                    servoLeftpos = (int)SERVOLIFTLEFTTOP_HOME;
                }
            } else if (gamepad1.dpad_up) {
                servoLeftpos = servoLeftpos + 1;
                if (servoLeftpos > (int)SERVOLIFTLEFTTOP_HOME) {
                    servoLeftpos = 0;
                }
            }
            telemetry.addData("Servo Left", servoLeftpos);

            if (gamepad1.dpad_left) {
                servoRightpos = servoRightpos - 1;
                if (servoRightpos < 0) {
                    servoRightpos = (int)SERVOLIFTRIGHTTOP_HOME;
                }
            } else if (gamepad1.dpad_right) {
                servoRightpos = servoRightpos + 1;
                if (servoRightpos > (int)SERVOLIFTRIGHTTOP_HOME) {
                    servoRightpos = 0;
                }
            }
            telemetry.addData("Servo Right", servoRightpos);

            //moveServo(servoGlyphGripTopLeft, servoLeftpos, SERVOLIFTLEFTTOP_MIN_RANGE, SERVOLIFTLEFTTOP_MAX_RANGE);
            //moveServo(servoGlyphGripTopRight, servoRightpos, SERVOLIFTRIGHTTOP_MIN_RANGE, SERVOLIFTRIGHTTOP_MAX_RANGE);

            telemetry.update();
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


