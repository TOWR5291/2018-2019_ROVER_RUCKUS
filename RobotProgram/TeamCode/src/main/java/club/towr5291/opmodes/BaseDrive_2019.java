package club.towr5291.opmodes;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import android.widget.TextView;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
import club.towr5291.functions.Constants;
import club.towr5291.functions.FileLogger;
import club.towr5291.functions.TOWR5291PID;
import club.towr5291.functions.TOWR5291Tick;
import club.towr5291.functions.TOWR5291Toggle;
import club.towr5291.libraries.TOWR5291LEDControl;
import club.towr5291.libraries.TOWRDashBoard;
import club.towr5291.libraries.robotConfig;
import club.towr5291.libraries.robotConfigSettings;
import club.towr5291.robotconfig.HardwareArmMotorsRoverRuckus;
import club.towr5291.robotconfig.HardwareDriveMotors;
import club.towr5291.robotconfig.HardwareSensorsRoverRuckus;

import static club.towr5291.functions.Constants.stepState.STATE_INIT;
import static club.towr5291.functions.Constants.stepState.STATE_RUNNING;

/*
    made by Wyatt Ashley on 8/2/2018
*/
@TeleOp(name = "Base Drive 2019", group = "5291")
public class BaseDrive_2019 extends OpModeMasterLinear {

    /* Hardware Set Up */
    private HardwareDriveMotors Robot               = new HardwareDriveMotors();
    private HardwareArmMotorsRoverRuckus Arms       = new HardwareArmMotorsRoverRuckus();
    private HardwareSensorsRoverRuckus Sensors      = new HardwareSensorsRoverRuckus();
    private TOWR5291LEDControl LEDs;
    private Constants.LEDState LEDStatus            = Constants.LEDState.STATE_ERROR;

    //Settings from the sharepreferences
    private SharedPreferences sharedPreferences;

    private FileLogger fileLogger;
    final String TAG = "RR TeleOp";
    private ElapsedTime runtime                     = new ElapsedTime();
    private club.towr5291.libraries.robotConfig ourRobotConfig;

    /* TOWR TICK FUNCTION */
    private TOWR5291Tick robotPowerMultiplier       = new TOWR5291Tick();
    private TOWR5291Tick controllerAMode            = new TOWR5291Tick();
    private TOWR5291Tick controllerBMode            = new TOWR5291Tick();
    private TOWR5291Tick teamMarkerServoPosition    = new TOWR5291Tick();
    private TOWR5291Toggle leftBumperToggle         = new TOWR5291Toggle();//For Moving Lift

    private double maxDrivePower                    = 1;
    private double minDrivePower                    = 0.3333333333333;
    private double incrementDrivePower              = 0.333333333333;
    private double startDrivePower                  = 1;
    //Controller A has 4 modes of operation
    private double controllerAModes                 = 5;
    private int debug;

    private boolean blnPassedALimit = false;
    private boolean blnRunningAutoTilt = false;
    private boolean blnRunningAutoLift = false;
    private double dblCurrentLiftCounts = 0;
    private double dblStartingAngleMotorCount = 0; //This is the counts when the motor is at 0
    private double mintCurrentLiftAngleDegrees = 0;
    private static final double ANGLETOSCOREDEGREE = 45;
    private static final double LIMITSWITCH1DEGREEMEASURE = 0;
    private static final double LIMITSWITCH2DEGREEMEASURE = 0;
    private static final double LIMITSWITCH3DEGREEMEASURE = 0;
    private static final double LIMITSWITCH4DEGREEMEASURE = 0;

    private static TOWRDashBoard dashboard = null;
    public static TOWRDashBoard getDashboard()
    {
        return dashboard;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = TOWRDashBoard.createInstance(telemetry);

        FtcRobotControllerActivity act = (FtcRobotControllerActivity)(hardwareMap.appContext);

        dashboard.setTextView((TextView)act.findViewById(R.id.textOpMode));
        dashboard.displayPrintf(0, "Starting Menu System");

        ourRobotConfig = new robotConfig();

        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);

        ourRobotConfig.setAllianceColor(sharedPreferences.getString("club.towr5291.Autonomous.Color", "Red"));// Using a Function to Store The Robot Specification
        ourRobotConfig.setTeamNumber(sharedPreferences.getString("club.towr5291.Autonomous.TeamNumber", "0000"));
        ourRobotConfig.setAllianceStartPosition(sharedPreferences.getString("club.towr5291.Autonomous.Position", "Left"));
        ourRobotConfig.setDelay(Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Delay", "0")));
        ourRobotConfig.setRobotConfigBase(sharedPreferences.getString("club.towr5291.Autonomous.RobotConfigBase", "TileRunner2x40"));
        debug = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", "1"));

        //now we have loaded the config from sharedpreferences we can setup the robot
        ourRobotConfig.initConfig();
        dashboard.displayPrintf(0, "Robot Config Loaded");

        fileLogger = new FileLogger(runtime, Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", "1")), true);// initializing FileLogger
        fileLogger.open();// Opening FileLogger
        fileLogger.writeEvent(TAG, "Log Started");// First Line Add To Log

        Robot.init(fileLogger, hardwareMap, robotConfigSettings.robotConfigChoice.valueOf(ourRobotConfig.getRobotConfigBase()));// Starting robot Hardware map
        Robot.allMotorsStop();
        dashboard.displayPrintf(0, "Robot Base Loaded");

        Arms.init(hardwareMap, dashboard);
        Arms.setHardwareArmDirections();
        Arms.tiltMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arms.tiltMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fileLogger.writeEvent(2,"Arms Init");
        dashboard.displayPrintf(0, "Arms Loaded");

        fileLogger.writeEvent("Starting init for sensors ", String.valueOf(runtime));
        Sensors.init(fileLogger, hardwareMap);
        fileLogger.writeEvent("end init for sensors ", String.valueOf(runtime));
        dashboard.displayPrintf(0, "Sensors Loaded");

        fileLogger.writeEvent(2,"Sensors Init");
        LEDs = new TOWR5291LEDControl(hardwareMap);
        LEDs.setLEDControlDemoMode(true);
        LEDs.setLEDControlAlliance(ourRobotConfig.getAllianceColor());
        dashboard.displayPrintf(0, "LEDs Loaded");

        //init all the values for the counters etc
        initFunction();
        fileLogger.writeEvent(2,"Init Function Done ");

        TOWR5291Toggle toggleGamePad1X = new TOWR5291Toggle(gamepad1.x);
        toggleGamePad1X.setDebounce(500);

        // All The Specification of the robot and controller
        fileLogger.writeEvent(1,"Alliance Color", ourRobotConfig.getAllianceColor());
        fileLogger.writeEvent(1,"Alliance Start Position", ourRobotConfig.getAllianceStartPosition());
        fileLogger.writeEvent(1,"Delay", String.valueOf(ourRobotConfig.getDelay()));
        fileLogger.writeEvent(1,"Robot Base Config", ourRobotConfig.getRobotConfigBase());
        fileLogger.writeEvent(1,"Team Number", ourRobotConfig.getTeamNumber());
        fileLogger.writeEvent(1,"Robot Controller Min Tick", String.valueOf(controllerAMode.getTickMin()));
        fileLogger.writeEvent(1,"Robot Controller Max Tick", String.valueOf(controllerAMode.getTickMax()));

        fileLogger.writeEvent(1,"","Wait For Start ");

        dashboard.displayPrintf(1, "Waiting for Start");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        dashboard.clearDisplay();

        fileLogger.writeEvent("Starting Loop");

        //dashboard.clearDisplay();

        dashboard.displayPrintf(1, "Controller A Options");
        dashboard.displayPrintf(2, "--------------------");
        dashboard.displayPrintf(6, "Controller B Options");
        dashboard.displayPrintf(7, "--------------------");

        //the main loop.  this is where the action happens
        while (opModeIsActive()) {
            fileLogger.writeEvent(1,"In Main Loop");

            //change LED state every cycle
            if (!Arms.gameDance) {
                LEDStatus = LEDs.LEDControlUpdate(LEDStatus);
            }

            //adjust the robot power using the dpad on the game controller
            robotPowerMultiplier.incrementTick(gamepad1.dpad_up);
            robotPowerMultiplier.decrementTick(gamepad1.dpad_down);

            controllerAMode.incrementTick(gamepad1.start);
            controllerBMode.incrementTick(gamepad2.start);

            dashboard.displayPrintf(3, "Power Multiplier:  " + robotPowerMultiplier.getTickCurrValue());
            dashboard.displayPrintf(4, "Controller A Mode: " + (int)controllerAMode.getTickCurrValue());
            dashboard.displayPrintf(8, "Controller B Mode: " + (int)controllerBMode.getTickCurrValue());

            //drivers controller, operation based on the mode selection
            switch ((int)controllerAMode.getTickCurrValue()) {
                case 1:
                    fileLogger.writeEvent(debug,"Controller Mode", "POV");
                    dashboard.displayPrintf(5, "Controller POV:");
                    /*
                     * Case 1 is the controller type POV
                     * POV uses both joy sticks to drive
                     * The left joy stick is for forward and back
                     * The right joystick is for turning left and right
                     */
                    fileLogger.writeEvent(debug,"Controller Mode", "Robot Multiplier: -" + robotPowerMultiplier.getTickCurrValue());
                    Robot.setHardwareDriveLeftMotorPower(Range.clip(-gamepad1.left_stick_y + gamepad1.right_stick_x, -1.0, 1.0) * robotPowerMultiplier.getTickCurrValue());
                    Robot.setHardwareDriveRightMotorPower(Range.clip(-gamepad1.left_stick_y - gamepad1.right_stick_x, -1.0, 1.0) * robotPowerMultiplier.getTickCurrValue());
                    fileLogger.writeEvent(debug,"SetPowers Done");
                    break;

                case 2:
                    /*
                     * Case 2 is the controller type Tank drive
                     * Tank uses both joy sticks to drive
                     * The left joy stick is for the left wheel speed
                     * The right joy stick is for the right wheel speed
                     */
                    dashboard.displayPrintf(5, "Controller Tank");
                    fileLogger.writeEvent("Controller Mode", "Tank");
                    Robot.setHardwareDriveLeftMotorPower(-gamepad1.left_stick_y * robotPowerMultiplier.getTickCurrValue());
                    Robot.setHardwareDriveRightMotorPower(-gamepad1.right_stick_y * robotPowerMultiplier.getTickCurrValue());
                    break;
                    
                case 3:
                    dashboard.displayPrintf(5, "Mecanum Drive New 2018-19");
                    fileLogger.writeEvent(debug,"Controller Mode", "Mecanum Drive New 2018-19");
                    Robot.baseMotor1.setPower(gamepad1.left_stick_x + -gamepad1.left_stick_y + gamepad1.right_stick_x);
                    Robot.baseMotor2.setPower(-gamepad1.left_stick_x + -gamepad1.left_stick_y + gamepad1.right_stick_x);
                    Robot.baseMotor3.setPower(-gamepad1.left_stick_x + -gamepad1.left_stick_y + -gamepad1.right_stick_x);
                    Robot.baseMotor4.setPower(-gamepad1.left_stick_x + -gamepad1.left_stick_y + -gamepad1.right_stick_x);

                    break;
                //last years driving mode, prefered not to use
                case 4:
                    dashboard.displayPrintf(5, "Mecanum Drive Relic Recovery");
                    fileLogger.writeEvent(debug,"Controller Mode", "Mecanum Drive Relic Recovery");
                    Robot.baseMotor1.setPower(Range.clip(-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x, -1, 1));
                    Robot.baseMotor2.setPower(Range.clip(-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x, -1, 1));
                    Robot.baseMotor3.setPower(Range.clip(-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x, -1, 1));
                    Robot.baseMotor4.setPower(Range.clip(-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x, -1, 1));
                    break;
            } //Switch ControllerA

            //controller 2 functions, arms and intakes, depends on which mode selected

            dashboard.displayPrintf(10, "Lift Motor 1 Enc: " + Arms.getLiftMotor1Encoder());
            dashboard.displayPrintf(11, "Lift Motor 2 Enc: " + Arms.getLiftMotor2Encoder());
            dashboard.displayPrintf(12, "Tilt Motor Enc  : " + Arms.getTiltLiftEncoder());
            
            switch ((int)controllerBMode.getTickCurrValue()){
                case 1:
                    dashboard.displayPrintf(9, "Controller B Standard");
                    fileLogger.writeEvent(debug,"Controller B Mode", "Standard");

                    //Arms.tiltMotor1.setPower(-gamepad2.left_stick_y);
                    //Arms.setHardwareLiftPower(-gamepad2.right_stick_y);

                    if (gamepad2.dpad_up && !blnRunningAutoTilt){
                        Arms.tiltMotor1.setTargetPosition((int) (mintCurrentLiftAngleDegrees  - ANGLETOSCOREDEGREE));
                        fileLogger.writeEvent(4, "Setting target position to tilt motor: " + String.valueOf(mintCurrentLiftAngleDegrees  - ANGLETOSCOREDEGREE));
                        Arms.tiltMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        blnRunningAutoTilt = true;
                    }
                    if (!blnRunningAutoTilt) {
                        Arms.intakeMotor.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
                    }

                    break;
            }

        }

        /* For the auto movement on the arms to work you need to find a limit switch to know the
           current position so non of the auto movements work until a limit is passed
         */
        if (!blnPassedALimit){
            if (Sensors.getLimitSwitch1AngleMotorState()){
                //Setting the starting counts
                //First get the amount of count per degree
                //Then get the degree measure of the limit
                //Finally take the current position and subtract it by the offset set above
                dblStartingAngleMotorCount = Arms.tiltMotor1.getCurrentPosition() - (ourRobotConfig.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH1DEGREEMEASURE);
                fileLogger.writeEvent(5, "Passing Limit Switch One For The First Time -- now Auto Functions Work");
                fileLogger.writeEvent(3, "Starting offset is: " + String.valueOf(dblStartingAngleMotorCount));
                blnPassedALimit = true;
            } else if (Sensors.getLimitSwitch2AngleMotorState()){
                dblStartingAngleMotorCount = Arms.tiltMotor1.getCurrentPosition() - (ourRobotConfig.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH2DEGREEMEASURE);
                fileLogger.writeEvent(5, "Passing Limit Switch Two For The First Time -- now Auto Functions Work");
                fileLogger.writeEvent(3, "Starting offset is: " + String.valueOf(dblStartingAngleMotorCount));
                blnPassedALimit = true;
            } else if (Sensors.getLimitSwitch2AngleMotorState()){
                dblStartingAngleMotorCount = Arms.tiltMotor1.getCurrentPosition() - (ourRobotConfig.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH3DEGREEMEASURE);
                fileLogger.writeEvent(5, "Passing Limit Switch Three For The First Time -- now Auto Functions Work");
                fileLogger.writeEvent(3, "Starting offset is: " + String.valueOf(dblStartingAngleMotorCount));
                blnPassedALimit = true;
            } else if (Sensors.getLimitSwitch2AngleMotorState()){
                dblStartingAngleMotorCount = Arms.tiltMotor1.getCurrentPosition() - (ourRobotConfig.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH4DEGREEMEASURE);
                fileLogger.writeEvent(5, "Passing Limit Switch Four For The First Time -- now Auto Functions Work");
                fileLogger.writeEvent(3, "Starting offset is: " + String.valueOf(dblStartingAngleMotorCount));
                blnPassedALimit = true;
            }
        } else{
            /*Now the Auto Function are OK to work me know the current position*/
            mintCurrentLiftAngleDegrees = (Arms.tiltMotor1.getCurrentPosition() - dblStartingAngleMotorCount) * ourRobotConfig.getCOUNTS_PER_DEGREE_TILT();
            fileLogger.writeEvent("Recorded the current tilt Position: " + String.valueOf(mintCurrentLiftAngleDegrees));

            if(blnRunningAutoTilt){
                Arms.tiltMotor1.setPower(.5);

                if (!Arms.tiltMotor1.isBusy()){
                    Arms.tiltMotor1.setPower(0);
                    blnRunningAutoTilt = false;
                    fileLogger.writeEvent(3, "Tilt Motor is not busy so now tuning off auto tilt");
                }
            }
        }

        //stop the logging
        if (fileLogger != null) {
            fileLogger.writeEvent(1, "TeleOP FINISHED - FINISHED");
            fileLogger.writeEvent(1, "Stopped");
            fileLogger.close();
            fileLogger = null;
        }
    } //RunOpMode

    private void initFunction() {
        fileLogger.writeEvent(debug, "initFunctions" ,  "Started");

        fileLogger.writeEvent(debug, "initFunctions" ,  "Robot Power Multiplier Start");

        robotPowerMultiplier.setRollOver(true);
        robotPowerMultiplier.setTickMax(maxDrivePower);
        robotPowerMultiplier.setTickMin(minDrivePower);
        robotPowerMultiplier.setTickIncrement(incrementDrivePower);
        robotPowerMultiplier.setTickValue(startDrivePower);
        fileLogger.writeEvent(debug, "initFunctions" ,  "Robot Power Multiplier End");

        fileLogger.writeEvent(debug, "initFunctions" ,  "ControllerAMode Start");
        controllerAMode.setRollOver(true);
        controllerAMode.setTickMin(1);
        controllerAMode.setTickIncrement(1);
        controllerAMode.setTickMax(controllerAModes);
        controllerAMode.setTickValue(5);
        fileLogger.writeEvent(debug, "initFunctions" ,  "ControllerAMode End");

        fileLogger.writeEvent(debug, "initFunctions" ,  "ControllerBMode Start");
        controllerBMode.setRollOver(true);
        controllerBMode.setTickValue(1);
        controllerBMode.setTickMin(1);
        controllerBMode.setTickMax(1);
        controllerBMode.setTickIncrement(1);
        fileLogger.writeEvent(debug, "initFunctions" ,  "ControllerBMode End");

        fileLogger.writeEvent(debug, "initFunctions" ,  "teamMarkerServoPosition Start");
        teamMarkerServoPosition.setRollOver(true);
        teamMarkerServoPosition.setTickMax(1);
        teamMarkerServoPosition.setTickMin(0);
        teamMarkerServoPosition.setTickIncrement(.25);
        teamMarkerServoPosition.setDebounceTime(1000);
        fileLogger.writeEvent(debug, "initFunctions" ,  "teamMarkerServoPosition End");

        fileLogger.writeEvent(debug, "initFunctions" ,  "Toggle for left bumper Start");
        leftBumperToggle.setDebounce(500);
        leftBumperToggle.toggleState(false);
        fileLogger.writeEvent(debug, "initFunctions" ,  "Toggle for left bumper End");

    }
}