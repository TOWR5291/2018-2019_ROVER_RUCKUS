package club.towr5291.opmodes;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import android.widget.TextView;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import club.towr5291.R;
import club.towr5291.functions.Constants;
import club.towr5291.functions.FileLogger;
import club.towr5291.functions.TOWR5291Tick;
import club.towr5291.functions.TOWR5291Toggle;
import club.towr5291.libraries.LibraryMotorType;
import club.towr5291.libraries.TOWR5291LEDControl;
import club.towr5291.libraries.TOWRDashBoard;
import club.towr5291.libraries.robotConfig;
import club.towr5291.libraries.robotConfigSettings;
import club.towr5291.robotconfig.HardwareArmMotorsRoverRuckus;
import club.towr5291.robotconfig.HardwareDriveMotors;
import club.towr5291.robotconfig.HardwareSensorsRoverRuckus;

import static club.towr5291.functions.Constants.stepState.STATE_COMPLETE;

/*
    made by Wyatt Ashley on 8/2/2018
*/
@TeleOp(name = "Base Drive 2019", group = "5291")
public class BaseDrive_2019 extends OpModeMasterLinear {
    private Constants.stepState stepState = Constants.stepState.STATE_COMPLETE;

    private boolean haveACurrentValue = false;
    private double currentTiltEncoderCountMotor1 = 0;
    private double currentTiltEncoderCountMotor2 = 0;
    private double startingAngleMotor1Count;
    private double startingAngleMotor2Count;
    private double targetDegreeMoveToPosition = 0;
    private double HOLDINGTILTMOTORPOWER = .2;
    private boolean hold = false;

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

    private static final double LIMITSWITCH1DEGREEMEASURE = 0;
    private static final double LIMITSWITCH2DEGREEMEASURE = 0;
    private static final double LIMITSWITCH3DEGREEMEASURE = 0;
    private static final double LIMITSWITCH4DEGREEMEASURE = 0;

    private double maxDrivePower                    = 1;
    private double minDrivePower                    = 0.33;
    private double incrementDrivePower              = 0.33;
    private double startDrivePower                  = 1;
    //Controller A has 4 modes of operation
    private double controllerAModes                 = 5;
    private int debug;

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
        ourRobotConfig.setRobotMotorType(sharedPreferences.getString("club.towr5291.Autonomous.RobotMotorChoice", "REV20ORBIT"));
        ourRobotConfig.setRobotConfigBase(sharedPreferences.getString("club.towr5291.Autonomous.RobotConfigBase", "TileRunner2x40"));
        debug = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", "1"));

        //now we have loaded the config from sharedpreferences we can setup the robot
        ourRobotConfig.initConfig();
        dashboard.displayPrintf(0, "Robot Config Loaded");

        fileLogger = new FileLogger(runtime, Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", "1")), true);// initializing FileLogger
        fileLogger.open();// Opening FileLogger
        fileLogger.writeEvent(TAG, "Log Started");// First Line Add To Log

        // All The Specification of the robot and controller
        fileLogger.writeEvent(1,"Alliance Color", ourRobotConfig.getAllianceColor());
        fileLogger.writeEvent(1,"Alliance Start Position", ourRobotConfig.getAllianceStartPosition());
        fileLogger.writeEvent(1,"Delay", String.valueOf(ourRobotConfig.getDelay()));
        fileLogger.writeEvent(1,"Robot Base Config", ourRobotConfig.getRobotConfigBase());
        fileLogger.writeEvent(1, "Robot Motor Type", ourRobotConfig.getRobotMotorType());
        fileLogger.writeEvent(1,"Team Number", ourRobotConfig.getTeamNumber());

        Robot.init(fileLogger, hardwareMap, robotConfigSettings.robotConfigChoice.valueOf(ourRobotConfig.getRobotConfigBase()), LibraryMotorType.MotorTypes.valueOf(ourRobotConfig.getRobotMotorType()));// Starting robot Hardware map
        Arms.init(hardwareMap, dashboard);
        Sensors.init(hardwareMap);
        dashboard.displayPrintf(0, "Robot Base Loaded");

        Arms.allMotorsStop();
        Robot.allMotorsStop();

        fileLogger.writeEvent(2,"LED Init");
        LEDs = new TOWR5291LEDControl(hardwareMap);
        LEDs.setLEDControlDemoMode(true);
        LEDs.setLEDControlAlliance(ourRobotConfig.getAllianceColor());
        dashboard.displayPrintf(0, "LEDs Loaded");

        //init all the values for the counters etc
        fileLogger.writeEvent(2, "INIT Function Started -- " + String.valueOf(this.runtime));
        initFunction();
        fileLogger.writeEvent(2,"Init Function Done -- " + String.valueOf(this.runtime));

        TOWR5291Toggle toggleGamePad1X = new TOWR5291Toggle(gamepad1.x);
        toggleGamePad1X.setDebounce(500);

        Arms.teamMarkerServo.setPosition(0);

        fileLogger.writeEvent(1,"","Wait For Start ");

        dashboard.displayPrintf(1, "Waiting for Start");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        dashboard.clearDisplay();

        fileLogger.writeEvent("Starting Loop");

        //dashboard.clearDisplay();

        dashboard.displayPrintf(3, "Controller A Options");
        dashboard.displayPrintf(4, "--------------------");
        dashboard.displayPrintf(8, "Controller B Options");
        dashboard.displayPrintf(9, "--------------------");

        //the main loop.  this is where the action happens
        while (opModeIsActive()) {
            fileLogger.writeEvent(1,"In Main Loop");

            //change LED state every cycle
            LEDStatus = LEDs.LEDControlUpdate(LEDStatus);

            //adjust the robot power using the dpad on the game controller
            robotPowerMultiplier.incrementTick(gamepad1.dpad_up);
            robotPowerMultiplier.decrementTick(gamepad1.dpad_down);

            //controllerAMode.incrementTick(gamepad1.start);
            //controllerBMode.incrementTick(gamepad2.start);

            dashboard.displayPrintf(6, "Power Multiplier:  " + robotPowerMultiplier.getTickCurrValue());
            dashboard.displayPrintf(7, "Controller A Mode: " + (int)controllerAMode.getTickCurrValue());
            dashboard.displayPrintf(10, "Controller B Mode: " + (int)controllerBMode.getTickCurrValue());

            //drivers controller, operation based on the mode selection
            switch ((int)controllerAMode.getTickCurrValue()) {

                //last years driving mode, prefered not to use
                case 1:
                    dashboard.displayPrintf(5, "Controller Mode -- ", "Mecanum Drive Relic Recovery (BAD)");
                    fileLogger.writeEvent(debug,"Controller Mode", "Mecanum Drive Relic Recovery");
                    /*
                     * This was made just for Rover Ruckus becaues the drivers wanted something new!
                     * THis also allows for strafing
                     */
                    Robot.baseMotor1.setPower(Range.clip(-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x, -1, 1));
                    Robot.baseMotor2.setPower(Range.clip(-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x, -1, 1));
                    Robot.baseMotor3.setPower(Range.clip(-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x, -1, 1));
                    Robot.baseMotor4.setPower(Range.clip(-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x, -1, 1));
                    break;

                case 2:
                    fileLogger.writeEvent(debug,"Controller Mode -- ", "POV");
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

                case 3:
                    dashboard.displayPrintf(5, "Controller Mode -- ", "Tank");
                    fileLogger.writeEvent("Controller Mode -- ", "Tank");
                    /*
                     * Case 2 is the controller type Tank drive
                     * Tank uses both joy sticks to drive
                     * The left joy stick is for the left wheel speed
                     * The right joy stick is for the right wheel speed
                     */
                    Robot.setHardwareDriveLeftMotorPower(-gamepad1.left_stick_y * robotPowerMultiplier.getTickCurrValue());
                    Robot.setHardwareDriveRightMotorPower(-gamepad1.right_stick_y * robotPowerMultiplier.getTickCurrValue());
                    break;

                case 4:
                    dashboard.displayPrintf(5, "Controller Mode -- ", "Mecanum Drive 18-19");
                    fileLogger.writeEvent(debug,"Controller Mode -- ", "Mecanum Drive New 2018-19");
                    /*
                     * This is a controller type that is going to be deprecated soon.
                     *
                     */
                    Robot.baseMotor1.setPower(gamepad1.left_stick_x + -gamepad1.left_stick_y + gamepad1.right_stick_x);
                    Robot.baseMotor2.setPower(-gamepad1.left_stick_x + -gamepad1.left_stick_y + gamepad1.right_stick_x);
                    Robot.baseMotor3.setPower(-gamepad1.left_stick_x + -gamepad1.left_stick_y + -gamepad1.right_stick_x);
                    Robot.baseMotor4.setPower(-gamepad1.left_stick_x + -gamepad1.left_stick_y + -gamepad1.right_stick_x);

                    break;

            } //Switch ControllerA

            //controller 2 functions, arms and intakes, depends on which mode selected

            dashboard.displayPrintf(10, "Lift Motor 1 Enc: " + Arms.getLiftMotor1Encoder());
            dashboard.displayPrintf(11, "Lift Motor 2 Enc: " + Arms.getLiftMotor2Encoder());
            //dashboard.displayPrintf(12, "Tilt Motor Enc  : " + Arms.getTiltLiftEncoder());

            switch ((int)controllerBMode.getTickCurrValue()){
                case 1:
                    dashboard.displayPrintf(9, "Controller B Standard");
                    fileLogger.writeEvent(debug,"Controller B Mode", "Standard");

                    //Arms.setHardwareLiftPower(-gamepad2.right_stick_y);
                    runAutoMove();
                    if (gamepad2.left_trigger > .8){
                        Arms.intakeServo1.setPosition(.9);
                        Arms.intakeServo2.setPosition(.9);
                    } else if (gamepad2.right_trigger > .8){
                        Arms.intakeServo1.setPosition(.1);
                        Arms.intakeServo2.setPosition(.1);
                    } else {
                        Arms.intakeServo1.setPosition(0);
                        Arms.intakeServo2.setPosition(0);
                    }

                    Arms.liftMotor1.setPower(-gamepad2.right_stick_y);
                    Arms.liftMotor2.setPower(-gamepad2.right_stick_y);

                    if (gamepad2.a){
                        fileLogger.writeEvent(4, "Got a new job for the auto function now setting the step state to INIT");
                        stepState = Constants.stepState.STATE_INIT;
                        fileLogger.writeEvent(4, "Setting target degree to..... " + 30);
                        targetDegreeMoveToPosition = 30;
                    } //else if (gamepad2.b) {
                        //autoMoveArmRoverRuckus.runNewJOB(10);
                    //}
                    break;
            }

            dashboard.displayText(1, "Tilt Motor 1 Encoder Counts" +  Arms.tiltMotor1.getCurrentPosition());
            dashboard.displayText(2, "Tilt Motor 2 Encoder Counts" +  Arms.tiltMotor2.getCurrentPosition());
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
        controllerAMode.setTickValue(1);
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

    private double[] checkTiltCounts(){
        if (Sensors.getLimitSwitch1AngleMotorState()){
            if (!haveACurrentValue){
                startingAngleMotor1Count = (ourRobotConfig.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH1DEGREEMEASURE) - Arms.tiltMotor1.getCurrentPosition();
                startingAngleMotor2Count = (ourRobotConfig.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH1DEGREEMEASURE) - Arms.tiltMotor2.getCurrentPosition();
                haveACurrentValue = true;
            } else {
                currentTiltEncoderCountMotor1 = (ourRobotConfig.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH1DEGREEMEASURE);
                currentTiltEncoderCountMotor2 = (ourRobotConfig.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH1DEGREEMEASURE);
            }

        } else if (Sensors.getLimitSwitch2AngleMotorState()){
            if (!haveACurrentValue){
                startingAngleMotor1Count = (ourRobotConfig.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH2DEGREEMEASURE) - Arms.tiltMotor1.getCurrentPosition();
                startingAngleMotor2Count = (ourRobotConfig.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH2DEGREEMEASURE) - Arms.tiltMotor2.getCurrentPosition();
                haveACurrentValue = true;
            } else {
                currentTiltEncoderCountMotor1 = (ourRobotConfig.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH2DEGREEMEASURE);
                currentTiltEncoderCountMotor2 = (ourRobotConfig.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH2DEGREEMEASURE);
            }

        } else if (Sensors.getLimitSwitch3AngleMotorState()){
            if (!haveACurrentValue){
                startingAngleMotor1Count = (ourRobotConfig.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH3DEGREEMEASURE) - Arms.tiltMotor1.getCurrentPosition();
                startingAngleMotor2Count = (ourRobotConfig.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH3DEGREEMEASURE) - Arms.tiltMotor2.getCurrentPosition();
                haveACurrentValue = true;
            } else {
                currentTiltEncoderCountMotor1 = (ourRobotConfig.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH3DEGREEMEASURE);
                currentTiltEncoderCountMotor2 = (ourRobotConfig.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH3DEGREEMEASURE);
            }

        } else if (Sensors.getLimitSwitch4AngleMotorState()){
            if (!haveACurrentValue){
                startingAngleMotor1Count = (ourRobotConfig.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH4DEGREEMEASURE) - Arms.tiltMotor1.getCurrentPosition();
                startingAngleMotor2Count = (ourRobotConfig.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH4DEGREEMEASURE) - Arms.tiltMotor2.getCurrentPosition();
                haveACurrentValue = true;
            } else {
                currentTiltEncoderCountMotor1 = (ourRobotConfig.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH4DEGREEMEASURE);
                currentTiltEncoderCountMotor2 = (ourRobotConfig.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH4DEGREEMEASURE);            }
        } else {
            if (haveACurrentValue){
                currentTiltEncoderCountMotor1 = Arms.tiltMotor1.getCurrentPosition() - startingAngleMotor1Count;
                currentTiltEncoderCountMotor2 = Arms.tiltMotor2.getCurrentPosition() - startingAngleMotor2Count;
            }
        }

        fileLogger.writeEvent(8, "Returning current position of the tilt motor 1... " + currentTiltEncoderCountMotor1);
        fileLogger.writeEvent(8, "Returning current position of the tilt motor 2... " + currentTiltEncoderCountMotor2);
        return new double[]{currentTiltEncoderCountMotor1, currentTiltEncoderCountMotor2};
    }

    public void runAutoMove(){
        switch(stepState) {
            case STATE_INIT:
                fileLogger.setEventTag("Auto Move Arm INIT");

                fileLogger.writeEvent("Running INIT Step in Library AutoMoveArmRoverRuckus");

                fileLogger.writeEvent("Setting Target Degree Position of: ", String.valueOf(targetDegreeMoveToPosition));

                double currentTilt1Position = checkTiltCounts()[0];
                double currentTilt2Position = checkTiltCounts()[1];
                fileLogger.writeEvent(4,"Checking the current Tilt 1 Val: " + currentTilt1Position);
                fileLogger.writeEvent(4, "Checking the current Tilt 2 Val: " + currentTilt2Position);

                Arms.tiltMotor1.setTargetPosition((int) ((targetDegreeMoveToPosition * ourRobotConfig.getCOUNTS_PER_DEGREE_TILT()) - currentTilt1Position));
                Arms.tiltMotor2.setTargetPosition((int) ((targetDegreeMoveToPosition * ourRobotConfig.getCOUNTS_PER_DEGREE_TILT()) - currentTilt2Position));

                fileLogger.writeEvent(5, "Setting Power TO: 1");
                Arms.tiltMotor1.setPower(1);
                Arms.tiltMotor2.setPower(1);

                fileLogger.writeEvent(2, "Setting both tilt motors to Run To Position");
                Arms.tiltMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arms.tiltMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                stepState = Constants.stepState.STATE_RUNNING;
                fileLogger.writeEvent(3, "Setting step state to RUNNING now");
                break;

            case STATE_RUNNING:
                if (!Arms.tiltMotor1.isBusy() && !Arms.tiltMotor2.isBusy()) {
                    stepState = STATE_COMPLETE;
                    fileLogger.writeEvent(1,"State now for Auto Drive Rover Ruckus is set to complete");
                    Arms.tiltMotor1.setPower(0);
                    Arms.tiltMotor2.setPower(0);

                }
                fileLogger.writeEvent(10, "Checking Tilt Counts NOW");
                checkTiltCounts();
                break;

            case STATE_COMPLETE:

                if ((gamepad2.left_stick_y == 0))
                {
                    if ((hold == false)) {
                        fileLogger.writeEvent(8, "Hold is Now true");
                        fileLogger.writeEvent(8,"Run using encoders now in hold function in the movement of the arm");
                        Arms.tiltMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        Arms.tiltMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                        fileLogger.writeEvent(8, "Setting the target position so that the arm does not move!");
                        Arms.tiltMotor1.setTargetPosition(Arms.tiltMotor1.getCurrentPosition());
                        Arms.tiltMotor2.setTargetPosition(Arms.tiltMotor2.getCurrentPosition());
                        hold = true;
                    }
                    Arms.tiltMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arms.tiltMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    Arms.tiltMotor1.setPower(HOLDINGTILTMOTORPOWER);
                    Arms.tiltMotor2.setPower(HOLDINGTILTMOTORPOWER);

                    fileLogger.writeEvent(8, "Setting the power to the tilt motor at " + Arms.tiltMotor1.getPower());
                } else {
                    Arms.tiltMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    Arms.tiltMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    hold = false;
                    Arms.tiltMotor1.setPower(-gamepad2.left_stick_y);
                    Arms.tiltMotor2.setPower(-gamepad2.left_stick_y);
                }

                break;
        }
    }


}