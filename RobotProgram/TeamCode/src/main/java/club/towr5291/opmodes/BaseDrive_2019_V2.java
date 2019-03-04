package club.towr5291.opmodes;


import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import android.widget.TextView;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

Written by Wyatt Ashley October 2018
2018-10-30 - Ian Haden  - Converted to OpModeMasterLinear from normal OpMode
                        - Made an init function so the standard functions can be initailised in a single location
                        - converted hard coded numbers to variables
                        - Fixed Reference to LEDs
*/
@TeleOp(name="TeleOp Rover Ruckus V2", group="5291")
@Disabled
public class BaseDrive_2019_V2 extends OpModeMasterLinear {

    /* Hardware Set Up */
    private HardwareDriveMotors Robot               = new HardwareDriveMotors();
    private HardwareArmMotorsRoverRuckus Arms       = new HardwareArmMotorsRoverRuckus();
    private HardwareSensorsRoverRuckus Sensors      = new HardwareSensorsRoverRuckus();
    private TOWR5291LEDControl LEDs;
    private Constants.LEDState LEDStatus = Constants.LEDState.STATE_ERROR;

    //Settings from the sharepreferences
    private SharedPreferences sharedPreferences;
    double correction = 0;
    double lastposition;
    boolean DisplayEncoderVaule = false;
    private int imuStartCorrectionVar = 0;
    private int imuMountCorrectionVar = 90;

    private FileLogger fileLogger;
    final String TAG = "RR TeleOp";
    private ElapsedTime runtime = new ElapsedTime();
    private club.towr5291.libraries.robotConfig ourRobotConfig;

    /* TOWR TICK FUNCTION */
    private TOWR5291Tick robotPowerMultiplier       = new TOWR5291Tick();
    private TOWR5291Tick controllerAMode            = new TOWR5291Tick();
    private TOWR5291Tick controllerBMode            = new TOWR5291Tick();
    private TOWR5291Tick intakeDirection            = new TOWR5291Tick();
    private TOWR5291Tick teamMarkerServoPosition    = new TOWR5291Tick();

    private Gamepad game2 = gamepad2;
    private Gamepad game1 = gamepad1;

    private TOWR5291PID driftRotateAngle;
    private BNO055IMU imu;

    private double maxDrivePower = 1;
    private double minDrivePower = 0.3333333333333;
    private double incrementDrivePower = 0.333333333333;
    private double startDrivePower = 0.6666666666666666;
    //Controller A has 4 modes of operation
    private double controllerAModes = 5;
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

        BNO055IMU.Parameters parametersAdafruitImu  = new BNO055IMU.Parameters();
        parametersAdafruitImu.angleUnit             = BNO055IMU.AngleUnit.DEGREES;
        parametersAdafruitImu.accelUnit             = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersAdafruitImu.calibrationDataFile   = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parametersAdafruitImu.loggingEnabled        = true;
        parametersAdafruitImu.loggingTag            = "IMU";
        parametersAdafruitImu.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parametersAdafruitImu);


        ourRobotConfig = new robotConfig();
        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);

        ourRobotConfig.setAllianceColor(sharedPreferences.getString("club.towr5291.Autonomous.Color", "Red"));// Using a Function to Store The Robot Specification
        ourRobotConfig.setTeamNumber(sharedPreferences.getString("club.towr5291.Autonomous.TeamNumber", "0000"));
        ourRobotConfig.setAllianceStartPosition(sharedPreferences.getString("club.towr5291.Autonomous.Position", "Left"));
        ourRobotConfig.setDelay(Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Delay", "0")));
        ourRobotConfig.setRobotConfigBase(sharedPreferences.getString("club.towr5291.Autonomous.RobotConfigBase", "TileRunner-2x40"));
        debug = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", "1"));

        fileLogger = new FileLogger(runtime, Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", "1")), true);// initializing FileLogger
        fileLogger.open();// Opening FileLogger
        fileLogger.writeEvent(TAG, "Log Started");// First Line Add To Log

        Robot.init(fileLogger, hardwareMap, robotConfigSettings.robotConfigChoice.valueOf(ourRobotConfig.getRobotConfigBase()));// Starting robot Hardware map
        Robot.allMotorsStop();

        Arms.init(hardwareMap, dashboard);
        fileLogger.writeEvent(2,"Arms Init");

        Sensors.init(fileLogger, hardwareMap);

        fileLogger.writeEvent(2,"Sensors Init");
        LEDs = new TOWR5291LEDControl(hardwareMap);
        LEDs.setLEDControlDemoMode(true);
        LEDs.setLEDControlAlliance(ourRobotConfig.getAllianceColor());

        fileLogger.writeEvent(2,"LEDs Init");

        switch (ourRobotConfig.getAllianceStartPosition()){
            case "Left":
                imuStartCorrectionVar = -45;
                break;
            default:
                imuStartCorrectionVar = 45;
                break;
        }

        fileLogger.writeEvent(2,"IMU Offset " + imuStartCorrectionVar);

        //init all the values for the counters etc
        initFunction();
        fileLogger.writeEvent(2,"Init Function Done ");

        driftRotateAngle = new TOWR5291PID(runtime,0,0,4.5,0,0);

        TOWR5291Toggle toggleGamePad2A = new TOWR5291Toggle(gamepad1.x);
        toggleGamePad2A.setDebounce(500);
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

        game2 = gamepad2;
        game1 = gamepad1;

        fileLogger.writeEvent(1,"","Wait For Start ");

        dashboard.displayPrintf(1, "Waiting for Start");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        dashboard.clearDisplay();

        fileLogger.writeEvent("Starting Loop");
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        lastposition = getAdafruitHeading();

        //dashboard.clearDisplay();

        dashboard.displayPrintf(1, "Controller A Options");
        dashboard.displayPrintf(2, "--------------------");
        dashboard.displayPrintf(6, "Controller B Options");
        dashboard.displayPrintf(7, "--------------------");

        //the main loop.  this is where the action happens
        while (opModeIsActive()) {
            fileLogger.writeEvent(1,"In Main Loop");
            //change LED state every cycle
            LEDStatus = LEDs.LEDControlUpdate(LEDStatus);

            //adjust the robot power using the dpad on the game controller
            robotPowerMultiplier.incrementTick(game1.dpad_up);
            robotPowerMultiplier.decrementTick(game1.dpad_down);

            //adjust the intake direction, toggle forward, backward
            intakeDirection.incrementTick(game2.left_bumper);

            controllerAMode.incrementTick(game1.start);
            controllerBMode.incrementTick(game2.start);

            dashboard.displayPrintf(3, "Power Multiplier:  " + robotPowerMultiplier.getTickCurrValue());
            dashboard.displayPrintf(4, "Controller A Mode: " + (int)controllerAMode.getTickCurrValue());
            dashboard.displayPrintf(8, "Controller B Mode: " + (int)controllerBMode.getTickCurrValue());

            //drivers controller, operation based on the mode selection
            switch ((int)controllerAMode.getTickCurrValue())
            {
                case 1:
                    fileLogger.writeEvent(debug,"Controller Mode", "POV");
                    dashboard.displayPrintf(4, "Controller POV:");
                    /*
                     * Case 1 is the controller type POV
                     * POV uses both joy sticks to drive
                     * The left joy stick is for forward and back
                     * The right joystick is for turning left and right
                     */
                    fileLogger.writeEvent(debug,"Controller Mode", "Robot Multiplier: -" + robotPowerMultiplier.getTickCurrValue());
                    Robot.setHardwareDriveLeftMotorPower(Range.clip(-game1.left_stick_y + game1.right_stick_x, -1.0, 1.0) * robotPowerMultiplier.getTickCurrValue());
                    Robot.setHardwareDriveRightMotorPower(Range.clip(-game1.left_stick_y - game1.right_stick_x, -1.0, 1.0) * robotPowerMultiplier.getTickCurrValue());
                    fileLogger.writeEvent(debug,"SetPowers Done");
                    break;

                case 2:
                    /*
                     * Case 2 is the controller type Tank drive
                     * Tank uses both joy sticks to drive
                     * The left joy stick is for the left wheel speed
                     * The right joy stick is for the right wheel speed
                     */
                    dashboard.displayPrintf(4, "Controller Tank");
                    fileLogger.writeEvent("Controller Mode", "Tank");
                    Robot.setHardwareDriveLeftMotorPower(-game1.left_stick_y * robotPowerMultiplier.getTickCurrValue());
                    Robot.setHardwareDriveRightMotorPower(-game1.right_stick_y * robotPowerMultiplier.getTickCurrValue());
                    break;

                case 3:
                    /*
                     * Mecanum Drive is for Mecanum bases ONLY
                     * If You Need help ask some one to explain it to you
                     */
                    dashboard.displayPrintf(4, "Mecanum Drive (IMU)");
                    fileLogger.writeEvent("Controller Mode", "Mecanum Drive");
                    if (game1.left_stick_x != 0 || game1.left_stick_y != 0) {
                        correction = driftRotateAngle.PIDCorrection(runtime,Math.sin(getAdafruitHeading() * (Math.PI / 180.0)), lastposition);
                    } else {
                        correction = 0;
                        lastposition = Math.sin(getAdafruitHeading() * (Math.PI / 180.0));
                    }

                    Robot.mecanumDrive_Cartesian(game1.left_stick_x, game1.left_stick_y, game1.right_stick_x - correction, getAdafruitHeading() + imuStartCorrectionVar, robotPowerMultiplier.getTickCurrValue());
                    break;

                case 4:
                    dashboard.displayPrintf(4, "Mecanum Drive New 2018-19");
                    fileLogger.writeEvent(debug,"Controller Mode", "Mecanum Drive New 2018-19");
                    Robot.baseMotor1.setPower(game1.left_stick_x + -game1.left_stick_y + game1.right_stick_x);
                    Robot.baseMotor2.setPower(-game1.left_stick_x + -game1.left_stick_y + game1.right_stick_x);
                    Robot.baseMotor3.setPower(-game1.left_stick_x + -game1.left_stick_y + -game1.right_stick_x);
                    Robot.baseMotor4.setPower(-game1.left_stick_x + -game1.left_stick_y + -game1.right_stick_x);

                    break;
                //last years driving mode, prefered not to use
                case 5:
                    dashboard.displayPrintf(4, "Mecanum Drive Relic Recovery");
                    fileLogger.writeEvent(debug,"Controller Mode", "Mecanum Drive Relic Recovery");
                    Robot.baseMotor1.setPower(Range.clip(-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x, -1, 1));
                    Robot.baseMotor2.setPower(Range.clip(-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x, -1, 1));
                    Robot.baseMotor3.setPower(Range.clip(-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x, -1, 1));
                    Robot.baseMotor4.setPower(Range.clip(-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x, -1, 1));
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

                    Arms.tiltMotor1.setPower(-game2.left_stick_y);
                    Arms.setHardwareLiftPower(-game2.right_stick_y);
                    if (game2.left_trigger > 0){
                        //Arms.AdvancedOptionsForArms(LEDs);
                        if (game2.b){
                            if (game2 == gamepad2){
                                game2 = gamepad1;
                            } else if (game2 == gamepad1) {
                                game2 = gamepad2;
                            }
                        }
                    }

                    if (toggleGamePad2A.toggleState(gamepad1.a)) {
                        Arms.setHardwareArmDirections(DcMotor.Direction.REVERSE);
                    } else {
                        Arms.setHardwareArmDirections(DcMotor.Direction.FORWARD);
                    }

                    switch ((int) intakeDirection.getTickCurrValue()){
                        case 1:
                            //Arms.intakeServo.setPosition(.1);
                            break;
                        case 2:
                            //Arms.intakeServo.setPosition(.5);
                            break;
                        case 3:
                            //Arms.intakeServo.setPosition(.9);
                            break;
                    } //switch intake direction

                    //Arms.teamMarkerServo.setPosition(teamMarkerServoPosition.getTickCurrValue());
                    break;
            } //Switch ControllerB

        }  //while (OpModeIsActive)

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

        fileLogger.writeEvent(debug, "initFunctions" ,  "intakeDirection Start");
        intakeDirection.setRollOver(true);
        intakeDirection.setTickMax(3);
        intakeDirection.setTickMin(1);
        intakeDirection.setTickIncrement(1);
        intakeDirection.setTickValue(2);
        fileLogger.writeEvent(debug, "initFunctions" ,  "intakeDirection End");
        fileLogger.writeEvent(debug, "initFunctions" ,  "Finished");

    }

    private Double getAdafruitHeading () {
        Orientation angles;
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return angleToHeading(formatAngle(angles.angleUnit, angles.firstAngle));
    }

    private Double formatAngle(AngleUnit angleUnit, double angle) {
        fileLogger.writeEvent(4,"Formating Angle For Gyro");
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

}
