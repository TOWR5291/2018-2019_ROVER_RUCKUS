package club.towr5291.opmodes;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import android.widget.TextView;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
import club.towr5291.functions.FileLogger;
import club.towr5291.functions.TOWR5291PID;
import club.towr5291.functions.TOWR5291Tick;
import club.towr5291.libraries.TOWR5291LEDControl;
import club.towr5291.libraries.TOWRDashBoard;
import club.towr5291.libraries.robotConfig;
import club.towr5291.libraries.robotConfigSettings;
import club.towr5291.robotconfig.HardwareArmMotorsRoverRuckus;
import club.towr5291.robotconfig.HardwareDriveMotors;
import club.towr5291.robotconfig.HardwareSensorsRoverRuckus;

/*
    made by Wyatt Ashley on 8/2/2018
 */
@TeleOp(name = "Base Drive 2019", group = "Base drive")
public class BaseDrive_2019 extends OpMode {

    /* Hardware Set Up */
    private HardwareDriveMotors Robot               = new HardwareDriveMotors();
    private HardwareArmMotorsRoverRuckus Arms       = new HardwareArmMotorsRoverRuckus();
    private HardwareSensorsRoverRuckus Sensors      = new HardwareSensorsRoverRuckus();
    private TOWR5291LEDControl LEDs;

    //Settings from the sharepreferences
    private SharedPreferences sharedPreferences;
    double correction = 0;
    double lastposition;
    boolean DisplayEncoderVaule = false;
    int StartCorrectionVar = 0;

    private FileLogger fileLogger;
    final String TAG = "Concept Logging";
    private ElapsedTime runtime = new ElapsedTime();
    private club.towr5291.libraries.robotConfig ourRobotConfig;

    /* TOWR TICK FUNCTION */
    private TOWR5291Tick robotTick                  = new TOWR5291Tick();
    private TOWR5291Tick controllerA                = new TOWR5291Tick();
    private TOWR5291Tick controllerB                = new TOWR5291Tick();
    private TOWR5291Tick IntakeDirection            = new TOWR5291Tick();
    private TOWR5291Tick teamMarkerServoPosition    = new TOWR5291Tick();

    private Gamepad game2 = gamepad2;
    private Gamepad game1 = gamepad1;

    private TOWR5291PID driftRotateAngle;
    private BNO055IMU imu;

    private static TOWRDashBoard dashboard = null;
    public static TOWRDashBoard getDashboard()
    {
        return dashboard;
    }

    @Override
    public void init() {
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
        ourRobotConfig.setRobotConfigBase(sharedPreferences.getString("club.towr5291.Autonomous.RobotConfig", "TileRunner-2x40"));

        fileLogger = new FileLogger(runtime, Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", "1")), true);// initializing FileLogger
        fileLogger.open();// Opening FileLogger
        fileLogger.writeEvent(TAG, "Log Started");// First Line Add To Log

        Robot.init(fileLogger, hardwareMap, robotConfigSettings.robotConfigChoice.valueOf(ourRobotConfig.getRobotConfigBase()));// Starting robot Hardware map
        Robot.logEncoderCounts(fileLogger);// Logging The Encoder Counts
        Robot.allMotorsStop();

        Arms.init(hardwareMap, dashboard);
        Sensors.init(fileLogger, hardwareMap);
        LEDs = new TOWR5291LEDControl(hardwareMap);
        LEDs.setLEDControlDemoMode(true);

        switch (ourRobotConfig.getAllianceStartPosition()){
            case "Left":
                StartCorrectionVar = -45;
                break;
            default:
                StartCorrectionVar = 45;
                break;
        }
        robotTick.setRollOver(true);
        robotTick.setTickMax(1);
        robotTick.setTickMin(0.2);
        robotTick.setTickIncrement(0.4);
        robotTick.setTickValue(.5);

        controllerA.setRollOver(true);
        controllerA.setTickMin(1);
        controllerA.setTickIncrement(1);

        controllerB.setRollOver(true);
        controllerB.setTickMin(1);
        controllerB.setTickMax(1);
        controllerB.setTickIncrement(1);

        teamMarkerServoPosition.setRollOver(true);
        teamMarkerServoPosition.setTickMax(1);
        teamMarkerServoPosition.setTickMin(0);
        teamMarkerServoPosition.setTickIncrement(.25);
        teamMarkerServoPosition.setDebounceTime(1000);

        IntakeDirection.setRollOver(true);
        IntakeDirection.setTickMax(3);
        IntakeDirection.setTickMin(1);
        IntakeDirection.setTickIncrement(1);
        IntakeDirection.setTickValue(2);

        driftRotateAngle = new TOWR5291PID(runtime,0,0,4.5,0,0);

        // All The Specification of the robot and controller
        fileLogger.writeEvent("Alliance Color", ourRobotConfig.getAllianceColor());
        fileLogger.writeEvent("Alliance Start Position", ourRobotConfig.getAllianceStartPosition());
        fileLogger.writeEvent("Delay", String.valueOf(ourRobotConfig.getDelay()));
        fileLogger.writeEvent("Robot Base Config", ourRobotConfig.getRobotConfigBase());
        fileLogger.writeEvent("Team Number", ourRobotConfig.getTeamNumber());
        fileLogger.writeEvent("Robot Controller Max Tick", String.valueOf(controllerA.getTickMax()));
        fileLogger.writeEvent("Robot Controller Min Tick", String.valueOf(controllerA.getTickMin()));
    }

    @Override
    public void start(){
        fileLogger.writeEvent("Starting Loop");
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        lastposition = getAdafruitHeading();

        game2 = gamepad2;
        game1 = gamepad1;
    }


    @Override
    public void loop() {
        Robot.setHardwareDriveDirections(robotConfigSettings.robotConfigChoice.valueOf(ourRobotConfig.getRobotConfigBase()));

        setControllerMaxTick(robotConfigSettings.robotConfigChoice.valueOf(ourRobotConfig.getRobotConfigBase()));

        robotTick.incrementTick(game1.dpad_up);
        robotTick.decrementTick(game1.dpad_down);

        IntakeDirection.incrementTick(game2.left_bumper);

        controllerA.incrementTick(game1.start);
        controllerB.incrementTick(game2.start);
        teamMarkerServoPosition.incrementTick(game2.a);

        dashboard.displayPrintf(0, "Current Tick" + robotTick.getTickCurrValue());

        switch ((int) controllerB.getTickCurrValue()){
            case 1:
                dashboard.displayPrintf(2, "Controller B Standard");
                fileLogger.writeEvent(1,"Controller B Mode", "Standard");

                Arms.angleMotor1.setPower(-game2.left_stick_y);
                Arms.liftMotor.setPower(game2.right_stick_y);
                Arms.liftMotor2.setPower(game2.right_stick_y);
                if (game2.left_trigger > 0){ 
                    Arms.AdvancedOptionsForArms(game2, 5);
                    if (game2.b){
                        if (game2 == gamepad2){
                            game2 = gamepad1;
                        } else if (game2 == gamepad1) {
                            game2 = gamepad2;
                        }
                    }
                    if (game2.x){
                        DisplayEncoderVaule = !DisplayEncoderVaule;
                        if (!DisplayEncoderVaule) {
                            dashboard.clearDisplay();
                        }
                    }
                } else{
                    dashboard.displayPrintf(5, "");
                    dashboard.displayPrintf(6, "");
                    dashboard.displayPrintf(7, "");
                }

                switch ((int) IntakeDirection.getTickCurrValue()){
                    case 1:
                        Arms.intakeServo.setPosition(.1);
                        break;
                    case 2:
                        Arms.intakeServo.setPosition(.5);
                        break;
                    case 3:
                        Arms.intakeServo.setPosition(.9);
                        break;
                }

                Arms.teamMarkerServo.setPosition(teamMarkerServoPosition.getTickCurrValue());
                break;
        }

        switch ((int) controllerA.getTickCurrValue()){
            case 1:
                /*
                 * Case 1 is the controller type POV
                 * POV uses both joy sticks to drive
                 * The left joy stick is for forward and back
                 * The right joy sstick is for turning left and right
                 */
                dashboard.displayPrintf(1, "Controller POV");
                fileLogger.writeEvent(1,"Controller Mode", "POV");
                Robot.setHardwareDriveLeftMotorPower(Range.clip(-game1.left_stick_y + game1.right_stick_x, -1.0, 1.0) * robotTick.getTickCurrValue());
                Robot.setHardwareDriveRightMotorPower(Range.clip(-game1.left_stick_y - game1.right_stick_x, -1.0, 1.0) * robotTick.getTickCurrValue());
                break;

            case 2:
                /*
                 * Case 2 is the controller type Tank drive
                 * Tank uses both joy sticks to drive
                 * The left joy stick is for the left wheel speed
                 * The right joy stick is for the right wheel speed
                 */
                dashboard.displayPrintf(1, "Controller Tank");
                fileLogger.writeEvent("Controller Mode", "Tank");
                Robot.setHardwareDriveLeftMotorPower(-game1.left_stick_y * robotTick.getTickCurrValue());
                Robot.setHardwareDriveRightMotorPower(-game1.right_stick_y * robotTick.getTickCurrValue());
                break;

            case 3:
                /*
                 * Mecanum Drive is for Mecanum bases ONLY
                 * If You Need help ask some one to explain it to you
                 */
                dashboard.displayPrintf(1, "Controller Mecanum Drive");
                fileLogger.writeEvent("Controller Mode", "Mecanum Drive");
                if (game1.left_stick_x != 0 || game1.left_stick_y != 0) {
                    correction = driftRotateAngle.PIDCorrection(runtime,Math.sin(getAdafruitHeading() * (Math.PI / 180.0)), lastposition);
                } else {
                    correction = 0;
                    lastposition = Math.sin(getAdafruitHeading() * (Math.PI / 180.0));
                }

                Robot.mecanumDrive_Cartesian(game1.left_stick_x, game1.left_stick_y, game1.right_stick_x - correction, getAdafruitHeading() + StartCorrectionVar, robotTick.getTickCurrValue());
                break;

            case 4:
                dashboard.displayPrintf(1, "Controller Mecanum Drive New 2018-19");
                fileLogger.writeEvent("Controller Mode", "Mecanum Drive New 2018-19");
                Robot.baseMotor1.setPower(game1.left_stick_x + -game1.left_stick_y + game1.right_stick_x);
                Robot.baseMotor2.setPower(-game1.left_stick_x + -game1.left_stick_y + game1.right_stick_x);
                Robot.baseMotor3.setPower(-game1.left_stick_x + -game1.left_stick_y + -game1.right_stick_x);
                Robot.baseMotor4.setPower(-game1.left_stick_x + -game1.left_stick_y + -game1.right_stick_x);

                break;
        }

        if (DisplayEncoderVaule){
            dashboard.displayPrintf(8, "baseMotor1" + Robot.baseMotor1.getCurrentPosition());
            dashboard.displayPrintf(9, "baseMotor2" + Robot.baseMotor2.getCurrentPosition());
            dashboard.displayPrintf(10, "baseMotor3" + Robot.baseMotor3.getCurrentPosition());
            dashboard.displayPrintf(11, "baseMotor4" + Robot.baseMotor4.getCurrentPosition());
            dashboard.displayPrintf(12, "LiftMotor1" + Arms.liftMotor.getCurrentPosition());
            dashboard.displayPrintf(13, "LiftMotor2" + Arms.liftMotor2.getCurrentPosition());
            dashboard.displayPrintf(14, "AngleMotor1" + Arms.angleMotor1.getCurrentPosition());
        }
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
    private double angleToHeading(double z) {
        double angle = -z;
        if (angle < 0)
            return angle + 360;
        else if (angle > 360)
            return angle - 360;
        else
            return angle;
    }
    private void setControllerMaxTick(robotConfigSettings.robotConfigChoice choice){
        switch (choice){
            case TileRunner2x20: controllerA.setTickMax(2); break;
            case TileRunner2x40: controllerA.setTickMax(2); break;
            case TileRunner2x60: controllerA.setTickMax(2); break;
            case TileRunnerOrbital2x20: controllerA.setTickMax(2); break;
            case TileRunnerMecanumOrbital2x20: controllerA.setTickMax(4); break;
            case TankTread2x40Custom: controllerA.setTickMax(2); break;
            case TileRunnerMecanum2x20: controllerA.setTickMax(3); break;
            case TileRunnerMecanum2x40: controllerA.setTickMax(3); break;
            case TileRunnerMecanum2x60: controllerA.setTickMax(3); break;
        }
        fileLogger.writeEvent(4,"Setting Controller Max Tick");
    }
    private boolean isMecanum (robotConfigSettings.robotConfigChoice choice){
        switch (choice){
            case TileRunner2x20: return false;
            case TileRunner2x40: return false;
            case TileRunner2x60: return false;
            case TileRunnerOrbital2x20: return false;
            case TileRunnerMecanumOrbital2x20: return true;
            case TileRunnerMecanum2x20: return true;
            case TileRunnerMecanum2x40: return true;
            case TileRunnerMecanum2x60: return true;
            case Custom_11231_2016: return false;
            case TankTread2x40Custom: return false;
            default: return false;
        }
    }
}
