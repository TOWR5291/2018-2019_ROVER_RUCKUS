package club.towr5291.opmodes;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import android.widget.TextView;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import club.towr5291.R;
import club.towr5291.functions.FileLogger;
import club.towr5291.functions.TOWR5291Tick;
import club.towr5291.libraries.TOWRDashBoard;
import club.towr5291.libraries.robotConfig;
import club.towr5291.libraries.robotConfigSettings;
import club.towr5291.robotconfig.HardwareArmMotorsRoverRuckus;
import club.towr5291.robotconfig.HardwareDriveMotors;
import club.towr5291.robotconfig.HardwareSensorsRoverRuckus;

/*
    made by Wyatt Ashley on 8/2/2018
 */
@TeleOp(name = "Motor Test", group = "Test")
public class MotorTest extends OpMode {

    private HardwareDriveMotors Robot = new HardwareDriveMotors();
    private HardwareArmMotorsRoverRuckus Arms = new HardwareArmMotorsRoverRuckus();
    private HardwareSensorsRoverRuckus Sensors = new HardwareSensorsRoverRuckus();
    private LEDControl leds = new LEDControl();
    private ElapsedTime timeLED = new ElapsedTime();

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
    private TOWR5291Tick controllerA = new TOWR5291Tick();

    private Gamepad game2 = gamepad2;
    private Gamepad game1 = gamepad1;

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

        timeLED.startTime();
        leds.init(Sensors, timeLED.milliseconds(), 2000);

        controllerA.setTickMin(1);
        controllerA.setTickMax(4);
        controllerA.setRollOver(true);
        controllerA.setTickIncrement(1);

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

        game2 = gamepad2;
        game1 = gamepad1;
    }


    @Override
    public void loop() {
        Robot.setHardwareDriveDirections(robotConfigSettings.robotConfigChoice.valueOf(ourRobotConfig.getRobotConfigBase()));

        controllerA.incrementTick(game1.start);
        dashboard.displayPrintf(0, String.valueOf(controllerA.getTickCurrValue()));

        switch ((int) controllerA.getTickCurrValue()) {
            case 1:
                Robot.baseMotor4.setPower(0);
                Robot.baseMotor2.setPower(0);
                Robot.baseMotor3.setPower(0);
                Robot.baseMotor1.setPower(-game1.left_stick_y);
                break;
            case 2:
                Robot.baseMotor1.setPower(0);
                Robot.baseMotor3.setPower(0);
                Robot.baseMotor4.setPower(0);
                Robot.baseMotor2.setPower(-game1.left_stick_y);
                break;
            case 3:
                Robot.baseMotor1.setPower(0);
                Robot.baseMotor2.setPower(0);
                Robot.baseMotor4.setPower(0);
                Robot.baseMotor3.setPower(-game1.left_stick_y);
                break;
            case 4:
                Robot.baseMotor1.setPower(0);
                Robot.baseMotor2.setPower(0);
                Robot.baseMotor3.setPower(0);
                Robot.baseMotor4.setPower(-game1.left_stick_y);
                break;

        }
    }
}
