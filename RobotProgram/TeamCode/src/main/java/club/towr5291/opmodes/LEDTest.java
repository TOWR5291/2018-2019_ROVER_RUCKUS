package club.towr5291.opmodes;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import android.widget.TextView;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import club.towr5291.R;
import club.towr5291.functions.Constants;
import club.towr5291.functions.FileLogger;
import club.towr5291.functions.TOWR5291Tick;
import club.towr5291.libraries.TOWR5291LEDControl;
import club.towr5291.libraries.TOWRDashBoard;
import club.towr5291.libraries.robotConfig;
import club.towr5291.libraries.robotConfigSettings;
import club.towr5291.robotconfig.HardwareDriveMotors;

public class LEDTest extends LinearOpMode {

    public TOWR5291LEDControl leds;
    private SharedPreferences sharedPreferences;

    private FileLogger fileLogger;
    final String TAG = "Concept Logging";
    private ElapsedTime runtime = new ElapsedTime();
    private club.towr5291.libraries.robotConfig ourRobotConfig;
    private TOWR5291Tick controllerA = new TOWR5291Tick();
    private TOWR5291Tick controllerB = new TOWR5291Tick();

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
        ourRobotConfig.setRobotConfigBase(sharedPreferences.getString("club.towr5291.Autonomous.RobotConfig", "TileRunner-2x40"));

        fileLogger = new FileLogger(runtime, Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", "1")), true);// initializing FileLogger
        fileLogger.open();// Opening FileLogger
        fileLogger.writeEvent(TAG, "Log Started");// First Line Add To Log

        // Set up the LEDS
        leds = new TOWR5291LEDControl(hardwareMap, "green1", "red1", "blue1", "green2", "red2", "blue2");
        leds.setLEDControlDemoMode(false);
        leds.setLEDColour(Constants.LEDColours.LED_MAGENTA);
        dashboard.displayPrintf(9, "initRobot LED Initiated!");

        controllerA.setTickMin(1);
        controllerA.setTickMax(4);
        controllerA.setRollOver(true);
        controllerA.setTickIncrement(1);

        controllerB.setTickMin(1);
        controllerB.setTickMax(4);
        controllerB.setRollOver(true);
        controllerB.setTickIncrement(1);

        leds.setLEDLeftColour(Constants.LEDColours.LED_WHITE);
        leds.setLEDRightColour(Constants.LEDColours.LED_WHITE);

        // All The Specification of the robot and controller
        fileLogger.writeEvent("Alliance Color", ourRobotConfig.getAllianceColor());
        fileLogger.writeEvent("Alliance Start Position", ourRobotConfig.getAllianceStartPosition());
        fileLogger.writeEvent("Delay", String.valueOf(ourRobotConfig.getDelay()));
        fileLogger.writeEvent("Robot Base Config", ourRobotConfig.getRobotConfigBase());
        fileLogger.writeEvent("Team Number", ourRobotConfig.getTeamNumber());
        fileLogger.writeEvent("Robot Controller Max Tick", String.valueOf(controllerA.getTickMax()));
        fileLogger.writeEvent("Robot Controller Min Tick", String.valueOf(controllerB.getTickMin()));
        waitForStart();

        while(opModeIsActive()){
            dashboard.displayText(1, "A To Change Left Color");
            dashboard.displayText(2, "B To Change Right Color");

            controllerA.incrementTick(gamepad1.a);
            controllerB.incrementTick(gamepad1.b);

            switch ((int) controllerA.getTickCurrValue()){
                case 1:
                    leds.setLEDLeftColour(Constants.LEDColours.LED_RED);
                    dashboard.displayText(3, "Current Color Left LED: RED");
                    break;
                case 2:
                    leds.setLEDLeftColour(Constants.LEDColours.LED_BLUE);
                    dashboard.displayText(3, "Current Color Left LED: BLUE");
                    break;
                case 3:
                    leds.setLEDLeftColour(Constants.LEDColours.LED_GREEN);
                    dashboard.displayText(3, "Current Color Left LED: GREEN");
                    break;
                case 4:
                    leds.setLEDLeftColour(Constants.LEDColours.LED_WHITE);
                    dashboard.displayText(3, "Current Color Left LED: WHITE");
                    break;
            }

            switch ((int) controllerB.getTickCurrValue()){
                case 1:
                    leds.setLEDRightColour(Constants.LEDColours.LED_RED);
                    dashboard.displayText(4, "Current Color Right LED: RED");
                    break;
                case 2:
                    leds.setLEDRightColour(Constants.LEDColours.LED_BLUE);
                    dashboard.displayText(4, "Current Color Right LED: BLUE");
                    break;
                case 3:
                    leds.setLEDRightColour(Constants.LEDColours.LED_GREEN);
                    dashboard.displayText(4, "Current Color Right LED: GREEN");
                    break;
                case 4:
                    leds.setLEDRightColour(Constants.LEDColours.LED_WHITE);
                    dashboard.displayText(4, "Current Color Right LED: WHITE");
                    break;
            }
        }
    }
}
