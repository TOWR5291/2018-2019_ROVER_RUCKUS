package club.towr5291.Concepts;


import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import android.widget.TextView;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import club.towr5291.R;
import club.towr5291.functions.Constants;
import club.towr5291.functions.FileLogger;
import club.towr5291.functions.TOWR5291Tick;
import club.towr5291.functions.TOWR5291Utils;
import club.towr5291.libraries.LibraryMotorType;
import club.towr5291.libraries.TOWRDashBoard;
import club.towr5291.libraries.robotConfig;
import club.towr5291.libraries.robotConfigSettings;
import club.towr5291.opmodes.OpModeMasterLinear;
import club.towr5291.robotconfig.HardwareDriveMotors;

/**
 * Created by Ian Haden TOWR5291 on 7/22/2018.
 * This opmode tests the TICK Class and motor max speed
 *
 * Modification history
 * Edited by:
 * Ian Haden 07/22/2018 -> Added comments and removed some unneeded steps, renamed some variables so they represent their respective functions
 */


@TeleOp(name="Concept: Tick Base Drive", group="5291Concept")
@Disabled
public class ConceptTickTestMaxPower extends OpModeMasterLinear
{
    //set up the variables for the logger
    final String TAG = "Concept Logging";
    private ElapsedTime runtime = new ElapsedTime();
    private FileLogger fileLogger;
    private int debug = 1;
    HardwareDriveMotors robot = new HardwareDriveMotors();
    final int LABEL_WIDTH = 200;

    private static TOWRDashBoard dashboard = null;

    public static TOWRDashBoard getDashboard() {
        return dashboard;
    }

    //The autonomous menu settings from the sharepreferences
    private SharedPreferences sharedPreferences;
    private robotConfig ourRobotConfig;

    @Override
    public void runOpMode() {
        FtcRobotControllerActivity activity = (FtcRobotControllerActivity) hardwareMap.appContext;

        dashboard = TOWRDashBoard.createInstance(telemetry);
        dashboard = TOWRDashBoard.getInstance();

        dashboard.setTextView((TextView) activity.findViewById(R.id.textOpMode));
        dashboard.displayPrintf(0, LABEL_WIDTH, "Text: ", "*** Robot Data ***");

        //create logging based on initial settings, sharepreferences will adjust levels
        fileLogger = new FileLogger(runtime, debug, true);
        fileLogger.open();
        fileLogger.write("Time,SysMS,Thread,Event,Desc");
        fileLogger.setEventTag("runOpMode()");
        fileLogger.writeEvent("Log Started");

        //load menu settings and setup robot and debug level
        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        ourRobotConfig = new robotConfig();

        ourRobotConfig.setAllianceColor(sharedPreferences.getString("club.towr5291.Autonomous.Color", "Red"));// Using a Function to Store The Robot Specification
        ourRobotConfig.setTeamNumber(sharedPreferences.getString("club.towr5291.Autonomous.TeamNumber", "0000"));
        ourRobotConfig.setAllianceStartPosition(sharedPreferences.getString("club.towr5291.Autonomous.Position", "Left"));
        ourRobotConfig.setDelay(Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Delay", "0")));
        ourRobotConfig.setRobotMotorType(sharedPreferences.getString("club.towr5291.Autonomous.RobotMotorChoice", "ANDY40SPUR"));
        ourRobotConfig.setRobotConfigBase(sharedPreferences.getString("club.towr5291.Autonomous.RobotConfigBase", "TileRunner2x40"));
        debug = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", "1"));

        //now we have loaded the config from sharedpreferences we can setup the robot
        ourRobotConfig.initConfig();

        robot.init(fileLogger, hardwareMap, robotConfigSettings.robotConfigChoice.valueOf(ourRobotConfig.getRobotConfigBase()), LibraryMotorType.MotorTypes.valueOf(ourRobotConfig.getRobotMotorType()));
        //robot.setHardwareDriveDirections(robotConfigSettings.robotConfigChoice.TileRunnerMecanum2x40);

        dashboard.displayPrintf(1, "initRobot");
        fileLogger.writeEvent(3, "FileLogger Started");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        dashboard.clearDisplay();
        dashboard.displayPrintf(0, LABEL_WIDTH, "Text: ", "*** Robot Data ***");

        TOWR5291Tick tickTest = new TOWR5291Tick();

        tickTest.setTickIncrement(.1);
        tickTest.setTickMax(1);
        tickTest.setTickMin(0.3);
        tickTest.setTickValue(1);
        tickTest.setRollOver(true);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            tickTest.incrementTick(gamepad1.dpad_up);
            tickTest.decrementTick(gamepad1.dpad_down);

            robot.setMaxOutput(tickTest.getTickCurrValue());

            dashboard.displayPrintf(1, LABEL_WIDTH, "Tick Value: ", "" + tickTest.getTickCurrValue());
            dashboard.displayPrintf(2, LABEL_WIDTH, "Tick Max  : ", "" + tickTest.getTickMax());
            dashboard.displayPrintf(3, LABEL_WIDTH, "Tick Min  : ", "" + tickTest.getTickMin());
            dashboard.displayPrintf(4, LABEL_WIDTH, "Robot Max : ", "" + robot.getMaxOutput());
            dashboard.displayPrintf(5, LABEL_WIDTH, "L Stick   : ", "" + gamepad1.left_stick_y);
            dashboard.displayPrintf(6, LABEL_WIDTH, "R Stick   : ", "" + gamepad1.right_stick_y);
            dashboard.displayPrintf(7, LABEL_WIDTH, "Voltage   : ", "" + TOWR5291Utils.getBatteryVoltage(hardwareMap));
            dashboard.displayPrintf(8, LABEL_WIDTH, "# Lines   : ", "" + dashboard.getNumTextLines());

            robot.setHardwareDriveLeftMotorPower(gamepad1.left_stick_y);
            robot.setHardwareDriveRightMotorPower(gamepad1.right_stick_y);

        }

        if (fileLogger != null) {
            fileLogger.writeEvent(TAG, "Stopped");
            fileLogger.close();
            fileLogger = null;
        }
    }

}
