package club.towr5291.opmodes;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import android.widget.TextView;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.robotcore.internal.opmode.RegisteredOpModes;

import club.towr5291.R;
import club.towr5291.functions.FileLogger;
import club.towr5291.libraries.FTCChoiceMenu;
import club.towr5291.libraries.FTCMenu;
import club.towr5291.libraries.FTCValueMenu;
import club.towr5291.libraries.TOWRDashBoard;
import club.towr5291.libraries.robotConfigSettings;

/**
 * Created by Ian Haden on 11/7/2016.
 */

@TeleOp(name = "**Configure Robot TEST**", group = "0")
@Disabled
public class AutoSetupMenuTest extends OpModeMasterLinear implements FTCMenu.MenuButtons {

    //set up the variables for the logger
    final String TAG = "Auton Menu";
    private ElapsedTime runtime = new ElapsedTime();
    private FileLogger fileLogger;
    private int debug = 3;

    private int loop = 0;

    private static TOWRDashBoard dashboard = null;

    public static TOWRDashBoard getDashboard()
    {
        return dashboard;
    }

    //The autonomous menu settings using sharedpreferences
    private SharedPreferences sharedPreferences;
    SharedPreferences.Editor editor;
    private String teamNumber;
    private String allianceColor;
    private String allianceStartPosition;
    private String allianceParkPosition;
    private int delay;
    private String numBeacons;
    private String robotConfig;
    private String autonOpMode;
    private String teleOpMode;

    //which opmode to load after config done
    private OpMode onStop = this;
    private OpModeManagerImpl opModeManager;
    private RegisteredOpModes registeredOpModes;

    private static AutoSetupMenuTest instance = null;

    public AutoSetupMenuTest()
    {
        super();
        instance = this;
    }

    //
    // Implements FtcMenu.MenuButtons interface.
    //

    @Override
    public boolean isMenuUpButton() { return gamepad1.dpad_up;} //isMenuUpButton

    @Override
    public boolean isMenuDownButton()
    {
        return gamepad1.dpad_down;
    } //isMenuDownButton

    @Override
    public boolean isMenuEnterButton()
    {
        return gamepad1.a;
    } //isMenuEnterButton

    @Override
    public boolean isMenuBackButton()
    {
        return gamepad1.dpad_left;
    }  //isMenuBackButton

    private void doMenus()
    {

        //load variables
        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        editor = sharedPreferences.edit();
        teamNumber = sharedPreferences.getString("club.towr5291.Autonomous.TeamNumber", "0000");
        allianceColor = sharedPreferences.getString("club.towr5291.Autonomous.Color", "Red");
        allianceStartPosition = sharedPreferences.getString("club.towr5291.Autonomous.StartPosition", "Left");
        delay = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Delay", "0"));
        robotConfig = sharedPreferences.getString("club.towr5291.Autonomous.RobotConfig", "TileRunner-Mecanum-2x40");
        autonOpMode = sharedPreferences.getString("club.towr5291.Autonomous.AutonOpMode", "");
        teleOpMode =  sharedPreferences.getString("club.towr5291.Autonomous.TeleOpMode", "");
        debug = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", "1"));

        //
        // Create the menus.
        //
        FTCChoiceMenu teamMenu           = new FTCChoiceMenu("robotConfigTeam:", null, this);
        FTCChoiceMenu allianceMenu       = new FTCChoiceMenu("Alliance:", teamMenu, this);
        FTCChoiceMenu startPosMenu       = new FTCChoiceMenu("Start:", allianceMenu, this);
        FTCValueMenu delayMenu           = new FTCValueMenu("Delay:", startPosMenu, this, 0.0, 20.0, 1.0, delay, "%1f");
        FTCChoiceMenu robotConfigMenu    = new FTCChoiceMenu("Robot:", delayMenu, this);
        FTCChoiceMenu AutonOpModeMenu    = new FTCChoiceMenu("AutoOpMode:", robotConfigMenu, this);
        FTCChoiceMenu TeleOpModeMenu     = new FTCChoiceMenu("TeleOpMode:", AutonOpModeMenu, this);
        FTCChoiceMenu debugConfigMenu    = new FTCChoiceMenu("Debug:", TeleOpModeMenu, this);

        //
        // remember last saved settings and reorder the menu with last run settings as the defaults
        //
        boolean selected;
        for (robotConfigSettings.robotConfigTeam team : robotConfigSettings.robotConfigTeam.values()) {
            if (teamNumber.equals(team)) {
                selected = true;
            } else {
                selected = false;
            }
            teamMenu.addChoice(team.toString(), team, selected, allianceMenu);
        }

        for (robotConfigSettings.Alliance alliance : robotConfigSettings.Alliance.values()) {
            if (allianceColor.equals(alliance)) {
                selected = true;
            } else {
                selected = false;
            }
            allianceMenu.addChoice(alliance.toString(), alliance, selected, startPosMenu);
        }

        for (robotConfigSettings.robotConfigStartPos startPos : robotConfigSettings.robotConfigStartPos.values()) {
            if (allianceStartPosition.equals(startPos)) {
                selected = true;
            } else {
                selected = false;
            }
            startPosMenu.addChoice(startPos.toString(), startPos, selected, delayMenu);
        }

        delayMenu.setChildMenu(robotConfigMenu);

        for (robotConfigSettings.robotConfigChoice robotConfigChoice : robotConfigSettings.robotConfigChoice.values()) {

            if (robotConfig.equals(robotConfigChoice)) {
                selected = true;
            } else {
                selected = false;
            }
            robotConfigMenu.addChoice(robotConfigChoice.toString(), robotConfigChoice, selected, AutonOpModeMenu);
        }

        registeredOpModes = registeredOpModes.getInstance();
        for (OpModeMeta opModes : registeredOpModes.getOpModes()) {
            if (autonOpMode.equals(opModes)) {
                selected = true;
            } else {
                selected = false;
            }
            AutonOpModeMenu.addChoice(opModes.name, opModes, selected, TeleOpModeMenu);
        }

        for (OpModeMeta opModes : registeredOpModes.getOpModes()) {
            if (teleOpMode.equals(opModes)) {
                selected = true;
            } else {
            selected = false;
            }
            TeleOpModeMenu.addChoice(opModes.name, opModes, selected, debugConfigMenu);
        }

        for (int x = 1; x <= 10; x++) {
            if (debug == x) {
                selected = true;
            } else {
                selected = false;
            }
            debugConfigMenu.addChoice("" + x, x, selected);
        }

        //
        // Walk the menu tree starting with the strategy menu as the root
        // menu and get user choices.
        //
        FTCMenu.walkMenuTree(teamMenu, this);

        //
        // Set choices variables
        //
        allianceStartPosition = startPosMenu.getCurrentChoiceText();
        allianceColor = allianceMenu.getCurrentChoiceText();
        teamNumber = teamMenu.getCurrentChoiceText();
        robotConfig = robotConfigMenu.getCurrentChoiceText();
        delay = (int)delayMenu.getCurrentValue();
        autonOpMode = AutonOpModeMenu.getCurrentChoiceText();
        teleOpMode = TeleOpModeMenu.getCurrentChoiceText();
        debug = Integer.parseInt(debugConfigMenu.getCurrentChoiceText());

        //write the options to sharedpreferences
        editor.putString("club.towr5291.Autonomous.TeamNumber", teamNumber);
        editor.putString("club.towr5291.Autonomous.Color", allianceColor);
        editor.putString("club.towr5291.Autonomous.StartPosition", allianceStartPosition);
        editor.putString("club.towr5291.Autonomous.Delay", String.valueOf(delay));
        editor.putString("club.towr5291.Autonomous.RobotConfig", robotConfig);
        editor.putString("club.towr5291.Autonomous.AutonOpMode", autonOpMode);
        editor.putString("club.towr5291.Autonomous.TeleOpMode", teleOpMode);
        editor.putString("club.towr5291.Autonomous.Debug", String.valueOf(debug));
        editor.commit();

        //read them back to ensure they were written
        teamNumber = sharedPreferences.getString("club.towr5291.Autonomous.TeamNumber", null);
        allianceColor = sharedPreferences.getString("club.towr5291.Autonomous.Color", null);
        allianceStartPosition = sharedPreferences.getString("club.towr5291.Autonomous.StartPosition", null);
        delay = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Delay", null));
        robotConfig = sharedPreferences.getString("club.towr5291.Autonomous.RobotConfig", null);
        autonOpMode = sharedPreferences.getString("club.towr5291.Autonomous.AutonOpMode", "");
        teleOpMode =  sharedPreferences.getString("club.towr5291.Autonomous.TeleOpMode", "");
        debug = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", null));

        int lnum = 1;
        dashboard.displayPrintf(lnum++, "robotConfigTeam:     " + teamNumber);
        dashboard.displayPrintf(lnum++, "Alliance: " + allianceColor);
        dashboard.displayPrintf(lnum++, "Start:    " + allianceStartPosition);
        dashboard.displayPrintf(lnum++, "Delay:    " + String.valueOf(delay));
        dashboard.displayPrintf(lnum++, "Robot:    " + robotConfig);
        dashboard.displayPrintf(lnum++, "Auton:    " + autonOpMode);
        dashboard.displayPrintf(lnum++, "TeleOp:   " + teleOpMode);
        dashboard.displayPrintf(lnum++, "Debug:    " + debug);

        fileLogger.writeEvent("AutonConfig", "robotConfigTeam     " + teamNumber);
        fileLogger.writeEvent("AutonConfig", "Alliance " + allianceColor);
        fileLogger.writeEvent("AutonConfig", "Start    " + allianceStartPosition);
        fileLogger.writeEvent("AutonConfig", "Delay    " + String.valueOf(delay));
        fileLogger.writeEvent("AutonConfig", "Robot    " + robotConfig);
        fileLogger.writeEvent("AutonConfig", "Auton    " + autonOpMode);
        fileLogger.writeEvent("AutonConfig", "TeleOp   " + teleOpMode);
        fileLogger.writeEvent("AutonConfig", "Debug:   " + debug);
    }

    public void initRobot()
    {
        //start the log
        fileLogger = new FileLogger(runtime,3,true);
        fileLogger.open();
        fileLogger.write("Time,SysMS,Thread,Event,Desc");
        fileLogger.writeEvent(TAG, "Log Started");
    }   //initRobot

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();

        dashboard = TOWRDashBoard.createInstance(telemetry);

        FtcRobotControllerActivity act = (FtcRobotControllerActivity)(hardwareMap.appContext);

        dashboard.setTextView((TextView)act.findViewById(R.id.textOpMode));
        dashboard.displayPrintf(0, "Starting Menu System");

        dashboard.displayPrintf(0, "INITIALIZING - Please wait for Menu");
        fileLogger.writeEvent(TAG, "SETUP");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        doMenus();
        dashboard.displayPrintf(0, "COMPLETE - Settings Written");
        Thread.sleep(1000);
        stopMode();

        //switch opmode to teleop
        //opModeManager = (OpModeManagerImpl) onStop.internalOpModeServices;
        //opModeManager.initActiveOpMode(TeleOpMode);
        //opmode not active anymore
    }

    public void stopMode()
    {
        //stop the log
        if (fileLogger != null) {
            fileLogger.writeEvent(TAG, "Stopped");
            fileLogger.close();
            fileLogger = null;
        }
    }   //stopMode

}


