package club.towr5291.Concepts;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import android.widget.TextView;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
import club.towr5291.opmodes.OpModeMasterLinear;
import club.towr5291.robotconfig.HardwareArmMotorsRoverRuckus;
import club.towr5291.robotconfig.HardwareDriveMotors;
import club.towr5291.robotconfig.HardwareSensorsRoverRuckus;

import static club.towr5291.functions.Constants.stepState.STATE_INIT;
import static club.towr5291.functions.Constants.stepState.STATE_RUNNING;

/*
    made by Wyatt Ashley on 8/2/2018
*/
@TeleOp(name = "Limit Test", group = "concept")
public class ConceptRoverRuckusLimitSwitches extends OpModeMasterLinear {

    /* Hardware Set Up */
    private HardwareDriveMotors Robot               = new HardwareDriveMotors();
    private HardwareArmMotorsRoverRuckus Arms       = new HardwareArmMotorsRoverRuckus();
    private HardwareSensorsRoverRuckus Sensors      = new HardwareSensorsRoverRuckus();

    //Settings from the sharepreferences
    private SharedPreferences sharedPreferences;

    private FileLogger fileLogger;
    final String TAG = "RR TeleOp";
    private ElapsedTime runtime                     = new ElapsedTime();
    private club.towr5291.libraries.robotConfig ourRobotConfig;

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
        ourRobotConfig.setRobotConfigBase(sharedPreferences.getString("club.towr5291.Autonomous.RobotConfigBase", "TileRunner-2x40"));

        //now we have loaded the config from sharedpreferences we can setup the robot
        ourRobotConfig.initConfig();

        fileLogger = new FileLogger(runtime, Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", "1")), true);// initializing FileLogger
        fileLogger.open();// Opening FileLogger
        fileLogger.writeEvent(TAG, "Log Started");// First Line Add To Log

        Robot.init(fileLogger, hardwareMap, robotConfigSettings.robotConfigChoice.valueOf(ourRobotConfig.getRobotConfigBase()));// Starting robot Hardware map
        Robot.allMotorsStop();

        Arms.init(hardwareMap, dashboard);
        Arms.setHardwareArmDirections();
        Arms.tiltMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arms.tiltMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fileLogger.writeEvent(2,"Arms Init");

        Sensors.init(fileLogger, hardwareMap);

        Sensors.limitSwitch1AngleMotor.setMode(DigitalChannel.Mode.INPUT);
        Sensors.limitSwitch2AngleMotor.setMode(DigitalChannel.Mode.INPUT);
        Sensors.limitSwitch3AngleMotor.setMode(DigitalChannel.Mode.INPUT);
        Sensors.limitSwitch4AngleMotor.setMode(DigitalChannel.Mode.INPUT);

        // All The Specification of the robot and controller
        fileLogger.writeEvent(1,"Alliance Color", ourRobotConfig.getAllianceColor());
        fileLogger.writeEvent(1,"Alliance Start Position", ourRobotConfig.getAllianceStartPosition());
        fileLogger.writeEvent(1,"Delay", String.valueOf(ourRobotConfig.getDelay()));
        fileLogger.writeEvent(1,"Robot Base Config", ourRobotConfig.getRobotConfigBase());
        fileLogger.writeEvent(1,"Team Number", ourRobotConfig.getTeamNumber());

        fileLogger.writeEvent(1,"","Wait For Start ");

        dashboard.displayPrintf(1, "Waiting for Start");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        dashboard.clearDisplay();

        fileLogger.writeEvent("Starting Loop");

        //the main loop.  this is where the action happens
        while (opModeIsActive()) {
            fileLogger.writeEvent(1,"In Main Loop");

            Arms.tiltMotor1.setPower(-gamepad2.left_stick_y);

            dashboard.displayPrintf(1, "Limit 1" + Sensors.limitSwitch1AngleMotor.getState());
            dashboard.displayPrintf(2, "Limit 2" + Sensors.limitSwitch2AngleMotor.getState());
            dashboard.displayPrintf(3, "Limit 3" + Sensors.limitSwitch3AngleMotor.getState());
            dashboard.displayPrintf(4, "Limit 4" + Sensors.limitSwitch4AngleMotor.getState());
        }  //while (OpModeIsActive)

        //stop the logging
        if (fileLogger != null) {
            fileLogger.writeEvent(1, "TeleOP FINISHED - FINISHED");
            fileLogger.writeEvent(1, "Stopped");
            fileLogger.close();
            fileLogger = null;
        }
    } //RunOpMode
}