package club.towr5291.Concepts;


import android.widget.TextView;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import club.towr5291.R;
import club.towr5291.functions.FileLogger;
import club.towr5291.functions.TOWR5291Toggle;
import club.towr5291.functions.TOWR5291ToggleTick;
import club.towr5291.libraries.robotConfigSettings;
import club.towr5291.opmodes.MecanumBaseDrive_2019;
import club.towr5291.opmodes.OpModeMasterLinear;
import club.towr5291.robotconfig.HardwareDriveMotors;
import hallib.HalDashboard;


/**

 */

@TeleOp(name="Concept: Tick", group="Concept")
//@Disabled
public class ConceptTOWR5291ToggleTickTest extends OpModeMasterLinear
{
    //set up the variables for the logger
    final String TAG = "Concept Logging";
    private ElapsedTime runtime = new ElapsedTime();
    private FileLogger fileLogger;
    private int debug = 1;
    HardwareDriveMotors robot = new HardwareDriveMotors();
    final int LABEL_WIDTH = 200;

    private static HalDashboard dashboard = null;

    public static HalDashboard getDashboard() {
        return dashboard;
    }

    @Override
    public void runOpMode() {
        FtcRobotControllerActivity activity = (FtcRobotControllerActivity) hardwareMap.appContext;

        dashboard = HalDashboard.createInstance(telemetry);
        dashboard = HalDashboard.getInstance();

        dashboard.setTextView((TextView) activity.findViewById(R.id.textOpMode));
        dashboard.displayPrintf(0, LABEL_WIDTH, "Text: ", "*** Robot Data ***");

        //create logging based on initial settings, sharepreferences will adjust levels
        fileLogger = new FileLogger(runtime, debug, true);
        fileLogger.open();
        fileLogger.write("Time,SysMS,Thread,Event,Desc");
        fileLogger.setEventTag("runOpMode()");
        fileLogger.writeEvent("Log Started");

        robot.init(fileLogger, hardwareMap, robotConfigSettings.robotConfigChoice.TileRunnerMecanum2x40, "LM1", "LM2", "RM1", "RM2");
        //robot.setHardwareDriveDirections(robotConfigSettings.robotConfigChoice.TileRunnerMecanum2x40);

        dashboard.displayPrintf(1, "initRobot");
        fileLogger.writeEvent(3, "FileLogger Started");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        dashboard.clearDisplay();
        dashboard.displayPrintf(0, LABEL_WIDTH, "Text: ", "*** Robot Data ***");

        TOWR5291ToggleTick togglex = new TOWR5291ToggleTick(gamepad1.start);
        togglex.setDebounceTimerTime(500);

        togglex.setIncrement(1);

        togglex.setMaxTick(5);
        togglex.setTOWR5291ToggleTickMin(1);

        togglex.setTOWR5291ToggleTickReset(1);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            togglex.addTick(gamepad1.start);
            dashboard.displayPrintf(1, LABEL_WIDTH, "X: ", "" + togglex.getNumberTick());
            dashboard.displayPrintf(2, LABEL_WIDTH, "Left Y ", "" + ((-gamepad1.left_stick_y * togglex.getNumberTick()) / togglex.getTOWR5291ToggleTickMax()));

//            robot.baseMotor1.setPower((-gamepad1.left_stick_y * togglex.getNumberTick()) / 10);
//            robot.baseMotor2.setPower((-gamepad1.left_stick_y * togglex.getNumberTick()) / 10);
//            robot.baseMotor3.setPower((-gamepad1.right_stick_y * togglex.getNumberTick()) / 10);
//            robot.baseMotor4.setPower((-gamepad1.right_stick_y * togglex.getNumberTick()) / 10);

        }

        if (fileLogger != null) {
            fileLogger.writeEvent(TAG, "Stopped");
            fileLogger.close();
            fileLogger = null;
        }
    }
}
