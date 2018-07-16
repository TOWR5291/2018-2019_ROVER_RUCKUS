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
import club.towr5291.opmodes.OpModeMasterLinear;
import hallib.HalDashboard;


/**

 */

@TeleOp(name="Concept: Toggle", group="Concept")
//@Disabled
public class ConceptToggleTest extends OpModeMasterLinear
{
    //set up the variables for the logger
    final String TAG = "Concept Logging";
    private ElapsedTime runtime = new ElapsedTime();
    private FileLogger fileLogger;
    private int debug = 1;

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

        dashboard.displayPrintf(1, "initRobot");
        fileLogger.writeEvent(3, "FileLogger Started");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        dashboard.clearDisplay();
        dashboard.displayPrintf(0, LABEL_WIDTH, "Text: ", "*** Robot Data ***");

        TOWR5291Toggle togglex = new TOWR5291Toggle(gamepad1.x);
        togglex.setDebounce(250);
        TOWR5291Toggle toggley = new TOWR5291Toggle(gamepad1.y);
        toggley.setDebounce(1000);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            dashboard.displayPrintf(1, LABEL_WIDTH, "X: ", "" + togglex.toggleState(gamepad1.x));
            dashboard.displayPrintf(2, LABEL_WIDTH, "Y: ", "" + toggley.toggleState(gamepad1.y));

        }

        if (fileLogger != null) {
            fileLogger.writeEvent(TAG, "Stopped");
            fileLogger.close();
            fileLogger = null;
        }
    }
}
