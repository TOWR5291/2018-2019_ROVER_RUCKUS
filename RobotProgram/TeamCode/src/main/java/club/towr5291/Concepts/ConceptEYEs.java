package club.towr5291.Concepts;

import android.widget.TextView;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import club.towr5291.R;
import club.towr5291.functions.Constants;
import club.towr5291.functions.FileLogger;
import club.towr5291.libraries.TOWR5291EYEControl;
import club.towr5291.libraries.TOWRDashBoard;
import club.towr5291.opmodes.OpModeMasterLinear;

/**
 * Created by Ian Haden TOWR5291 on 7/27/2018.
 * This OpMode is to demonstrate how to use the EYE Class
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 *
 * Modification history
 * Edited by:
 * Ian Haden 07/27/2018 -> Initial creation
 */

@TeleOp(name = "Concept EYEs", group = "5291Concept")
//@Disabled
public class ConceptEYEs extends OpModeMasterLinear {

    //set up the variables for the logger
    final String TAG = "Concept EYEs Demo";
    private ElapsedTime runtime = new ElapsedTime();
    private FileLogger fileLogger;
    private int debug = 3;
    private int loop = 0;
    private TOWR5291EYEControl EYEs;
    private Constants.EYEState EYEStatus = Constants.EYEState.STATE_BLINK;

    private static TOWRDashBoard dashboard = null;
    public static TOWRDashBoard getDashboard()
    {
        return dashboard;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        dashboard = TOWRDashBoard.createInstance(telemetry);
        dashboard = TOWRDashBoard.getInstance();

        fileLogger = new FileLogger(runtime, debug, true);
        fileLogger.open();
        fileLogger.write("Time,SysMS,Thread,Event,Desc");
        fileLogger.writeEvent(1,TAG, "Log Started");
        EYEs = new TOWR5291EYEControl(hardwareMap);
        //EYEs.setEYEControlDemoMode(true);
        //EYEs.setEYEControlAlliance("Red");
        EYEs.setEYEControlDemoMode(true);

        FtcRobotControllerActivity activity = (FtcRobotControllerActivity) hardwareMap.appContext;

        dashboard.setTextView((TextView) activity.findViewById(R.id.textOpMode));
        dashboard.displayPrintf(0, 200, "Text: ", "*** Robot Data ***");
        dashboard.displayPrintf(1, "Waiting For Start!");
        EYEStatus = Constants.EYEState.STATE_BLINK;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        dashboard.displayPrintf(1, "Running!");

        //the main loop.  this is where the action happens
        while (opModeIsActive()) {
            EYEStatus = EYEs.EYEControlUpdate(EYEStatus);
            //this will log only when debug is at level 3 or above
            fileLogger.writeEvent(3, TAG, "Runtime " + runtime + " - EYEStatus " + EYEStatus.toString());
            dashboard.displayPrintf(2, "Status = " + EYEStatus.toString());
        }
        dashboard.displayPrintf(1, "Stopped!");
        //stop the log
        if (fileLogger != null) {
            fileLogger.writeEvent(1, TAG, "Stopped");
            fileLogger.close();
            fileLogger = null;
        }
    }
}

