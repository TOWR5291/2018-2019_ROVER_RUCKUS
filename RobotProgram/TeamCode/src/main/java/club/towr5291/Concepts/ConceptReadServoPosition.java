package club.towr5291.Concepts;

import android.widget.TextView;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import club.towr5291.R;
import club.towr5291.functions.FileLogger;
import club.towr5291.functions.TOWR5291Tick;
import club.towr5291.functions.TOWR5291Utils;
import club.towr5291.libraries.robotConfig;
import club.towr5291.opmodes.OpModeMasterLinear;
import hallib.HalDashboard;


/**
 * Created by Ian Haden TOWR5291 on 7/27/2018.
 * This OpMode is to read the positions of servos to get the home positions
 * and maximum and minimum travels
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

@TeleOp(name = "Concept Servo", group = "5291Concept")
//@Disabled
public class ConceptReadServoPosition extends OpModeMasterLinear {

    //set up the variables for the logger
    final String TAG = "Concept Read Servo";
    private ElapsedTime runtime = new ElapsedTime();
    private FileLogger fileLogger;
    private int debug = 3;

    private Servo leftEYEServo;
    private Servo rightEYEServo;

    private static HalDashboard dashboard = null;

    public static HalDashboard getDashboard()
    {
        return dashboard;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = HalDashboard.createInstance(telemetry);
        dashboard = HalDashboard.getInstance();

        fileLogger = new FileLogger(runtime, debug, true);
        fileLogger.open();
        fileLogger.write("Time,SysMS,Thread,Event,Desc");
        fileLogger.writeEvent(1,TAG, "Log Started");

        FtcRobotControllerActivity activity = (FtcRobotControllerActivity) hardwareMap.appContext;

        dashboard.setTextView((TextView) activity.findViewById(R.id.textOpMode));
        dashboard.displayPrintf(0, 200, "Text: ", "*** Robot Data ***");
        dashboard.displayPrintf(1, "Waiting For Start!");

        rightEYEServo = hardwareMap.servo.get(robotConfig.eyeServos.rightEYE.toString());
        leftEYEServo = hardwareMap.servo.get(robotConfig.eyeServos.leftEYE.toString());

        TOWR5291Tick ticker1 = new TOWR5291Tick();
        TOWR5291Tick ticker2 = new TOWR5291Tick();

        ticker1.setTickIncrement(1);
        ticker1.setTickMax(180);
        ticker1.setTickMin(0);
        ticker1.setTickValue(90);
        ticker1.setRollOver(false);

        ticker2.setTickIncrement(1);
        ticker2.setTickMax(180);
        ticker2.setTickMin(0);
        ticker2.setTickValue(90);
        ticker2.setRollOver(false);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        dashboard.displayPrintf(1, "Running!");

        //the main loop.  this is where the action happens
        while (opModeIsActive()) {//this will log only when debug is at level 3 or above
            ticker1.incrementTick(gamepad1.x);
            ticker1.decrementTick(gamepad1.a);
            ticker2.incrementTick(gamepad1.y);
            ticker2.decrementTick(gamepad1.b);
            TOWR5291Utils.moveServo(rightEYEServo, ticker1.getTickCurrValue(), (double)0, (double)180);
            TOWR5291Utils.moveServo(leftEYEServo, ticker2.getTickCurrValue(), (double)0, (double)180);
            dashboard.displayPrintf(2, "Position 1 = " + ticker1.getTickCurrValue());
            dashboard.displayPrintf(3, "Position 2 = " + ticker2.getTickCurrValue());
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

