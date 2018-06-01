package club.towr5291.Concepts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.util.ElapsedTime;

import club.towr5291.functions.Constants;
import club.towr5291.functions.FileLogger;
import club.towr5291.libraries.LEDControl;
import club.towr5291.opmodes.OpModeMasterLinear;

/**
 * Created by ianhaden on 26/5/18.
 */

@TeleOp(name = "Concept LEDs", group = "5291Concept")
public class ConceptLEDs extends OpModeMasterLinear {

    //set up the variables for the logger
    final String TAG = "Concept LEDs Demo";
    private ElapsedTime runtime = new ElapsedTime();
    private FileLogger fileLogger;
    private int debug = 3;
    private int loop = 0;
    private LEDControl LEDs;
    private Constants.LEDState LEDStatus = Constants.LEDState.STATE_NULL;


    @Override
    public void runOpMode() throws InterruptedException {
        fileLogger = new FileLogger(runtime, debug, true);
        fileLogger.open();
        fileLogger.write("Time,SysMS,Thread,Event,Desc");
        fileLogger.writeEvent(1,TAG, "Log Started");
        LEDs = new LEDControl(hardwareMap, "lg", "lr", "lb", "rg", "rr", "rb");
        //LEDs.setLEDControlDemoMode(true);
        //LEDs.setLEDControlObjectColour(Constants.ObjectColours.UNKNOWN);
        //LEDs.setLEDControlAlliance("Red");
        LEDs.setLEDControlDemoMode(false);
        LEDs.setLEDColour(Constants.LEDColours.LED_MAGENTA);
        LEDStatus = Constants.LEDState.STATE_FLASHC_COLOUR;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //the main loop.  this is where the action happens
        while (opModeIsActive()) {
            LEDStatus = LEDs.LEDControlUpdate(LEDStatus);
            //this will log only when debug is at level 3 or above
            fileLogger.writeEvent(3, TAG, "Runtime " + runtime + " - LEDStatus " + LEDStatus);
            telemetry.addLine("runtime " + runtime + " - LEDStatus " + LEDStatus);
            telemetry.update();
        }
        //stop the log
        if (fileLogger != null) {
            fileLogger.writeEvent(1, TAG, "Stopped");
            fileLogger.close();
            fileLogger = null;
        }
    }
}

