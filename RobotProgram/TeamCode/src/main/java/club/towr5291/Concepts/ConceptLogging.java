package club.towr5291.Concepts;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;

import club.towr5291.functions.FileLogger;

/**
 * Created by Ian Haden on 11/7/2016.
 */

@TeleOp(name = "Concept Logging", group = "5291Concept")
@Disabled
public class ConceptLogging extends OpMode {

    //set up the variables for the logger
    final String TAG = "Concept Logging";
    private ElapsedTime runtime = new ElapsedTime();
    private FileLogger fileLogger;
    private int debug = 1;

    private int loop = 0;


    @Override
    public void init() {
        //start the log
        if (debug >= 1) {
            fileLogger = new FileLogger(runtime);
            fileLogger.open();
            fileLogger.write("Time,SysMS,Thread,Event,Desc");
            fileLogger.writeEvent(TAG, "Log Started");
        }
    }

    @Override
    public void loop() {

        //this will log only when debug is at level 3 or above
        if (debug >= 3) {
            fileLogger.writeEvent(TAG, "In Loop # " + loop);
        }
        loop++;
    }

    @Override
    public void stop() {
        //stop the log
        if (debug >= 1) {
            if (fileLogger != null) {
                fileLogger.writeEvent(TAG, "Stopped");
                fileLogger.close();
                fileLogger = null;
            }
        }
    }
}


