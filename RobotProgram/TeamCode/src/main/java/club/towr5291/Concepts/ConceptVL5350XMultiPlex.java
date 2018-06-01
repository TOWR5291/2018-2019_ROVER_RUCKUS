package club.towr5291.Concepts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import club.towr5291.functions.FileLogger;
import club.towr5291.libraries.VL53L0X;
import club.towr5291.libraries.VL53L0XMultiPlex;
import club.towr5291.opmodes.OpModeMasterLinear;

/**
 * Created by Ian Haden on 5/24/2018.
 */

@TeleOp(name = "Concept VL5350X MP", group = "5291Concept")
public class ConceptVL5350XMultiPlex extends OpModeMasterLinear {

    //set up the variables for the logger
    final String TAG = "Concept VL5350XMP";
    private ElapsedTime runtime = new ElapsedTime();
    private FileLogger fileLogger;
    private int debug = 5;
    private int loop = 0;
    int[] ports = {0,1,2,3,4};
    private VL53L0XMultiPlex VL53L0XSensor;
    private double[] distance = {0,0,0,0,0,0,0,0};


    @Override
    public void runOpMode() throws InterruptedException {
        //start the log
        if (debug >= 1) {
            fileLogger = new FileLogger(runtime, debug, true);
            fileLogger.open();
            fileLogger.write("Time,SysMS,Thread,Event,Desc");
            fileLogger.writeEvent(1,TAG, "Log Started");
        }
        VL53L0XSensor = new VL53L0XMultiPlex(hardwareMap,"mux");
        for (int i = 0; i < ports.length ; i++) {
            telemetry.addLine("Sensor " + i + " Starting on port # " + ports[i]);
            telemetry.update();
            VL53L0XSensor.VL53L0XMultiPlexInitPort("VL53Sensor1", ports[i]);
        }
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //the main loop.  this is where the action happens
        while (opModeIsActive()) {
            for (int i = 0; i < ports.length; i++) {
                distance[i] = VL53L0XSensor.getRangemm(i);
                fileLogger.writeEvent(3, TAG, "In Loop # " + loop + " - Distance # " + distance[i]);
                telemetry.addLine(i + "=In Loop # " + loop + " - Distance # " + distance[i]);
            }
            distance[0] = VL53L0XSensor.getRangemm(0);
            //this will log only when debug is at level 3 or above
            //fileLogger.writeEvent(3, TAG, "In Loop # " + loop + " - Distance # " + distance);
            //telemetry.addLine("In Loop # " + loop + " - Distance # " + distance);
            telemetry.update();
            loop++;

        }

        //stop the log
        if (fileLogger != null) {
            fileLogger.writeEvent(1, TAG, "Stopped");
            fileLogger.close();
            fileLogger = null;
        }

    }


}


