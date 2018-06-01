package club.towr5291.Concepts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import club.towr5291.functions.FileLogger;
import club.towr5291.libraries.VL53L0X;

/**
 * Created by Ian Haden on 5/24/2018.
 */

@TeleOp(name = "Concept VL5350X", group = "5291Concept")
public class ConceptVL5350X extends OpMode {

    //set up the variables for the logger
    final String TAG = "Concept VL5350X";
    private ElapsedTime runtime = new ElapsedTime();
    private FileLogger fileLogger;
    private int debug = 5;
    private int loop = 0;
    private VL53L0X VL53L0XSensor1;

    @Override
    public void init() {
        //start the log
        if (debug >= 1) {
            fileLogger = new FileLogger(runtime, debug, true);
            fileLogger.open();
            fileLogger.write("Time,SysMS,Thread,Event,Desc");
            fileLogger.writeEvent(1,TAG, "Log Started");
        }

        VL53L0XSensor1 = hardwareMap.get(VL53L0X.class, "VL53Sensor1");
        VL53L0XSensor1.init();
        VL53L0XSensor1.startContinuous();
    }

    @Override
    public void loop() {
        double distance;

        distance = VL53L0XSensor1.getDistanceContinousmm();
        //this will log only when debug is at level 3 or above
        fileLogger.writeEvent(3, TAG, "In Loop # " + loop + " - Distance # " + distance);
        telemetry.addLine("In Loop # " + loop + " - Distance # " + distance);
        telemetry.update();
        loop++;
    }

    @Override
    public void stop() {
        //stop the log
        if (fileLogger != null) {
            fileLogger.writeEvent(1, TAG, "Stopped");
            fileLogger.close();
            fileLogger = null;
        }
    }
}


