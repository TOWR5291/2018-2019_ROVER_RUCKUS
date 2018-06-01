package club.towr5291.Concepts;

/**
 * Created by LZTDD0 on 1/17/2017.
 */


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.util.ElapsedTime;

import club.towr5291.functions.FileLogger;
import club.towr5291.libraries.LibraryAdafruitIMU;

/**
 * Created by Owner on 8/31/2015.
 */
public class ConceptAdafruitIMUAndrew extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private FileLogger fileLogger;
    LibraryAdafruitIMU adafruitBNO055;

    //The following arrays contain both the Euler angles reported by the IMU (indices = 0) AND the
    // Tait-Bryan angles calculated from the 4 components of the quaternion vector (indices = 1)
    volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];

    volatile double[] rollAngle2 = new double[2], pitchAngle2 = new double[2], yawAngle2 = new double[2];
    volatile double[] rollChange = new double[2], pitchChange = new double[2], yawChange = new double[2];
    long systemTime;//Relevant values of System.nanoTime


    /************************************************************************************************
     * The following method was introduced in the 3 August 2015 FTC SDK beta release and it runs
     * before "start" runs.
     */
    @Override
    public void init() {
        fileLogger = new FileLogger(runtime);
        fileLogger.open();
        fileLogger.write("Time,SysMS,Thread,Event,Desc");
        fileLogger.writeEvent(TAG, "Log Started");

        systemTime = System.nanoTime();
        try {
            adafruitBNO055 = new LibraryAdafruitIMU(hardwareMap, "bno055"

                    //The following was required when the definition of the "I2cDevice" class was incomplete.
                    //, "cdim", 5

                    , (byte)(LibraryAdafruitIMU.BNO055_ADDRESS_A * 2)//By convention the FTC SDK always does 8-bit I2C bus
                    //addressing
                    , (byte)LibraryAdafruitIMU.OPERATION_MODE_IMU);
        } catch (RobotCoreException e){
            fileLogger.writeEvent("FtcRobotController", "Exception: " + e.getMessage());
        }
        fileLogger.writeEvent("FtcRobotController", "IMU Init method finished in: "
                + (-(systemTime - (systemTime = System.nanoTime()))) + " ns.");
        //ADDRESS_B is the "standard" I2C bus address for the Bosch BNO055 (IMU data sheet, p. 90).
        //BUT DAVID PIERCE, MENTOR OF TEAM 8886, HAS EXAMINED THE SCHEMATIC FOR THE ADAFRUIT BOARD ON
        //WHICH THE IMU CHIP IS MOUNTED. SINCE THE SCHEMATIC SHOWS THAT THE COM3 PIN IS PULLED LOW,
        //ADDRESS_A IS THE IMU'S OPERATIVE I2C BUS ADDRESS
        //IMU is an appropriate operational mode for FTC competitions. (See the IMU datasheet, Table
        // 3-3, p.20 and Table 3-5, p.21.)
    }

    /************************************************************************************************
     * Code to run when the op mode is first enabled goes here
     * @see OpMode#start()
     */
    @Override
    public void start() {
        /*
      	* Use the hardwareMap to get the dc motors, servos and other sensors by name. Note
      	* that the names of the devices must match the names used when you
      	* configured your robot and created the configuration file. The hardware map
      	* for this OpMode is not initialized until the OpModeManager's "startActiveOpMode" method
      	* runs.
    		*/
        systemTime = System.nanoTime();
        adafruitBNO055.startIMU();//Set up the IMU as needed for a continual stream of I2C reads.
        fileLogger.writeEvent("FtcRobotController", "IMU Start method finished in: "
                + (-(systemTime - (systemTime = System.nanoTime()))) + " ns.");
    }

    /***********************************************************************************************
     * This method will be called repeatedly in a loop
     * @see OpMode#loop()
     * NOTE: BECAUSE THIS "loop" METHOD IS PART OF THE OVERALL OpMode/EventLoop/ReadWriteRunnable
     * MECHANISM, ALL THAT THIS METHOD WILL BE USED FOR, IN AUTONOMOUS MODE, IS TO:
     * 1. READ SENSORS AND ENCODERS AND STORE THEIR VALUES IN SHARED VARIABLES
     * 2. WRITE MOTOR POWER AND CONTROL VALUES STORED IN SHARED VARIABLES BY "WORKER" THREADS, AND
     * 3. SEND TELELMETRY DATA TO THE DRIVER STATION
     * THIS "loop" METHOD IS THE ONLY ONE THAT "TOUCHES" ANY SENSOR OR MOTOR HARDWARE.
     */
    @Override
    public void loop() {
        //Log.i("FtcRobotController", "Loop method starting at: " +
        //      -(systemTime - (systemTime = System.nanoTime())) + " since last loop start.");

        // write the values computed by the "worker" threads to the motors (if any)

        //Read the encoder values that the "worker" threads will use in their computations
        if(gamepad1.left_bumper) {
            adafruitBNO055.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
		/*
		 * Send whatever telemetry data you want back to driver station.
		 */
        } else if(gamepad1.right_bumper) {
            adafruitBNO055.getIMUGyroAngles(rollAngle2, pitchAngle2, yawAngle2);
        }

        calculateChange();

        //telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("Headings(yaw): ",
                String.format("Euler= %4.5f, Quaternion calculated= %4.5f", yawAngle[0], yawAngle[1]));
        telemetry.addData("Pitches: ",
                String.format("Euler= %4.5f, Quaternion calculated= %4.5f", pitchAngle[0], pitchAngle[1]));
        telemetry.addData("Headings(yaw)2: ",
                String.format("Euler= %4.5f, Quaternion calculated= %4.5f", yawAngle2[0], yawAngle2[1]));
        telemetry.addData("Pitches: 2",
                String.format("Euler= %4.5f, Quaternion calculated= %4.5f", pitchAngle[0], pitchAngle2[1]));
        telemetry.addData("Headings(yaw)Change: ",
                String.format("Euler= %4.5f, Quaternion calculated= %4.5f", yawChange[0], yawChange[1]));
        telemetry.addData("Pitches Change: ",
                String.format("Euler= %4.5f, Quaternion calculated= %4.5f", pitchChange[0], pitchChange[1]));
        telemetry.addData("Max I2C read interval: ",
                String.format("%4.4f ms. Average interval: %4.4f ms.", adafruitBNO055.maxReadInterval
                        , adafruitBNO055.avgReadInterval));

    }

    /*
    * Code to run when the op mode is first disabled goes here
    * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
    */
    @Override
    public void stop() {
        //When the FTC Driver Station's "Start with Timer" button commands autonomous mode to start,
        //then stop after 30 seconds, stop the motors immediately!
        //Following this method, the underlying FTC system will call a "stop" routine of its own
        systemTime = System.nanoTime();
        fileLogger.writeEvent("FtcRobotController", "IMU Stop method finished in: "
                + (-(systemTime - (systemTime = System.nanoTime()))) + " ns.");

        if (fileLogger != null) {
            fileLogger.writeEvent(TAG, "Stopped");
            fileLogger.close();
            fileLogger = null;
        }
    }

    //set up the variables for the logger
    final String TAG = "Concept Logging";

    public void calculateChange() {
        rollChange[0] = rollAngle2[0] - rollAngle[0];
        rollChange[1] = rollAngle2[1] - rollAngle[1];
        pitchChange[0] = pitchAngle2[0] - pitchAngle[0];
        pitchChange[1] = pitchAngle2[1] - pitchAngle[1];
        yawChange[0] = yawAngle2[0] - yawAngle[0];
        yawChange[1] = yawAngle2[1] - yawAngle[1];
    }
}
