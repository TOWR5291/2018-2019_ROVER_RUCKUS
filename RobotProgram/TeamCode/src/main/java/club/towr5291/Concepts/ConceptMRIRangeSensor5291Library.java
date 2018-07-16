/*
Modern Robotics Range Sensor Example
Created 9/8/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 2.x Beta
Reuse permitted with credit where credit is due

Configuration:
I2cDevice on an Interface Module named "range" at the default address of 0x28 (0x14 7-bit)

This program can be run without a battery and Power Destitution Module.

For more information, visit modernroboticsedu.com.
Support is available by emailing support@modernroboticsinc.com.
*/

package club.towr5291.Concepts;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import club.towr5291.functions.FileLogger;
import club.towr5291.sensors.MRRangeSensor;

@TeleOp(name = "Concept Range Sensor Custom", group = "5291Concept")
@Disabled
public class ConceptMRIRangeSensor5291Library extends OpMode {
    private MRRangeSensor rangeSensor1;
    private MRRangeSensor rangeSensor2;

    //set up the variables for the logger
    final String TAG = "Concept Logging";
    private ElapsedTime runtime = new ElapsedTime();
    private FileLogger fileLogger;
    private int debug = 1;

    @Override
    public void init() {
        fileLogger = new FileLogger(runtime,1,true);
        fileLogger.open();
        fileLogger.setDebugLevel(1);
        fileLogger.write("Time,SysMS,Thread,Event,Desc");
        fileLogger.writeEvent(TAG, "Log Started");
        rangeSensor1 = hardwareMap.get(MRRangeSensor.class, "range1");
        MRRangeSensor.Parameters parameters = new MRRangeSensor.Parameters();
        parameters.I2CADDR = I2cAddr.create7bit(0x14);
        rangeSensor1.initialize(parameters);
        rangeSensor2 = hardwareMap.get(MRRangeSensor.class, "range2");
        MRRangeSensor.Parameters parameters2 = new MRRangeSensor.Parameters();
        parameters2.I2CADDR = I2cAddr.create7bit(0x18);
        rangeSensor2.initialize(parameters2);
    }

    @Override
    public void loop() {
        //telemetry.addLine("Runtime " + runtime);
        telemetry.addData("Distance1= ", rangeSensor1.getDistance(DistanceUnit.CM));
        telemetry.addData("Distance2= ", rangeSensor2.getDistance(DistanceUnit.CM));
        fileLogger.writeEvent(TAG, "Distance1= " + rangeSensor1.getDistance(DistanceUnit.CM) + " Distance2= " + rangeSensor2.getDistance(DistanceUnit.CM));
        telemetry.update();
    }

    @Override
    public void stop() {
        if (fileLogger != null) {
            fileLogger.writeEvent(TAG, "Stopped");
            fileLogger.close();
            fileLogger = null;
        }
    }

}