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

import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Looper;
import android.os.Message;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


// Written by Ian Haden
// Based of the single MR Range Sensor Code

@TeleOp(name = "Concept Range Sensor Dual Thread", group = "5291Concept")
@Disabled
public class ConceptMRIRangeSensorDualThread extends OpMode {

    public double mdblRangeSensor1 = 0;
    public double mdblRangeSensor2 = 0;
    Bundle mbundle = new Bundle();

    Runnable runnable1 = new Runnable() {
        //set up range sensor1
        byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable

        I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
        public static final int RANGE1_REG_START = 0x04; //Register to start reading
        public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read

        public I2cDevice RANGE1;
        public I2cDeviceSynch RANGE1Reader;
        private ElapsedTime threadRuntime = new ElapsedTime();

        public void run() {
            double dblRangeSensor1;
            double dblThreadLoopTime;
            RANGE1 = hardwareMap.i2cDevice.get("range1");
            RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
            RANGE1Reader.engage();

            threadRuntime.reset();

            while (true) {
                dblThreadLoopTime = threadRuntime.milliseconds();
                Bundle bundle = new Bundle();
                Message msg = new Message();
                range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
                dblRangeSensor1 = getDistance(range1Cache[0] & 0xFF, range1Cache[1] & 0xFF, DistanceUnit.CM);
                bundle.putString("sensor1", dblRangeSensor1 + "");
                msg.setData(bundle);
                handler1.sendMessage(msg);
                //lets just loop and do nothing for a bit
                while ((dblThreadLoopTime + 30) > threadRuntime.milliseconds()) {
                    //lets just sit here
                }

                if (Thread.interrupted()) {
                    // We've been interrupted: no more crunching.
                    return;
                }
            }
        }
    };

    Runnable runnable2 = new Runnable() {
        //set up ragesensor 2
        byte[] range2Cache; //The read will return an array of bytes. They are stored in this variable

        I2cAddr RANGE2ADDRESS = new I2cAddr(0x18); //Default I2C address for MR Range (7-bit)
        public static final int RANGE2_REG_START = 0x04; //Register to start reading
        public static final int RANGE2_READ_LENGTH = 2; //Number of byte to read

        public I2cDevice RANGE2;
        public I2cDeviceSynch RANGE2Reader;
        private ElapsedTime threadRuntime = new ElapsedTime();

        public void run() {
            double dblRangeSensor2;
            double dblThreadLoopTime;

            RANGE2 = hardwareMap.i2cDevice.get("range2");
            RANGE2Reader = new I2cDeviceSynchImpl(RANGE2, RANGE2ADDRESS, false);
            RANGE2Reader.engage();

            threadRuntime.reset();

            while (true) {
                dblThreadLoopTime = threadRuntime.milliseconds();
                Bundle bundle = new Bundle();
                Message msg = new Message();
                range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
                dblRangeSensor2 = getDistance(range2Cache[0] & 0xFF, range2Cache[1] & 0xFF, DistanceUnit.CM);
                bundle.putString("sensor2", dblRangeSensor2 + "");
                msg.setData(bundle);
                handler2.sendMessage(msg);
                while ((dblThreadLoopTime + 30) > threadRuntime.milliseconds()) {
                    //lets just sit here
                }
                if (Thread.interrupted()) {
                    // We've been interrupted: no more crunching.
                    return;
                }
            }
        }
    };


    Handler handler1 = new Handler(Looper.getMainLooper()) {
        @Override
        public void handleMessage(Message msg) {
            Bundle bundle = new Bundle();
            bundle = msg.getData();
            mdblRangeSensor1 = Double.parseDouble(bundle.getString("sensor1"));
            //Log.d("Message1", "Received: " + mdblRangeSensor1);
        }
    };

    Handler handler2 = new Handler(Looper.getMainLooper()) {
        @Override
        public void handleMessage(Message msg) {
            Bundle bundle = new Bundle();
            bundle = msg.getData();
            mdblRangeSensor2 = Double.parseDouble(bundle.getString("sensor2"));
            //Log.d("Message2", "Received: " + mdblRangeSensor2);
        }
    };

    Thread mythread1 = new Thread(runnable1);
    Thread mythread2 = new Thread(runnable2);

    @Override
    public void init() {

        mythread1.start();
        mythread2.start();
    }

    @Override
    public void loop() {
        telemetry.addData("R1 Distance ", mdblRangeSensor1);
        telemetry.addData("R2 Distance ", mdblRangeSensor2);
    }

    @Override
    public void stop() {
        mythread1.interrupt();
        mythread2.interrupt();
    }

    /**
     * Converts a reading of the optical sensor into centimeters. This computation
     * could be adjusted by altering the numeric parameters, or by providing an alternate
     * calculation in a subclass.
     */
    private double cmFromOptical(int opticalReading)
    {
        double pParam = -1.02001;
        double qParam = 0.00311326;
        double rParam = -8.39366;
        int    sParam = 10;

        if (opticalReading < sParam)
            return 0;
        else
            return pParam * Math.log(qParam * (rParam + opticalReading));
    }
    private int cmUltrasonic(int rawUS)
    {
        return rawUS;
    }

    private double cmOptical(int rawOptical)
    {
        return cmFromOptical(rawOptical);
    }

    public double getDistance(int rawUS, int rawOptical, DistanceUnit unit)
    {
        double cmOptical = cmOptical(rawOptical);
        double cm        = cmOptical > 0 ? cmOptical : cmUltrasonic(rawUS);
        return unit.fromUnit(DistanceUnit.CM, cm);
    }

}