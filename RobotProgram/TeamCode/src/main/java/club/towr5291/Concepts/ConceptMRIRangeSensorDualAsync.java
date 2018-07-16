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

import android.content.Context;
import android.os.AsyncTask;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Written by Ian Haden
// Based of the single MR Range Sensor Code

@TeleOp(name = "Concept Range Sensor Dual Async", group = "5291Concept")
@Disabled
public class ConceptMRIRangeSensorDualAsync extends OpMode {

    //set up range sensor1
    byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable

    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    public static final int RANGE1_REG_START = 0x04; //Register to start reading
    public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read

    public I2cDevice RANGE1;
    public I2cDeviceSynch RANGE1Reader;

    //set up ragesensor 2
    byte[] range2Cache; //The read will return an array of bytes. They are stored in this variable

    I2cAddr RANGE2ADDRESS = new I2cAddr(0x18); //Default I2C address for MR Range (7-bit)
    public static final int RANGE2_REG_START = 0x04; //Register to start reading
    public static final int RANGE2_READ_LENGTH = 2; //Number of byte to read

    public I2cDevice RANGE2;
    public I2cDeviceSynch RANGE2Reader;

    public double mdblRangeSensor1;
    public double mdblRangeSensor2;

    ConceptMRIRangeSensorDualThreadedAsync backgroundTask = new ConceptMRIRangeSensorDualThreadedAsync();

    @Override
    public void init() {

        backgroundTask.execute();
    }

    @Override
    public void loop() {

        telemetry.addData("R1 Ultra Sonic ", range1Cache[0] & 0xFF);
        telemetry.addData("R1 ODS ", range1Cache[1] & 0xFF);
        telemetry.addData("R1 Distance ", mdblRangeSensor1);
        telemetry.addData("R2 Ultra Sonic ", range2Cache[0] & 0xFF);
        telemetry.addData("R2 ODS ", range2Cache[1] & 0xFF);
        telemetry.addData("R2 Distance ", mdblRangeSensor2);
    }

    @Override
    public void stop() {
        backgroundTask.cancel(true);
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

    public class ConceptMRIRangeSensorDualThreadedAsync extends AsyncTask<Void,Void,Void> {
        Context context;
        //int progressValue, result;
        int i = 0;

        @Override
        protected Void doInBackground(Void... params) {
            synchronized (this) {
                while (true) {
                    range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
                    range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
                    mdblRangeSensor1 = getDistance(range1Cache[0] & 0xFF, range1Cache[1] & 0xFF, DistanceUnit.CM);
                    mdblRangeSensor2 = getDistance(range2Cache[0] & 0xFF, range2Cache[1] & 0xFF, DistanceUnit.CM);
                }
            }

        }

        @Override
        protected void onPreExecute() {
            RANGE1 = hardwareMap.i2cDevice.get("range1");
            RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
            RANGE1Reader.engage();
            RANGE2 = hardwareMap.i2cDevice.get("range2");
            RANGE2Reader = new I2cDeviceSynchImpl(RANGE2, RANGE2ADDRESS, false);
            RANGE2Reader.engage();
        }

        @Override
        protected void onPostExecute(Void aVoid) {
        }

        @Override
        protected void onProgressUpdate(Void... values) {
        }

        @Override
        protected void onCancelled(Void aVoid) {
        }

        @Override
        protected void onCancelled() {
        }


    }




}