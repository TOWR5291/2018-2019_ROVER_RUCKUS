package club.towr5291.sensors;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDeviceWithParameters;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by ianhaden on 3/6/18.
 */


@SuppressWarnings({"WeakerAccess", "unused"}) // Ignore access and unused warnings
// Both driver classes cannot register the sensor at the same time. One driver should have the
// sensor registered, and the other should be commented out
@I2cSensor(name = "MR Range Sensor I2C 5291", description = "Modern Robotics Range Sensor New Address", xmlTag = "MRRANGE5291")
public class MRRangeSensor extends I2cDeviceSynchDeviceWithParameters<I2cDeviceSynch, MRRangeSensor.Parameters> {


    ////////////////////////////////////////////////////////////////////////////////////////////////
    // User Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////


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

    private double getDistance(int rawUS, int rawOptical, DistanceUnit unit)
    {
        double cmOptical = cmOptical(rawOptical);
        double cm        = cmOptical > 0 ? cmOptical : cmUltrasonic(rawUS);
        return unit.fromUnit(DistanceUnit.CM, cm);
    }

    public double getDistance(DistanceUnit unit)
    {
        return getDistance(getUltraSonicRaw(), getOpticalRaw(), unit);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Raw Register Reads
    ////////////////////////////////////////////////////////////////////////////////////////////////
    private int getUltraSonicRaw()
    {
        return readReg(Register.RAW_ULTRA_SONIC) & 0xFF;
    }

    private int getOpticalRaw()
    {
        return readReg(Register.RAW_OPTICAL) & 0xFF;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Read and Write Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////

    protected short readShort(Register reg)
    {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }

    protected int readReg(Register reg) {
        byte array[] = this.deviceClient.read(reg.bVal, 1);
        return TypeConversion.unsignedByteToInt(array[0]);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Registers and Config Settings
    ////////////////////////////////////////////////////////////////////////////////////////////////
    public enum Register
    {
        FIRST(0x04),
        RAW_ULTRA_SONIC(0x04),
        RAW_OPTICAL(0x05),
        LAST(RAW_OPTICAL.bVal);

        public int bVal;

        Register(int bVal)
        {
            this.bVal = bVal;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Construction and Initialization
    ////////////////////////////////////////////////////////////////////////////////////////////////

    public MRRangeSensor(I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true, new Parameters());
        this.setOptimalReadWindow();

        super.registerArmingStateCallback(false); // Deals with USB cables getting unplugged
        // Sensor starts off disengaged so we can change things like I2C address. Need to engage
        this.deviceClient.engage();
    }

    public static class Parameters implements Cloneable
    {

        // All settings available
        public I2cAddr I2CADDR = I2cAddr.create7bit(0x14);

        public Parameters clone()
        {
            try
            {
                return (Parameters) super.clone();
            }
            catch(CloneNotSupportedException e)
            {
                throw new RuntimeException("Internal Error: Parameters not cloneable");
            }
        }
    }

    protected void setOptimalReadWindow()
    {
        // Sensor registers are read repeatedly and stored in a register. This method specifies the
        // registers and repeat read mode
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.FIRST.bVal,
                Register.LAST.bVal - Register.FIRST.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }



    @Override
    protected synchronized boolean internalInitialize(@NonNull Parameters params)
    {
        this.parameters = params.clone();
        deviceClient.setI2cAddress(params.I2CADDR);
        return true;
    }

    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.ModernRobotics;
    }

    @Override
    public String getDeviceName()
    {
        return "MR Range Sensor Custom I2C Address";
    }



}
