package club.towr5291.libraries;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import club.towr5291.libraries.VL53L0X;

/**
 * Created by ianhaden on 26/5/18.
 */

public class VL53L0XMultiPlex {

    //***********************************************************************
    // Default I2C address for multiplexer. The address can be changed to any
    // value from 0x70 to 0x77, so this line would need to be changed if a
    // non-default address is to be used.
    //***********************************************************************
    static final I2cAddr MUX_ADDRESS = new I2cAddr(0x70);
    private I2cDevice mux;
    private I2cDeviceSynch muxReader;
    private VL53L0X VL53L0XSensors;
    HardwareMap hardwareMap;
    //array containing which ports are being used
    //example, if port 1 and 3 are used {1,3}

    public VL53L0XMultiPlex(HardwareMap hardwareMapIn, String muxName) {

            //get the id from the controller using the multiplexer name
            // set up the I2C and engage
        hardwareMap = hardwareMapIn;
        mux = hardwareMap.i2cDevice.get(muxName);
        muxReader = new I2cDeviceSynchImpl(mux, MUX_ADDRESS, false);
        muxReader.engage();

//        // Loop over the ports activating each sensor
//        for (int i = 0; i < ports.length; i++) {
//            // Write to given output port on the multiplexer
//            muxReader.write8(0x0, 1 << ports[i]);
//            VL53L0XSensors = hardwareMap.get(VL53L0X.class, sensorName);
//            VL53L0XSensors.init();
//            VL53L0XSensors.startContinuous();
//        }
    }

    public void VL53L0XMultiPlexInitPort(String sensorName, int port) {


        // Write to given output port on the multiplexer
        muxReader.write8(0x0, 1 << port);
        VL53L0XSensors = hardwareMap.get(VL53L0X.class, sensorName);
        VL53L0XSensors.init();
        VL53L0XSensors.startContinuous();

    }

    /**
     * Retrieve the color read by the given color sensor
     *
     * @param port Port on multiplexer of given color sensor
     * @return Double of the distance
     */
     public double getRangemm(int port) {
        // Write to I2C port on the multiplexer
        muxReader.write8(0x0, 1 << port);
        // Read range
        return VL53L0XSensors.getDistanceContinousmm();
    }
    public double getRangecm(int port) {
        return getRangemm(port) / 10;
    }
    public double getRangein(int port) {
        return getRangecm(port) / 2.54;
    }
}
