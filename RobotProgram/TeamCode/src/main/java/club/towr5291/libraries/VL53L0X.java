package club.towr5291.libraries;

//import android.util.Log;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.TypeConversion;

/*
 *
 * This version of the driver does not make use of the I2C device with parameters. This means the
 * settings for the configuration register are hard coded and cannot be changed by the user, nor can
 * they be different for each OpMode.
 */

/**
 * Created by ianhaden on 22/5/2018.
 * FTC team 5291 The Oxford Wilcats Robotics.
 *
 * Most of the functionality has bee converted from the pololu arduino library
 * https://github.com/pololu/vl53l0x-arduino/blob/master/VL53L0X.cpp
 *
 * I seem to be getting good results with a refresh of anout 50-60 ms using the
 * modern robotics cdim.  This has not been tested on the controller yet as of 26/5/2018
 *
 * I converted the arduino library using the guide from FTC
 * https://github.com/ftctechnh/ftc_app/wiki/Writing-an-I2C-Driver by Andryw Wade
 *
 // Most of the functionality of this library is based on the VL53L0X API
 // provided by ST (STSW-IMG005), and some of the explanatory comments are quoted
 // or paraphrased from the API source code, API user manual (UM2039), and the
 // VL53L0X datasheet.
 */

@SuppressWarnings({"WeakerAccess", "unused"}) // Ignore access and unused warnings
// Both driver classes cannot register the sensor at the same time. One driver should have the
// sensor registered, and the other should be commented out
@I2cSensor(name = "VL53L0X Light Ranging Sensor", description = "Light Ranging Sensor From ST Microelectronics", xmlTag = "VL53L0X")

public class VL53L0X extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    private int stop_variable;
    private long timeout_start_ms, io_timeout;
    private int spad_count;
    private int spad_type_is_aperture;
    private int measurement_timing_budget_us;
    private boolean did_timeout;

    class SequenceStepEnables {
        boolean tcc;
        boolean msrc;
        boolean dss;
        boolean pre_range;
        boolean final_range;
    };

    class SequenceStepTimeouts {
        int pre_range_vcsel_period_pclks;
        int final_range_vcsel_period_pclks;
        int msrc_dss_tcc_mclks;
        int pre_range_mclks;
        int final_range_mclks;
        int msrc_dss_tcc_us;
        int pre_range_us;
        int final_range_us;
    };

    enum vcselPeriodType {VcselPeriodPreRange, VcselPeriodFinalRange};

    private boolean convertToBoolean(String value) {
        boolean returnValue = false;
        if ("1".equalsIgnoreCase(value) || "yes".equalsIgnoreCase(value) ||
                "true".equalsIgnoreCase(value) || "on".equalsIgnoreCase(value))
            returnValue = true;
        return returnValue;
    }

    private boolean convertToBoolean(int value) {
        boolean returnValue = false;
        if (value == 1)
            returnValue = true;
        return returnValue;
    }


    private void setTimeout(int timeout) {
        io_timeout = timeout;
    }

    private int getTimeout() {
        return (int) (io_timeout);
    }

    // Record the current time to check an upcoming timeout against
    private void startTimeout() {
        timeout_start_ms = System.currentTimeMillis();
    }

    private void startTimeout(int timeout) {
        setTimeout(timeout);
        timeout_start_ms = System.currentTimeMillis();
    }

    // Check if timeout is enabled (set to nonzero value) and has expired
    private boolean checkTimeoutExpired() {
        return (io_timeout > 0 && (System.currentTimeMillis() - timeout_start_ms) > io_timeout);
    }

    // Did a timeout occur in one of the read functions since the last call to
    // timeoutOccurred()?
    public boolean timeoutOccurred()
    {
        boolean tmp = did_timeout;
        did_timeout = false;
        return tmp;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // User Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////
    public void startContinuous() {
        startContinuous(0);
    }

    //***************************************************************************
    // put the sensor into continous reading mode
    // the sensor will continue to read range
    // need to use the get continous functions to get the results of each read
    // takes about 60ms for each reading
    //***************************************************************************
       public void startContinuous(int period_ms) {
        writeReg(0x80, 0x01);
        writeReg(0xFF, 0x01);
        writeReg(0x00, 0x00);
        writeReg(0x91, stop_variable);
        writeReg(0x00, 0x01);
        writeReg(0xFF, 0x00);
        writeReg(0x80, 0x00);

        if (period_ms != 0) {
            // continuous timed mode
            // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin
            int osc_calibrate_val = readReg16(Register.OSC_CALIBRATE_VAL);
            if (osc_calibrate_val != 0) {
                period_ms *= osc_calibrate_val;
            }
            writeReg(Register.SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);
            // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end
            writeReg(Register.SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
        } else {
            // continuous back-to-back mode
            writeReg(Register.SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
        }
    }

    //***************************************************************************
    // reads the range value in mm from the register from a continous reading
    // get results in millimeters, centimeters or inches
    //***************************************************************************
    public double getDistanceContinousin() {
        double range = getDistanceContinouscm() / 2.54;
        return range;
    }
    public double getDistanceContinouscm() {
        double range = getDistanceContinousmm() / 10;
        return range;
    }
    public int getDistanceContinousmm() {
        startTimeout(100);
        while ((readReg(Register.RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
            if (checkTimeoutExpired()) {
                did_timeout = true;
                return 65535;
            }
        }
        // assumptions: Linearity Corrective Gain is 1000 (default);
        // fractional ranging is not enabled
        //int range = readReg16(Register.RESULT_RANGE_STATUS.bVal + 10);
        int range = readReg16(Register.RESULT_RANGE_STATUS_MM.bVal);
        writeReg(Register.SYSTEM_INTERRUPT_CLEAR, 0x01);
        return range;
    }

    //***************************************************************************
    // Performs a single-shot range measurement and returns the reading in
    // millimeters
    // based on VL53L0X_PerformSingleRangingMeasurement()
    // get results in millimeters, centimeters or inches
    //***************************************************************************
    public double getDistancein()
    {
        double range = getDistancecm() / 2.54;
        return range;
    }
    public double getDistancecm()
    {
        double range = getDistancemm() / 10;
        return range;
    }
    public int getDistancemm()
    {
        writeReg(0x80, 0x01);
        writeReg(0xFF, 0x01);
        writeReg(0x00, 0x00);
        writeReg(0x91, stop_variable);
        writeReg(0x00, 0x01);
        writeReg(0xFF, 0x00);
        writeReg(0x80, 0x00);
        writeReg(Register.SYSRANGE_START, 0x01);

        // "Wait until start bit has been cleared"
        startTimeout(200);
        while (convertToBoolean(readReg(Register.SYSRANGE_START) & 0x01))
        {
            if (checkTimeoutExpired())
            {
                did_timeout = true;
                return 65535;
            }
        }
        return getDistanceContinousmm();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Raw Register Reads
    ////////////////////////////////////////////////////////////////////////////////////////////////

    // Get reference SPAD (single photon avalanche diode) count and type
    // based on VL53L0X_get_info_from_device(),
    // but only gets reference SPAD count and type
    boolean getSpadInfo() {
        int tmp;
        writeReg(0x80, 0x01);
        writeReg(0xFF, 0x01);
        writeReg(0x00, 0x00);
        writeReg(0xFF, 0x06);
        writeReg(0x83, readReg(0x83) | 0x04);
        writeReg(0xFF, 0x07);
        writeReg(0x81, 0x01);
        writeReg(0x80, 0x01);
        writeReg(0x94, 0x6b);
        writeReg(0x83, 0x00);
        startTimeout(500);
        while (readReg(0x83) == 0x00) {
            if (checkTimeoutExpired()) {
                return false;
            }
        }
        writeReg(0x83, 0x01);
        tmp = readReg(0x92);
        spad_count = tmp & 0x7f;
        spad_type_is_aperture = ((tmp >> 7) & 0x01);
        writeReg(0x81, 0x00);
        writeReg(0xFF, 0x06);
        writeReg(0x83, readReg(0x83) & ~0x04);
        writeReg(0xFF, 0x01);
        writeReg(0x00, 0x01);
        writeReg(0xFF, 0x00);
        writeReg(0x80, 0x00);
        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Read and Write Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////

    protected void writeReg(int reg, int value) {
        byte array = (byte)(value);
        //Log.d("writeReg", "1 register: " + reg + ", value: " + value + ", array: " + TypeConversion.unsignedByteToInt(array));
        deviceClient.write8(reg, value);
    }

    protected void writeReg(final Register reg, int value) {
        byte array = (byte)(value);
        //Log.d("writeReg", "1 register: " + reg.bVal + ", value: " + value + ", array: " + TypeConversion.unsignedByteToInt(array));
        deviceClient.write8(reg.bVal, value);
        //deviceClient.write8(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    protected int readReg16(Register reg) {
        byte array[] = this.deviceClient.read(reg.bVal,2);
        //Log.d("readReg16", "register: " + reg.bVal + ", array: " + (array[0]) + array[1]);
        int value;
        value = TypeConversion.unsignedByteToInt(array[0]) * 256 + TypeConversion.unsignedByteToInt(array[1]);
        return value;
    }

    protected int readReg16(int reg) {
        byte array[] = this.deviceClient.read(reg,2);
        //Log.d("readReg16", "register: " + reg + ", array: " + (array[0]) + array[1]);
        int value;
        value = TypeConversion.unsignedByteToInt(array[0]) * 256 + TypeConversion.unsignedByteToInt(array[1]);
        return value;
    }

    protected int readReg(Register reg) {
        byte array[] = this.deviceClient.read(reg.bVal, 1);
        //Log.d("readReg", "register: " + reg.bVal + ", array: " + TypeConversion.unsignedByteToInt(array[0]));
        return TypeConversion.unsignedByteToInt(array[0]);
        //return TypeConversion.byteArrayToInt(deviceClient.read(reg.bVal, 1));
    }

    protected int readReg(int reg) {
        byte array[] = this.deviceClient.read(reg, 1);
        //byte array = this.deviceClient.read8(reg);
        //Log.d("readReg", "1 register: " + reg + ", array: " + TypeConversion.unsignedByteToInt(array[0]));
        return TypeConversion.unsignedByteToInt(array[0]);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Registers and Config Settings
    ////////////////////////////////////////////////////////////////////////////////////////////////

    // Set the return signal rate limit check value in units of MCPS (mega counts
    // per second). "This represents the amplitude of the signal reflected from the
    // target and detected by the device"; setting this limit presumably determines
    // the minimum measurement necessary for the sensor to report a valid reading.
    // Setting a lower limit increases the potential range of the sensor but also
    // seems to increase the likelihood of getting an inaccurate reading because of
    // unwanted reflections from objects other than the intended target.
    // Defaults to 0.25 MCPS as initialized by the ST API and this library.
    private boolean setSignalRateLimit(double limit_Mcps) {
        if (limit_Mcps < 0 || limit_Mcps > 511.99) {
            return false;
        }
        // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
        writeReg(Register.FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, (int) (limit_Mcps * (1 << 7)));
        return true;
    }

    // Get sequence step enables
    // based on VL53L0X_GetSequenceStepEnables()
    private SequenceStepEnables getSequenceStepEnables() {
        SequenceStepEnables tempSequenceSteps = new SequenceStepEnables();
        int sequence_config = readReg(Register.SYSTEM_SEQUENCE_CONFIG);
        tempSequenceSteps.tcc = convertToBoolean((sequence_config >> 4) & 0x1);
        tempSequenceSteps.dss = convertToBoolean((sequence_config >> 3) & 0x1);
        tempSequenceSteps.msrc = convertToBoolean((sequence_config >> 2) & 0x1);
        tempSequenceSteps.pre_range = convertToBoolean((sequence_config >> 6) & 0x1);
        tempSequenceSteps.final_range = convertToBoolean((sequence_config >> 7) & 0x1);
        return tempSequenceSteps;
    }

    // Decode sequence step timeout in MCLKs from register value
    // based on VL53L0X_decode_timeout()
    // Note: the original function returned a uint32_t, but the return value is
    // always stored in a uint16_t.
    private int decodeTimeout(int reg_val)
    {
        // format: "(LSByte * 2^MSByte) + 1"
        return (int)((reg_val & 0x00FF) << (int)((reg_val & 0xFF00) >> 8)) + 1;
    }

    // Encode sequence step timeout register value from timeout in MCLKs
    // based on VL53L0X_encode_timeout()
    // Note: the original function took a uint16_t, but the argument passed to it
    // is always a uint16_t.
    private int encodeTimeout(int timeout_mclks)
    {
        // format: "(LSByte * 2^MSByte) + 1"
        int ls_byte = 0;
        int ms_byte = 0;
        if (timeout_mclks > 0)
        {
            ls_byte = timeout_mclks - 1;
            while ((ls_byte & 0xFFFFFF00) > 0)
            {
                ls_byte >>= 1;
                ms_byte++;
            }
            return (ms_byte << 8) | (ls_byte & 0xFF);
        }
        else { return 0; }
    }
    private int decodeVcselPeriod(int reg_val) {
         return (((reg_val)+1)<<1);
    }

    // Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
    // based on VL53L0X_calc_macro_period_ps()
    // PLL_period_ps = 1655; macro_period_vclks = 2304
    private int calcMacroPeriod(int vcsel_period_pclks) {
        return ((((int)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000);
    }

    // Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
    // based on VL53L0X_calc_timeout_us()
    private int timeoutMclksToMicroseconds(int timeout_period_mclks, int vcsel_period_pclks)
    {
        int macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
        return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
    }

    // Get the VCSEL pulse period in PCLKs for the given period type.
    // based on VL53L0X_get_vcsel_pulse_period()
    private int getVcselPulsePeriod(vcselPeriodType type)
    {
        if (type == vcselPeriodType.VcselPeriodPreRange)
        {
            return decodeVcselPeriod(readReg(Register.PRE_RANGE_CONFIG_VCSEL_PERIOD));
        }
        else if (type == vcselPeriodType.VcselPeriodFinalRange)
        {
            return decodeVcselPeriod(readReg(Register.FINAL_RANGE_CONFIG_VCSEL_PERIOD));
        }
        else { return 255; }
    }

    private int timeoutMicrosecondsToMclks(int timeout_period_us, int vcsel_period_pclks) {
        int macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
        return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
    };

    // Get sequence step timeouts
    // based on get_sequence_step_timeout(),
    // but gets all timeouts instead of just the requested one, and also stores
    // intermediate values
    private SequenceStepTimeouts getSequenceStepTimeouts(SequenceStepEnables enables)
    {
        SequenceStepTimeouts timeouts = new SequenceStepTimeouts();
        timeouts.pre_range_vcsel_period_pclks = getVcselPulsePeriod(vcselPeriodType.VcselPeriodPreRange);
        timeouts.msrc_dss_tcc_mclks = readReg(Register.MSRC_CONFIG_TIMEOUT_MACROP) + 1;
        timeouts.msrc_dss_tcc_us = timeoutMclksToMicroseconds(timeouts.msrc_dss_tcc_mclks, timeouts.pre_range_vcsel_period_pclks);
        timeouts.pre_range_mclks = decodeTimeout(readReg(Register.PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
        timeouts.pre_range_us = timeoutMclksToMicroseconds(timeouts.pre_range_mclks, timeouts.pre_range_vcsel_period_pclks);
        timeouts.final_range_vcsel_period_pclks = getVcselPulsePeriod(vcselPeriodType.VcselPeriodFinalRange);
        timeouts.final_range_mclks = decodeTimeout(readReg(Register.FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));
        if (enables.pre_range)
        {
            timeouts.final_range_mclks -= timeouts.pre_range_mclks;
        }
        timeouts.final_range_us = timeoutMclksToMicroseconds(timeouts.final_range_mclks, timeouts.final_range_vcsel_period_pclks);
        return timeouts;
    }

    // based on VL53L0X_perform_single_ref_calibration()
    private boolean performSingleRefCalibration(int vhv_init_byte)
    {
        //Log.d("performSingleRefCal", "vhv_init_byte: " + vhv_init_byte);
        writeReg(Register.SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP
        startTimeout(500);
        //Log.d("performSingleRefCal", "Start Timer");
        while ((readReg(Register.RESULT_INTERRUPT_STATUS) & 0x07) == 0)
        {
            if (checkTimeoutExpired()) {
                //Log.d("performSingleRefCal", "Timeout Bad");
                return false; }
        }
        writeReg(Register.SYSTEM_INTERRUPT_CLEAR, 0x01);
        writeReg(Register.SYSRANGE_START, 0x00);
        //Log.d("performSingleRefCal", "Good");
        return true;
    }

    // Set the measurement timing budget in microseconds, which is the time allowed
    // for one measurement; the ST API and this library take care of splitting the
    // timing budget among the sub-steps in the ranging sequence. A longer timing
    // budget allows for more accurate measurements. Increasing the budget by a
    // factor of N decreases the range measurement standard deviation by a factor of
    // sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
    // based on VL53L0X_set_measurement_timing_budget_micro_seconds()
    private boolean setMeasurementTimingBudget(int budget_us)
    {
        SequenceStepEnables enables;
        SequenceStepTimeouts timeouts;
        final int StartOverhead      = 1320; // note that this is different than the value in get_
        final int EndOverhead        = 960;
        final int MsrcOverhead       = 660;
        final int TccOverhead        = 590;
        final int DssOverhead        = 690;
        final int PreRangeOverhead   = 660;
        final int FinalRangeOverhead = 550;
        final int MinTimingBudget = 20000;
        if (budget_us < MinTimingBudget) { return false; }
        int used_budget_us = StartOverhead + EndOverhead;
        enables  = getSequenceStepEnables();
        timeouts = getSequenceStepTimeouts(enables);
        if (enables.tcc)
        {
            used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
        }
        if (enables.dss)
        {
            used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
        }
        else if (enables.msrc)
        {
            used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
        }
        if (enables.pre_range)
        {
            used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
        }
        if (enables.final_range)
        {
            used_budget_us += FinalRangeOverhead;
            // "Note that the final range timeout is determined by the timing
            // budget and the sum of all other timeouts within the sequence.
            // If there is no room for the final range timeout, then an error
            // will be set. Otherwise the remaining time will be applied to
            // the final range."
            if (used_budget_us > budget_us)
            {
                // "Requested timeout too big."
                return false;
            }
            int final_range_timeout_us = budget_us - used_budget_us;
            // set_sequence_step_timeout() begin
            // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)
            // "For the final range timeout, the pre-range timeout
            //  must be added. To do this both final and pre-range
            //  timeouts must be expressed in macro periods MClks
            //  because they have different vcsel periods."
            int final_range_timeout_mclks = timeoutMicrosecondsToMclks(final_range_timeout_us, timeouts.final_range_vcsel_period_pclks);
            if (enables.pre_range)
            {
                final_range_timeout_mclks += timeouts.pre_range_mclks;
            }
            writeReg(Register.FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(final_range_timeout_mclks));
            // set_sequence_step_timeout() end
            measurement_timing_budget_us = budget_us; // store for internal reuse
        }
        return true;
    }

    // Get the measurement timing budget in microseconds
    // based on VL53L0X_get_measurement_timing_budget_micro_seconds()
    // in us
    private int getMeasurementTimingBudget()
    {
        SequenceStepEnables enables;
        SequenceStepTimeouts timeouts;
        final int StartOverhead      = 1910; // note that this is different than the value in set_
        final int EndOverhead        = 960;
        final int MsrcOverhead       = 660;
        final int TccOverhead        = 590;
        final int DssOverhead        = 690;
        final int PreRangeOverhead   = 660;
        final int FinalRangeOverhead = 550;
        // "Start and end overhead times always present"
        int budget_us = StartOverhead + EndOverhead;
        enables = getSequenceStepEnables();
        timeouts = getSequenceStepTimeouts(enables);
        if (enables.tcc)
        {
            budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
        }
        if (enables.dss)
        {
            budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
        }
        else if (enables.msrc)
        {
            budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
        }
        if (enables.pre_range)
        {
            budget_us += (timeouts.pre_range_us + PreRangeOverhead);
        }
        if (enables.final_range)
        {
            budget_us += (timeouts.final_range_us + FinalRangeOverhead);
        }
        measurement_timing_budget_us = budget_us; // store for internal reuse
        return budget_us;
    }

    public enum Register
    {
        SYSRANGE_START                              (0x00),
        SYSTEM_THRESH_HIGH                          (0x0C),
        SYSTEM_THRESH_LOW                           (0x0E),
        SYSTEM_SEQUENCE_CONFIG                      (0x01),
        SYSTEM_RANGE_CONFIG                         (0x09),
        SYSTEM_INTERMEASUREMENT_PERIOD              (0x04),
        SYSTEM_INTERRUPT_CONFIG_GPIO                (0x0A),
        SYSTEM_INTERRUPT_CLEAR                      (0x0B),
        RESULT_INTERRUPT_STATUS                     (0x13),
        RESULT_RANGE_STATUS                         (0x14),
        RESULT_RANGE_STATUS_MM                      (0x1E),
        CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       (0x20),
        PRE_RANGE_CONFIG_MIN_SNR                    (0x27),
        ALGO_PART_TO_PART_RANGE_OFFSET_MM           (0x28),
        ALGO_PHASECAL_LIM                           (0x30),
        ALGO_PHASECAL_CONFIG_TIMEOUT                (0x30),
        GLOBAL_CONFIG_VCSEL_WIDTH                   (0x32),
        HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       (0x33),
        FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT (0x44),
        MSRC_CONFIG_TIMEOUT_MACROP                  (0x46),
        FINAL_RANGE_CONFIG_VALID_PHASE_LOW          (0x47),
        FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         (0x48),
        DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         (0x4E),
        DYNAMIC_SPAD_REF_EN_START_OFFSET            (0x4F),
        PRE_RANGE_CONFIG_VCSEL_PERIOD               (0x50),
        PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          (0x51),
        PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          (0x52),
        HISTOGRAM_CONFIG_READOUT_CTRL               (0x55),
        PRE_RANGE_CONFIG_VALID_PHASE_LOW            (0x56),
        PRE_RANGE_CONFIG_VALID_PHASE_HIGH           (0x57),
        MSRC_CONFIG_CONTROL                         (0x60),
        PRE_RANGE_CONFIG_SIGMA_THRESH_HI            (0x61),
        PRE_RANGE_CONFIG_SIGMA_THRESH_LO            (0x62),
        PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          (0x64),
        FINAL_RANGE_CONFIG_MIN_SNR                  (0x67),
        FINAL_RANGE_CONFIG_VCSEL_PERIOD             (0x70),
        FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        (0x71),
        FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        (0x72),
        I2C_SLAVE_DEVICE_ADDRESS                    (0x8A),
        POWER_MANAGEMENT_GO1_POWER_FORCE            (0x80),
        SYSTEM_HISTOGRAM_BIN                        (0x81),
        GPIO_HV_MUX_ACTIVE_HIGH                     (0x84),
        VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           (0x89),
        GLOBAL_CONFIG_SPAD_ENABLES_REF_0            (0xB0),
        GLOBAL_CONFIG_SPAD_ENABLES_REF_1            (0xB1),
        GLOBAL_CONFIG_SPAD_ENABLES_REF_2            (0xB2),
        GLOBAL_CONFIG_SPAD_ENABLES_REF_3            (0xB3),
        GLOBAL_CONFIG_SPAD_ENABLES_REF_4            (0xB4),
        GLOBAL_CONFIG_SPAD_ENABLES_REF_5            (0xB5),
        GLOBAL_CONFIG_REF_EN_START_SELECT           (0xB6),
        RESULT_PEAK_SIGNAL_RATE_REF                 (0xB6),
        RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       (0xBC),
        SOFT_RESET_GO2_SOFT_RESET_N                 (0xBF),
        IDENTIFICATION_MODEL_ID                     (0xC0),
        RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        (0xC0),
        IDENTIFICATION_REVISION_ID                  (0xC2),
        RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       (0xD0),
        RESULT_CORE_RANGING_TOTAL_EVENTS_REF        (0xD4),
        OSC_CALIBRATE_VAL                           (0xF8),
        FIRST(RESULT_INTERRUPT_STATUS.bVal),
        LAST(RESULT_RANGE_STATUS.bVal);

        public int bVal;
        Register(int bVal)
        {
            this.bVal = bVal;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Construction and Initialization
    ////////////////////////////////////////////////////////////////////////////////////////////////

    private final static I2cAddr ADDRESS_I2C_DEFAULT7BIT = I2cAddr.create7bit(0x29);
    private final static I2cAddr ADDRESS_I2C_DEFAULT8BIT = I2cAddr.create8bit(0x52);

    public VL53L0X(I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true);
        did_timeout = false;
        //this.setOptimalReadWindow();
        deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT8BIT);
        super.registerArmingStateCallback(false); // Deals with USB cables getting unplugged
        // Sensor starts off disengaged so we can change things like I2C address. Need to engage
        deviceClient.engage();
    }

    public boolean init() {

        // "Set I2C standard mode"
        writeReg(0x88, 0x00);
        writeReg(0x80, 0x01);
        writeReg(0xFF, 0x01);
        writeReg(0x00, 0x00);
        stop_variable = readReg(0x91);
        writeReg(0x00, 0x01);
        writeReg(0xFF, 0x00);
        writeReg(0x80, 0x00);

        // set final range signal rate limit to 0.25 MCPS (million counts per second)
        setSignalRateLimit(0.25);

        writeReg(Register.SYSTEM_SEQUENCE_CONFIG, 0xFF);
        if (!getSpadInfo()) {
            return false;
        }
        int first_spad_to_enable = (spad_type_is_aperture == 1) ? 12 : 0; // 12 is the first aperture spad
        int spads_enabled = 0;

        // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
        // the API, but the same data seems to be more easily readable from
        // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
        byte ref_spad_map[];
        ref_spad_map = deviceClient.read(Register.GLOBAL_CONFIG_SPAD_ENABLES_REF_0.bVal , 6);

        // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

        writeReg(0xFF, 0x01);
        writeReg(Register.DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
        writeReg(Register.DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
        writeReg(0xFF, 0x00);
        writeReg(Register.GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

        for (int i = 0; i < 48; i++)
        {
            if (i < first_spad_to_enable || spads_enabled == spad_count)
            {
                // This bit is lower than the first one that should be enabled, or
                // (reference_spad_count) bits have already been enabled, so zero this bit
                ref_spad_map[i / 8] &= ~(1 << (i % 8));
            }
            else if (((ref_spad_map[i / 8] >> (i % 8)) & 0x1) == 1)
            {
                spads_enabled++;
            }
        }
        //for (byte a : ref_spad_map)
        //    Log.d("0 writeReg", "value:" + TypeConversion.unsignedByteToInt(a));

        deviceClient.write(Register.GLOBAL_CONFIG_SPAD_ENABLES_REF_0.bVal, ref_spad_map);
        //Log.d("writeReg", "register: " + Register.GLOBAL_CONFIG_SPAD_ENABLES_REF_0.bVal);
        //for (byte a : ref_spad_map)
        //    Log.d("1 writeReg", "value:" + TypeConversion.unsignedByteToInt(a));
        // -- VL53L0X_load_tuning_settings() begin
        // DefaultTuningSettings from vl53l0x_tuning.h
        writeReg(0xFF, 0x01);
        writeReg(0x00, 0x00);
        writeReg(0xFF, 0x00);
        writeReg(0x09, 0x00);
        writeReg(0x10, 0x00);
        writeReg(0x11, 0x00);
        writeReg(0x24, 0x01);
        writeReg(0x25, 0xFF);
        writeReg(0x75, 0x00);
        writeReg(0xFF, 0x01);
        writeReg(0x4E, 0x2C);
        writeReg(0x48, 0x00);
        writeReg(0x30, 0x20);
        writeReg(0xFF, 0x00);
        writeReg(0x30, 0x09);
        writeReg(0x54, 0x00);
        writeReg(0x31, 0x04);
        writeReg(0x32, 0x03);
        writeReg(0x40, 0x83);
        writeReg(0x46, 0x25);
        writeReg(0x60, 0x00);
        writeReg(0x27, 0x00);
        writeReg(0x50, 0x06);
        writeReg(0x51, 0x00);
        writeReg(0x52, 0x96);
        writeReg(0x56, 0x08);
        writeReg(0x57, 0x30);
        writeReg(0x61, 0x00);
        writeReg(0x62, 0x00);
        writeReg(0x64, 0x00);
        writeReg(0x65, 0x00);
        writeReg(0x66, 0xA0);
        writeReg(0xFF, 0x01);
        writeReg(0x22, 0x32);
        writeReg(0x47, 0x14);
        writeReg(0x49, 0xFF);
        writeReg(0x4A, 0x00);
        writeReg(0xFF, 0x00);
        writeReg(0x7A, 0x0A);
        writeReg(0x7B, 0x00);
        writeReg(0x78, 0x21);
        writeReg(0xFF, 0x01);
        writeReg(0x23, 0x34);
        writeReg(0x42, 0x00);
        writeReg(0x44, 0xFF);
        writeReg(0x45, 0x26);
        writeReg(0x46, 0x05);
        writeReg(0x40, 0x40);
        writeReg(0x0E, 0x06);
        writeReg(0x20, 0x1A);
        writeReg(0x43, 0x40);
        writeReg(0xFF, 0x00);
        writeReg(0x34, 0x03);
        writeReg(0x35, 0x44);
        writeReg(0xFF, 0x01);
        writeReg(0x31, 0x04);
        writeReg(0x4B, 0x09);
        writeReg(0x4C, 0x05);
        writeReg(0x4D, 0x04);
        writeReg(0xFF, 0x00);
        writeReg(0x44, 0x00);
        writeReg(0x45, 0x20);
        writeReg(0x47, 0x08);
        writeReg(0x48, 0x28);
        writeReg(0x67, 0x00);
        writeReg(0x70, 0x04);
        writeReg(0x71, 0x01);
        writeReg(0x72, 0xFE);
        writeReg(0x76, 0x00);
        writeReg(0x77, 0x00);
        writeReg(0xFF, 0x01);
        writeReg(0x0D, 0x01);
        writeReg(0xFF, 0x00);
        writeReg(0x80, 0x01);
        writeReg(0x01, 0xF8);
        writeReg(0xFF, 0x01);
        writeReg(0x8E, 0x01);
        writeReg(0x00, 0x01);
        writeReg(0xFF, 0x00);
        writeReg(0x80, 0x00);
        // -- VL53L0X_load_tuning_settings() end

        // "Set interrupt config to new sample ready"
        // -- VL53L0X_SetGpioConfig() begin
        writeReg(Register.SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
        writeReg(Register.GPIO_HV_MUX_ACTIVE_HIGH, readReg(Register.GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
        writeReg(Register.SYSTEM_INTERRUPT_CLEAR, 0x01);
        // -- VL53L0X_SetGpioConfig() end

        measurement_timing_budget_us = getMeasurementTimingBudget();

        // "Disable MSRC and TCC by default"
        // MSRC = Minimum Signal Rate Check
        // TCC = Target CentreCheck
        // -- VL53L0X_SetSequenceStepEnable() begin
        writeReg(Register.SYSTEM_SEQUENCE_CONFIG, 0xE8);
        // -- VL53L0X_SetSequenceStepEnable() end

        // "Recalculate timing budget"
        setMeasurementTimingBudget(measurement_timing_budget_us);
        // VL53L0X_StaticInit() end

        // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

        // -- VL53L0X_perform_vhv_calibration() begin
        writeReg(Register.SYSTEM_SEQUENCE_CONFIG, 0x01);
        if (!performSingleRefCalibration(0x40)) { return false; }
        // -- VL53L0X_perform_vhv_calibration() end

        // -- VL53L0X_perform_phase_calibration() begin
        writeReg(Register.SYSTEM_SEQUENCE_CONFIG, 0x02);
        if (!performSingleRefCalibration(0x00)) { return false; }

        // -- VL53L0X_perform_phase_calibration() end
        // "restore the previous Sequence Config"
        writeReg(Register.SYSTEM_SEQUENCE_CONFIG, 0xE8);
        // VL53L0X_PerformRefCalibration() end

        return true;
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
    protected synchronized boolean doInitialize()
    {
        return true;
    }

    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.valueOf("ST Electronics");
    }

    @Override
    public String getDeviceName()
    {
        return "VL53L0X Light Ranging Sensor";
    }

}
