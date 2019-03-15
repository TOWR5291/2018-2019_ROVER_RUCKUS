package club.towr5291.robotconfig;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import club.towr5291.functions.FileLogger;

public class HardwareSensorsRoverRuckus {

    public DigitalChannel limitSwitch1AngleMotor;
    public DigitalChannel limitSwitch2AngleMotor;
    public DigitalChannel limitSwitch3AngleMotor;
    public DigitalChannel limitSwitch4AngleMotor;

    HardwareMap hwMap = null;

    /* Constructor */
    public HardwareSensorsRoverRuckus(){

    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //One of these is the port for the limit switch because I do not know if it is port 1 or two for each
        //limitSwitch1AngleMotor = hwMap.get(DigitalChannel.class, "limitSwitch1A");
        limitSwitch1AngleMotor = hwMap.get(DigitalChannel.class, "limitSwitch1B");
        //limitSwitch2AngleMotor = hwMap.get(DigitalChannel.class, "limitSwitch2A");
        limitSwitch2AngleMotor = hwMap.get(DigitalChannel.class, "limitSwitch2B");
        //limitSwitch3AngleMotor = hwMap.get(DigitalChannel.class, "limitSwitch3A");
        limitSwitch3AngleMotor = hwMap.get(DigitalChannel.class, "limitSwitch3B");
        //limitSwitch4AngleMotor = hwMap.get(DigitalChannel.class, "limitSwitch4A");
        limitSwitch4AngleMotor = hwMap.get(DigitalChannel.class, "limitSwitch4B");

        limitSwitch1AngleMotor.setMode(DigitalChannel.Mode.INPUT);
        limitSwitch2AngleMotor.setMode(DigitalChannel.Mode.INPUT);
        limitSwitch3AngleMotor.setMode(DigitalChannel.Mode.INPUT);
        limitSwitch4AngleMotor.setMode(DigitalChannel.Mode.INPUT);

    }


    public boolean getLimitSwitch1AngleMotorState() {
        return limitSwitch1AngleMotor.getState();
    }

    public boolean getLimitSwitch2AngleMotorState() {
        return limitSwitch2AngleMotor.getState();
    }

    public boolean getLimitSwitch3AngleMotorState() {
        return limitSwitch3AngleMotor.getState();
    }

    public boolean getLimitSwitch4AngleMotorState() {
        return limitSwitch4AngleMotor.getState();
    }

}

