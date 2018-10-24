package club.towr5291.robotconfig;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import club.towr5291.functions.Constants;
import club.towr5291.functions.FileLogger;
import club.towr5291.functions.TOWR5291Utils;
import club.towr5291.libraries.robotConfig;
import club.towr5291.libraries.robotConfigSettings;

public class HardwareSensorsRoverRuckus {

    public DigitalChannel limitSwitch1AngleMotor;
    public DigitalChannel limitSwitchMinForLift;
    public DigitalChannel Green1, Green2;
    public DigitalChannel Blue1, Blue2;
    public DigitalChannel Red1, Red2;

    /* local OpMode members. */
    HardwareMap hwMap            = null;
    //set up the variables for the logger
    private FileLogger fileLogger = null;
    private static final String TAG = "HardwareDriveMotors";

    /* Constructor */
    public HardwareSensorsRoverRuckus(){

    }

    public void init(FileLogger fileloggerhandle, HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        this.fileLogger = fileloggerhandle;

        Green1 = hwMap.get(DigitalChannel.class, "green1");
        Green2 = hwMap.get(DigitalChannel.class, "green2");
        Red1 = hwMap.get(DigitalChannel.class, "red1");
        Red2 = hwMap.get(DigitalChannel.class, "red2");
        Blue1 = hwMap.get(DigitalChannel.class, "blue1");
        Blue2 = hwMap.get(DigitalChannel.class, "blue2");

        Green1.setMode(DigitalChannel.Mode.OUTPUT);
        Green2.setMode(DigitalChannel.Mode.OUTPUT);
        Red1.setMode(DigitalChannel.Mode.OUTPUT);
        Red2.setMode(DigitalChannel.Mode.OUTPUT);
        Blue1.setMode(DigitalChannel.Mode.OUTPUT);
        Blue2.setMode(DigitalChannel.Mode.OUTPUT);

        limitSwitch1AngleMotor = hwMap.get(DigitalChannel.class, "limitSwitch1");
        limitSwitch1AngleMotor.setMode(DigitalChannel.Mode.INPUT);

        limitSwitchMinForLift = hwMap.get(DigitalChannel.class, "limitSwitch2");
        limitSwitchMinForLift.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean getLimit1State(){return limitSwitch1AngleMotor.getState();}
    public boolean getLimit2State(){return limitSwitchMinForLift.getState();}

    public void LedState (boolean greenLED1, boolean redLED1, boolean blueLED1, boolean greenLED2, boolean redLED2, boolean blueLED2) {
        Red1.setState(redLED1);//R
        Green1.setState(greenLED1);//G
        Blue1.setState(blueLED1);//B
        Red2.setState(redLED2);//R
        Green2.setState(greenLED2);//G
        Blue2.setState(blueLED2);//B
    }
}

