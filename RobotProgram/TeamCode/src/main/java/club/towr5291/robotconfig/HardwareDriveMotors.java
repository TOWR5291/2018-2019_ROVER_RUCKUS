package club.towr5291.robotconfig;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import club.towr5291.functions.FileLogger;
import club.towr5291.libraries.robotConfigSettings;
import club.towr5291.libraries.towrUtils;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the hardware for a drive base.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 */
public class HardwareDriveMotors
{
    /* Public OpMode members. */
    public DcMotor  baseMotor1  = null;
    public DcMotor  baseMotor2  = null;
    public DcMotor  baseMotor3  = null;
    public DcMotor  baseMotor4  = null;

    /* local OpMode members. */
    HardwareMap hwMap            = null;
    private ElapsedTime period   = new ElapsedTime();

    private boolean gyroAssistEnabled = false;

    //set up the variables for the logger
    private FileLogger fileLogger;
    private static final String TAG = "HardwareDriveMotors";
    /* Constructor */
    public HardwareDriveMotors(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(FileLogger fileLoggerFromMaster, HardwareMap ahwMap, robotConfigSettings.robotConfigChoice baseConfig) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        this.fileLogger = fileLoggerFromMaster;

        // Define and Initialize Motors
        baseMotor1  = hwMap.dcMotor.get("leftMotor1");
        baseMotor2  = hwMap.dcMotor.get("leftMotor2");
        baseMotor3  = hwMap.dcMotor.get("rightMotor1");
        baseMotor4  = hwMap.dcMotor.get("rightMotor2");

        setHardwareDriveDirections(baseConfig);

        // Set all motors to zero power
        setHardwareDrivePower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        setHardwareDriveResetEncoders();

        setHardwareDriveRunUsingEncoders();
    }

    public void init(FileLogger fileloggerhandle, HardwareMap ahwMap, robotConfigSettings.robotConfigChoice baseConfig, String motor1, String motor2, String motor3, String motor4) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        if (motor1 != null)
            baseMotor1  = hwMap.dcMotor.get(motor1);
        if (motor2 != null)
            baseMotor2  = hwMap.dcMotor.get(motor2);
        if (motor3 != null)
            baseMotor3  = hwMap.dcMotor.get(motor3);
        if (motor4 != null)
            baseMotor4  = hwMap.dcMotor.get(motor4);

        setHardwareDriveDirections(baseConfig);

        // Set all motors to zero power
        setHardwareDrivePower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        setHardwareDriveResetEncoders();

        setHardwareDriveRunUsingEncoders();
    }

    public void init(HardwareMap ahwMap, robotConfigSettings.robotConfigChoice baseConfig) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        baseMotor1  = hwMap.dcMotor.get("leftMotor1");
        baseMotor2  = hwMap.dcMotor.get("leftMotor2");
        baseMotor3  = hwMap.dcMotor.get("rightMotor1");
        baseMotor4  = hwMap.dcMotor.get("rightMotor2");

        setHardwareDriveDirections(baseConfig);

        // Set all motors to zero power
        setHardwareDrivePower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        setHardwareDriveResetEncoders();

        setHardwareDriveRunUsingEncoders();
    }

    public void setHardwareDriveDirections(robotConfigSettings.robotConfigChoice baseConfig){

        switch (baseConfig) {
            case TileRunner2x20:
            case TileRunner2x40:
            case TileRunner2x60:
                if (baseMotor1 != null)
                    baseMotor1.setDirection(DcMotor.Direction.FORWARD);
                if (baseMotor2 != null)
                    baseMotor2.setDirection(DcMotor.Direction.REVERSE);
                if (baseMotor3 != null)
                    baseMotor3.setDirection(DcMotor.Direction.FORWARD);
                if (baseMotor4 != null)
                    baseMotor4.setDirection(DcMotor.Direction.REVERSE);
                break;
            case TileRunnerMecanum2x40:
                if (baseMotor1 != null)
                    baseMotor1.setDirection(DcMotor.Direction.FORWARD);
                if (baseMotor2 != null)
                    baseMotor2.setDirection(DcMotor.Direction.REVERSE);
                if (baseMotor3 != null)
                    baseMotor3.setDirection(DcMotor.Direction.REVERSE);
                if (baseMotor4 != null)
                    baseMotor4.setDirection(DcMotor.Direction.FORWARD);
                break;
            default:
                if (baseMotor1 != null)
                    baseMotor1.setDirection(DcMotor.Direction.FORWARD);
                if (baseMotor2 != null)
                    baseMotor2.setDirection(DcMotor.Direction.REVERSE);
                if (baseMotor3 != null)
                    baseMotor3.setDirection(DcMotor.Direction.FORWARD);
                if (baseMotor4 != null)
                    baseMotor4.setDirection(DcMotor.Direction.FORWARD);
                break;
        }
    }

    public int getHardwareDriveIsBusy() {
        //this will return a bitmapped integer,
        // int 0  = 0000 is right2, right1, left2, left1
        // int 1  = 0001 is right2, right1, left2, left1 (1)
        // int 2  = 0010 is right2, right1, left2 (1), left1
        // int 3  = 0011 is right2, right1, left2 (1), left1 (1)
        // int 4  = 0100 is right2, right1 (1), left2, left1
        // int 5  = 0101 is right2, right1 (1), left2, left1 (1)
        // int 6  = 0110 is right2, right1 (1), left2 (1), left1
        // int 7  = 0111 is right2, right1 (1), left2 (1), left1 (1)
        // int 8  = 1000 is right2 (1), right1, left2, left1
        // int 9  = 1001 is right2 (1), right1, left2, left1 (1)
        // int 10 = 1010 is right2 (1), right1, left2 (1), left1
        // int 11 = 1011 is right2 (1), right1, left2 (1), left1 (1)
        // int 12 = 1100 is right2 (1), right1 (1), left2, left1
        // int 13 = 1101 is right2 (1), right1 (1), left2, left1 (1)
        // int 14 = 1101 is right2 (1), right1 (1), left2, left1 (1)
        // int 15 = 1110 is right2 (1), right1 (1), left2 (1), left1
        // int 16 = 1111 is right2 (1), right1 (1), left2 (1), left1 (1)

        int myInt1 = 0;
        int myInt2 = 0;
        int myInt3 = 0;
        int myInt4 = 0;

        if (baseMotor1 != null)
            myInt1 = (baseMotor1.isBusy()) ? 1 : 0;
        if (baseMotor2 != null)
            myInt2 = (baseMotor2.isBusy()) ? 1 : 0;
        if (baseMotor3 != null)
            myInt3 = (baseMotor3.isBusy()) ? 1 : 0;
        if (baseMotor4 != null)
            myInt4 = (baseMotor4.isBusy()) ? 1 : 0;

        return (myInt1) + (2 * myInt2) + (4 * myInt3)  + (8 * myInt4);

    }

    public class motorEncoderPositions {

        private int motor1;      //is the current encoder position or motor 1
        private int motor2;      //is the current encoder position or motor 2
        private int motor3;      //is the current encoder position or motor 3
        private int motor4;      //is the current encoder position or motor 4


        // Constructor
        public motorEncoderPositions()
        {
            this.motor1 = 0;
            this.motor2 = 0;
            this.motor3 = 0;
            this.motor4 = 0;
        }

        public void setMotor1EncoderValue (int value) {
            this.motor1 = value;
        }

        public void setMotor2EncoderValue (int value) {
            this.motor2 = value;
        }

        public void setMotor3EncoderValue (int value) {
            this.motor3 = value;
        }

        public void setMotor4EncoderValue (int value) {
            this.motor4 = value;
        }

        public int getMotor1EncoderValue () {
            return this.motor1;
        }

        public int getMotor2EncoderValue () {
            return this.motor2;
        }

        public int getMotor3EncoderValue () {
            return this.motor3;
        }

        public int getMotor4EncoderValue () {
            return this.motor4;
        }

    }

    public motorEncoderPositions getHardwareDriveEncoderPosition() {

        motorEncoderPositions positions = new motorEncoderPositions();
        if (baseMotor1 != null)
            positions.setMotor1EncoderValue(baseMotor1.getCurrentPosition());
        else
            positions.setMotor1EncoderValue(0);
        if (baseMotor2 != null)
            positions.setMotor2EncoderValue(baseMotor2.getCurrentPosition());
        else
            positions.setMotor2EncoderValue(0);
        if (baseMotor3 != null)
            positions.setMotor3EncoderValue(baseMotor3.getCurrentPosition());
        else
            positions.setMotor3EncoderValue(0);
        if (baseMotor4 != null)
            positions.setMotor4EncoderValue(baseMotor4.getCurrentPosition());
        else
            positions.setMotor4EncoderValue(0);

        return positions;
    }

    public void setHardwareDriveResetEncoders() {
        if (baseMotor1 != null)
            baseMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (baseMotor2 != null)
            baseMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (baseMotor3 != null)
            baseMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (baseMotor4 != null)
            baseMotor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setHardwareDriveRunUsingEncoders() {
        setHardwareDriveLeftRunUsingEncoders();
        setHardwareDriveRightRunUsingEncoders();
    }

    public void setHardwareDriveLeftRunUsingEncoders() {
        if (baseMotor1 != null)
            baseMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (baseMotor2 != null)
            baseMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setHardwareDriveRightRunUsingEncoders() {
        if (baseMotor3 != null)
            baseMotor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (baseMotor4 != null)
            baseMotor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setHardwareDriveRunWithoutEncoders() {
        if (baseMotor1 != null)
            baseMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (baseMotor2 != null)
            baseMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (baseMotor3 != null)
            baseMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (baseMotor4 != null)
            baseMotor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setHardwareDriveRunToPosition() {
        setHardwareDriveLeftRunToPosition();
        setHardwareDriveRightRunToPosition();
    }

    public void setHardwareDriveLeftRunToPosition() {
        if (baseMotor1 != null)
            baseMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (baseMotor2 != null)
            baseMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setHardwareDriveRightRunToPosition() {
        if (baseMotor3 != null)
            baseMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (baseMotor4 != null)
            baseMotor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setHardwareDrivePower (double leftFront, double leftBack, double rightFront, double rightBack) {
        if (baseMotor1 != null)
            baseMotor1.setPower(leftFront);
        if (baseMotor2 != null)
            baseMotor2.setPower(leftBack);
        if (baseMotor3 != null)
            baseMotor3.setPower(rightFront);
        if (baseMotor4 != null)
            baseMotor4.setPower(rightBack);
    }

    //set the drive motors power, both left and right
    public void setHardwareDrivePower (double power) {
        setHardwareDriveLeftMotorPower(power);
        setHardwareDriveRightMotorPower(power);
    }

    //set the left motors drive power
    public void setHardwareDriveLeftMotorPower (double power) {
        setHardwareDriveLeft1MotorPower(power);
        setHardwareDriveLeft2MotorPower(power);
    }

    //set the right drive motors power
    public void setHardwareDriveRightMotorPower (double power) {
        setHardwareDriveRight1MotorPower(power);
        setHardwareDriveRight2MotorPower(power);
    }

    public void setHardwareDriveLeft1MotorPower (double power) {
        if (baseMotor1 != null)
            baseMotor1.setPower(power);
    }

    public void setHardwareDriveLeft2MotorPower (double power) {
        if (baseMotor2 != null)
            baseMotor2.setPower(power);
    }

    public void setHardwareDriveRight1MotorPower (double power) {
        if (baseMotor3 != null)
            baseMotor3.setPower(power);
    }

    public void setHardwareDriveRight2MotorPower (double power) {
        if (baseMotor4 != null)
            baseMotor4.setPower(power);
    }


    /**
     * This Method was taken from Triton Robotics Library
     * This method normalizes the power to the four wheels for mecanum drive.
     *
     * @param wheelSpeeds specifies the wheel speed of all four wheels.
     */
    private void normalize(double[] wheelSpeeds)
    {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++)
        {
            double magnitude = Math.abs(wheelSpeeds[i]);
            if (magnitude > maxMagnitude)
            {
                maxMagnitude = magnitude;
            }
        }

        if (maxMagnitude > 1.0)
        {
            for (int i = 0; i < wheelSpeeds.length; i++)
            {
                wheelSpeeds[i] /= maxMagnitude;
            }
        }
    }   //normalize

    public class HardwareDriveTankMotorSpeeds {
        public double tankLeft;
        public double tankRight;

        public HardwareDriveTankMotorSpeeds(double leftPower, double rightPower) {
            this.tankLeft = leftPower;
            this.tankRight = rightPower;
        }
        public HardwareDriveTankMotorSpeeds() {
            this(0, 0);
        }

        public HardwareDriveTankMotorSpeeds(double[] vals) {
            this();
            set(vals);
        }

        public void set(double[] vals) {
            if (vals != null) {
                tankLeft = vals.length > 0 ? vals[0] : 0;
                tankRight = vals.length > 1 ? vals[1] : 0;
            } else {
                tankLeft = 0;
                tankRight = 0;
            }
        }

        @Override
        public String toString() {
            return "{LeftPower= " + tankLeft + ", RightPower= " + tankRight + "}";
        }
    }
}

