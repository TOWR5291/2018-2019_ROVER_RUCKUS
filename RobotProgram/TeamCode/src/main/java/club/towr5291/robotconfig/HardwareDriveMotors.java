package club.towr5291.robotconfig;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import club.towr5291.functions.Constants;
import club.towr5291.functions.FileLogger;
import club.towr5291.functions.TOWR5291Utils;
import club.towr5291.libraries.LibraryMotorType;
import club.towr5291.libraries.robotConfig;
import club.towr5291.libraries.robotConfigSettings;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the hardware for a drive base.
 *
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 *
 * Modification history
 * Edited by:
 * Ian Haden 07/15/2017 -> Initial creation
 * Ian Haden 07/27/2018 -> Initial creation
 * Wyatt Ashley 03/03/2019 -> Initial creation
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

    private double mMotorMaxSpeed = 1;
    private boolean gyroAssistEnabled = false;

    //set up the variables for the logger
    private FileLogger fileLogger = null;
    private static final String TAG = "HardwareDriveMotors";

    /* Constructor */
    public HardwareDriveMotors(){

    }

    public void init(FileLogger fileloggerhandle, HardwareMap ahwMap, robotConfigSettings.robotConfigChoice baseConfig, String motor1, String motor2, String motor3, String motor4, LibraryMotorType.MotorTypes motorTypes) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        this.fileLogger = fileloggerhandle;

        // Define and Initialize Motors
        if (motor1 != null)
            baseMotor1  = hwMap.dcMotor.get(motor1);
        if (motor2 != null)
            baseMotor2  = hwMap.dcMotor.get(motor2);
        if (motor3 != null)
            baseMotor3  = hwMap.dcMotor.get(motor3);
        if (motor4 != null)
            baseMotor4  = hwMap.dcMotor.get(motor4);

        setHardwareDriveDirections(baseConfig, motorTypes);

        // Set all motors to zero power
        setHardwareDrivePower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        setHardwareDriveResetEncoders();

        setHardwareDriveRunUsingEncoders();
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void init(FileLogger fileloggerhandle, robotConfigSettings.robotConfigChoice baseConfig, HardwareMap ahwMap, String motor1, String motor2, String motor3, String motor4, LibraryMotorType.MotorTypes motorTypes) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        this.fileLogger = fileloggerhandle;

        // Define and Initialize Motors
        if (motor1 != null)
            baseMotor1  = hwMap.dcMotor.get(motor1);
        if (motor2 != null)
            baseMotor2  = hwMap.dcMotor.get(motor2);
        if (motor3 != null)
            baseMotor3  = hwMap.dcMotor.get(motor3);
        if (motor4 != null)
            baseMotor4  = hwMap.dcMotor.get(motor4);

        setHardwareDriveDirections(baseConfig, motorTypes);
        // Set all motors to zero power
        setHardwareDrivePower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        setHardwareDriveResetEncoders();

        setHardwareDriveRunUsingEncoders();

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /* Initialize standard Hardware interfaces */
    public void init(FileLogger fileloggerhandle, HardwareMap ahwMap, robotConfigSettings.robotConfigChoice baseConfig, LibraryMotorType.MotorTypes motorTypes) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        this.fileLogger = fileloggerhandle;

        initMotorDefaults(baseConfig, motorTypes);

        setHardwareDriveDirections(baseConfig, motorTypes);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, robotConfigSettings.robotConfigChoice baseConfig, LibraryMotorType.MotorTypes motorTypes) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        initMotorDefaults();

        setHardwareDriveDirections(baseConfig, motorTypes);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initMotorDefaults () {

        // Define and Initialize Motors
        baseMotor1  = hwMap.dcMotor.get(robotConfig.motors.leftMotor1.toString());
        baseMotor2  = hwMap.dcMotor.get(robotConfig.motors.leftMotor2.toString());
        baseMotor3  = hwMap.dcMotor.get(robotConfig.motors.rightMotor1.toString());
        baseMotor4  = hwMap.dcMotor.get(robotConfig.motors.rightMotor2.toString());

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set all motors to zero power
        setHardwareDrivePower(0);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        setHardwareDriveResetEncoders();
        setHardwareDriveRunUsingEncoders();

    }

    private void initMotorDefaults (robotConfigSettings.robotConfigChoice baseConfig, LibraryMotorType.MotorTypes motorTypes) {

        // Define and Initialize Motors
        baseMotor1  = hwMap.dcMotor.get(robotConfig.motors.leftMotor1.toString());
        baseMotor2  = hwMap.dcMotor.get(robotConfig.motors.leftMotor2.toString());
        baseMotor3  = hwMap.dcMotor.get(robotConfig.motors.rightMotor1.toString());
        baseMotor4  = hwMap.dcMotor.get(robotConfig.motors.rightMotor2.toString());

        setHardwareDriveDirections(baseConfig, motorTypes);
        // Set all motors to zero power
        setHardwareDrivePower(0);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        setHardwareDriveResetEncoders();
        setHardwareDriveRunUsingEncoders();
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior){
        if (baseMotor1 != null)
            baseMotor1.setZeroPowerBehavior(zeroPowerBehavior);
        if (baseMotor2 != null)
            baseMotor2.setZeroPowerBehavior(zeroPowerBehavior);
        if (baseMotor3 != null)
            baseMotor3.setZeroPowerBehavior(zeroPowerBehavior);
        if (baseMotor4 != null)
            baseMotor4.setZeroPowerBehavior(zeroPowerBehavior);
    }
    public void setHardwareDriveDirections(robotConfigSettings.robotConfigChoice baseConfig, LibraryMotorType.MotorTypes motorTypes){
        switch (baseConfig) {
            case TileRunnerRegular:
                if (baseMotor1 != null)
                    baseMotor1.setDirection(DcMotor.Direction.REVERSE);
                if (baseMotor2 != null)
                    baseMotor2.setDirection(DcMotor.Direction.REVERSE);
                if (baseMotor3 != null)
                    baseMotor3.setDirection(DcMotor.Direction.FORWARD);
                if (baseMotor4 != null)
                    baseMotor4.setDirection(DcMotor.Direction.FORWARD);
                break;
            case TileRunnerRegularOrbital:
                if (baseMotor1 != null)
                    baseMotor1.setDirection(DcMotor.Direction.REVERSE);
                if (baseMotor2 != null)
                    baseMotor2.setDirection(DcMotor.Direction.REVERSE);
                if (baseMotor3 != null)
                    baseMotor3.setDirection(DcMotor.Direction.FORWARD);
                if (baseMotor4 != null)
                    baseMotor4.setDirection(DcMotor.Direction.FORWARD);
                break;
            case TileRunnerMecanum:
                //TOWR5291 Tilrunner has 2 motors running from belts to the wheel, 2 motors running on gears
                if (baseMotor1 != null)
                    baseMotor1.setDirection(DcMotor.Direction.FORWARD);
                if (baseMotor2 != null)
                    baseMotor2.setDirection(DcMotor.Direction.REVERSE);
                if (baseMotor3 != null)
                    baseMotor3.setDirection(DcMotor.Direction.REVERSE);
                if (baseMotor4 != null)
                    baseMotor4.setDirection(DcMotor.Direction.FORWARD);
                break;
            case TileRunnerMecanumOrbital:
                //TOWR5291 Tilrunner has 2 motors running from belts to the wheel, 2 motors running on gears
                if (motorTypes.isAndyMark()) {
                    if (baseMotor1 != null)
                        baseMotor1.setDirection(DcMotor.Direction.REVERSE);
                    if (baseMotor2 != null)
                        baseMotor2.setDirection(DcMotor.Direction.FORWARD);
                    if (baseMotor3 != null)
                        baseMotor3.setDirection(DcMotor.Direction.FORWARD);
                    if (baseMotor4 != null)
                        baseMotor4.setDirection(DcMotor.Direction.REVERSE);
                } else {
                    if (baseMotor1 != null)
                        baseMotor1.setDirection(DcMotor.Direction.REVERSE);
                    if (baseMotor2 != null)
                        baseMotor2.setDirection(DcMotor.Direction.FORWARD);
                    if (baseMotor3 != null)
                        baseMotor3.setDirection(DcMotor.Direction.FORWARD);
                    if (baseMotor4 != null)
                        baseMotor4.setDirection(DcMotor.Direction.REVERSE);
                }
                break;
            case TiltRunnerOmni:
                if (baseMotor1 != null)
                    baseMotor1.setDirection(DcMotor.Direction.REVERSE);
                if (baseMotor2 != null)
                    baseMotor2.setDirection(DcMotor.Direction.REVERSE);
                if (baseMotor3 != null)
                    baseMotor3.setDirection(DcMotor.Direction.FORWARD);
                if (baseMotor4 != null)
                    baseMotor4.setDirection(DcMotor.Direction.FORWARD);
                break;
            default:
                if (baseMotor1 != null)
                    baseMotor1.setDirection(DcMotor.Direction.FORWARD);
                if (baseMotor2 != null)
                    baseMotor2.setDirection(DcMotor.Direction.FORWARD);
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


    public boolean getHardwareBaseDriveBusy() {
        boolean blnDrive1 = false;
        boolean blnDrive2 = false;
        boolean blnDrive3 = false;
        boolean blnDrive4 = false;

        if (baseMotor1 != null)
            blnDrive1 = (baseMotor1.isBusy());
        if (baseMotor2 != null)
            blnDrive2 = (baseMotor2.isBusy());
        if (baseMotor3 != null)
            blnDrive3 = (baseMotor3.isBusy());
        if (baseMotor4 != null)
            blnDrive4 = (baseMotor4.isBusy());

        return (blnDrive1) || (blnDrive2) || (blnDrive3) || (blnDrive4);
    }

    public boolean[] isBusy(){
        boolean MotorsBusy[] = new boolean[4];
        if (baseMotor1 != null) MotorsBusy[0] = baseMotor1.isBusy();
        if (baseMotor2 != null) MotorsBusy[1] = baseMotor2.isBusy();
        if (baseMotor3 != null) MotorsBusy[2] = baseMotor3.isBusy();
        if (baseMotor4 != null) MotorsBusy[3] = baseMotor4.isBusy();
        return MotorsBusy;
    }

    public void allMotorsStop(){
        fileLogger.writeEvent(4,"HardwareDriveMotors", "allMotorsStop()");

        baseMotor1.setPower(0);
        baseMotor2.setPower(0);
        baseMotor3.setPower(0);
        baseMotor4.setPower(0);
    }

    public void setTarget(int moveCounts){

        fileLogger.writeEvent(4,"HardwareDriveMotors", "setTarget() " + moveCounts);
        baseMotor1.setTargetPosition(baseMotor1.getCurrentPosition() + moveCounts);
        baseMotor2.setTargetPosition(baseMotor2.getCurrentPosition() + moveCounts);
        baseMotor3.setTargetPosition(baseMotor3.getCurrentPosition() + moveCounts);
        baseMotor4.setTargetPosition(baseMotor4.getCurrentPosition() + moveCounts);
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

        public void setMotor1EncoderValue (int value) { this.motor1 = value; }

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
        fileLogger.writeEvent(4,"HardwareDriveMotors", "setHardwareDriveResetEncoders()");
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
        fileLogger.writeEvent(4,"HardwareDriveMotors", "setHardwareDriveRunUsingEncoders()");
        setHardwareDriveLeftRunUsingEncoders();
        setHardwareDriveRightRunUsingEncoders();
    }

    public void setHardwareDriveLeftRunUsingEncoders() {
        fileLogger.writeEvent(4,"HardwareDriveMotors", "setHardwareDriveLeftRunUsingEncoders()");
        if (baseMotor1 != null)
            baseMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (baseMotor2 != null)
            baseMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setHardwareDriveRightRunUsingEncoders() {
        fileLogger.writeEvent(4,"HardwareDriveMotors", "setHardwareDriveRightRunUsingEncoders()");
        if (baseMotor3 != null)
            baseMotor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (baseMotor4 != null)
            baseMotor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setHardwareDriveRunWithoutEncoders() {
        fileLogger.writeEvent(4,"HardwareDriveMotors", "setHardwareDriveRunWithoutEncoders()");
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
        fileLogger.writeEvent(4,"HardwareDriveMotors", "setHardwareDriveRunToPosition()");
        setHardwareDriveLeftRunToPosition();
        setHardwareDriveRightRunToPosition();
    }

    public void logEncoderCounts(FileLogger MasterFileLogger){
        this.fileLogger = MasterFileLogger;
        fileLogger.writeEvent(3,"baseMotor1 Encoder Counts" + String.valueOf(baseMotor1.getCurrentPosition()));
        fileLogger.writeEvent(3,"baseMotor2 Encoder Counts" + String.valueOf(baseMotor2.getCurrentPosition()));
        fileLogger.writeEvent(3,"baseMotor3 Encoder Counts" + String.valueOf(baseMotor3.getCurrentPosition()));
        fileLogger.writeEvent(3,"baseMotor4 Encoder Counts" + String.valueOf(baseMotor4.getCurrentPosition()));
    }

    public void logEncoderCounts(){
        if (this.fileLogger != null) {
            fileLogger.writeEvent(3, "baseMotor1 Encoder Counts" + String.valueOf(baseMotor1.getCurrentPosition()));
            fileLogger.writeEvent(3, "baseMotor2 Encoder Counts" + String.valueOf(baseMotor2.getCurrentPosition()));
            fileLogger.writeEvent(3, "baseMotor3 Encoder Counts" + String.valueOf(baseMotor3.getCurrentPosition()));
            fileLogger.writeEvent(3, "baseMotor4 Encoder Counts" + String.valueOf(baseMotor4.getCurrentPosition()));
        }
    }

    public void setHardwareDriveLeftRunToPosition() {
        fileLogger.writeEvent(4,"HardwareDriveMotors", "setHardwareDriveLeftRunToPosition()");
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

    /**
     * Set the power of the motors on the robot
     *
     * @param leftFront set the power of right 1 motor.
     * @param leftBack set the power of right 1 motor.
     * @param rightFront set the power of right 1 motor.
     * @param rightBack set the power of right 1 motor.
     */
    public void setHardwareDrivePower (double leftFront, double leftBack, double rightFront, double rightBack) {

        fileLogger.writeEvent(4,"HardwareDriveMotors", "setHardwareDrivePower(): -" +leftFront + ", " + leftBack + ", " +rightFront + ", " + rightBack );
        if (baseMotor1 != null)
            baseMotor1.setPower(checkMotorMaxPower(leftFront));
        if (baseMotor2 != null)
            baseMotor2.setPower(checkMotorMaxPower(leftBack));
        if (baseMotor3 != null)
            baseMotor3.setPower(checkMotorMaxPower(rightFront));
        if (baseMotor4 != null)
            baseMotor4.setPower(checkMotorMaxPower(rightBack));
    }

    /**
     * Set the power of the motors for all 4 motors on the robot
     *
     * @param power set the power of all the motors on the base.
     */
    public void setHardwareDrivePower (double power) {
        fileLogger.writeEvent(4,"HardwareDriveMotors", "setHardwareDrivePower() " + power);
        setHardwareDriveLeftMotorPower(power);
        setHardwareDriveRightMotorPower(power);
    }

    /**
     * Set the power of the motors on the left of the robot
     *
     * @param power set the power of left motors.
     */
    public void setHardwareDriveLeftMotorPower (double power) {
        fileLogger.writeEvent(4,"HardwareDriveMotors", "setHardwareDriveLeftMotorPower() " + power);
        setHardwareDriveLeft1MotorPower(power);
        setHardwareDriveLeft2MotorPower(power);
    }

    /**
     * Set the power of the motors on the right of the robot
     *
     * @param power set the power of right motors.
     */
    public void setHardwareDriveRightMotorPower (double power) {
        fileLogger.writeEvent(4,"HardwareDriveMotors", "setHardwareDriveRightMotorPower() " + power);
        setHardwareDriveRight1MotorPower(power);
        setHardwareDriveRight2MotorPower(power);
    }

    private double checkMotorMaxPower(double power) {
        return TOWR5291Utils.scaleRange(power, -1 ,1, -this.mMotorMaxSpeed, this.mMotorMaxSpeed);
        //if (Math.abs(power) <= this.mMotorMaxSpeed) {
        //    return power;
        //} else {
        //    if (power < 0)
        //        return -this.mMotorMaxSpeed;
        //    else
        //        return this.mMotorMaxSpeed;
        //}
    }

    /**
     * Set the power of the motors on the left 1 of the robot
     *
     * @param power set the power of left 1 motor.
     */
    public void setHardwareDriveLeft1MotorPower (double power) {
        fileLogger.writeEvent(4,"HardwareDriveMotors", "setHardwareDriveLeft1MotorPower() " + power);
        if (baseMotor1 != null)
            baseMotor1.setPower(checkMotorMaxPower(power));
    }

    /**
     * Set the power of the motors on the left 2 of the robot
     *
     * @param power set the power of left 2 motor.
     */
    public void setHardwareDriveLeft2MotorPower (double power) {
        fileLogger.writeEvent(4,"HardwareDriveMotors", "setHardwareDriveLeft2MotorPower() " + power);
        if (baseMotor2 != null)
            baseMotor2.setPower(checkMotorMaxPower(power));
    }

    /**
     * Set the power of the motors on the right 1 of the robot
     *
     * @param power set the power of right 1 motor.
     */
    public void setHardwareDriveRight1MotorPower (double power) {
        fileLogger.writeEvent(4,"HardwareDriveMotors", "setHardwareDriveRight1MotorPower() " + power);
        if (baseMotor3 != null)
            baseMotor3.setPower(checkMotorMaxPower(power));
    }

    /**
     * Set the power of the motors on the right 2 of the robot
     *
     * @param power set the power of right 2 motor.
     */
    public void setHardwareDriveRight2MotorPower (double power) {
        fileLogger.writeEvent(4,"HardwareDriveMotors", "setHardwareDriveRight2MotorPower() " + power);
        if (baseMotor4 != null)
            baseMotor4.setPower(checkMotorMaxPower(power));
    }

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

    protected static double[] rotateVector(double x, double y, double angle) {
        double cosA = Math.cos(angle * (Math.PI / 180.0));
        double sinA = Math.sin(angle * (Math.PI / 180.0));
        double[] out = new double[2];
        out[0] = x * cosA - y * sinA;
        out[1] = x * sinA + y * cosA;
        return out;
    }

    /**
     * FROM https://github.com/wpilibsuite/allwpilib/blob/master/wpilibj/src/main/java/edu/wpi/first/wpilibj/RobotDrive.java
     * Normalize all wheel speeds if the magnitude of any wheel is greater than 1.0.
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

    /**
    * FROM https://github.com/wpilibsuite/allwpilib/blob/master/wpilibj/src/main/java/edu/wpi/first/wpilibj/RobotDrive.java
    * Drive method for Mecanum wheeled robots.
    *
    * <p>A method for driving with Mecanum wheeled robots. There are 4 wheels on the robot, arranged
    * so that the front and back wheels are toed in 45 degrees. When looking at the wheels from the
    * top, the roller axles should form an X across the robot.
    *
    * <p>This is designed to be directly driven by joystick axes.
    *
    * @param x         The speed that the robot should drive in the X direction. [-1.0..1.0]
    * @param y         The speed that the robot should drive in the Y direction. This input is
    *                  inverted to match the forward == -1.0 that joysticks produce. [-1.0..1.0]
    * @param rotation  The rate of rotation for the robot that is completely independent of the
    *                  translation. [-1.0..1.0]
    * @param gyroAngle The current angle reading from the gyro. Use this to implement field-oriented
    *                  controls.
    */
    public void mecanumDrive_Cartesian(double x, double y, double rotation, double gyroAngle) {

        double xIn = x;
        double yIn = y;
        double[] wheelSpeeds = new double[4];

        // Negate y for the joystick.
        yIn = -yIn;
        xIn = -xIn;

        // Compensate for gyro angle.rotateVector
        double[] rotated = rotateVector(xIn, yIn, -gyroAngle);
        xIn = rotated[0];
        yIn = rotated[1];

        wheelSpeeds[0] = xIn + yIn + rotation;
        wheelSpeeds[1] = -xIn + yIn - rotation;
        wheelSpeeds[2] = -xIn + yIn + rotation;
        wheelSpeeds[3] = xIn + yIn - rotation;

        normalize(wheelSpeeds);
        setHardwareDriveLeft1MotorPower(wheelSpeeds[0]);
        setHardwareDriveLeft2MotorPower(wheelSpeeds[1]);
        setHardwareDriveRight1MotorPower(wheelSpeeds[2]);
        setHardwareDriveRight2MotorPower(wheelSpeeds[3]);
    }

    public void mecanumDrive_Cartesian(double x, double y, double rotation, double gyroAngle, double speedToMultplayBy) {

        double xIn = x;
        double yIn = y;
        double[] wheelSpeeds = new double[4];

        // Negate y for the joystick.
        yIn = -yIn;
        xIn = -xIn;

        // Compensate for gyro angle.rotateVector
        double[] rotated = rotateVector(xIn, yIn, -gyroAngle);
        xIn = rotated[0];
        yIn = rotated[1];

        wheelSpeeds[0] = xIn + yIn + rotation;
        wheelSpeeds[1] = -xIn + yIn - rotation;
        wheelSpeeds[2] = -xIn + yIn + rotation;
        wheelSpeeds[3] = xIn + yIn - rotation;

        normalize(wheelSpeeds);
        setHardwareDriveLeft1MotorPower(wheelSpeeds[0] * Range.clip(speedToMultplayBy, -1.0, 1.0));
        setHardwareDriveLeft2MotorPower(wheelSpeeds[1] * Range.clip(speedToMultplayBy, -1.0, 1.0));
        setHardwareDriveRight1MotorPower(wheelSpeeds[2] * Range.clip(speedToMultplayBy, -1.0, 1.0));
        setHardwareDriveRight2MotorPower(wheelSpeeds[3] * Range.clip(speedToMultplayBy, -1.0, 1.0));
    }
    /**
     * FROM https://github.com/wpilibsuite/allwpilib/blob/master/wpilibj/src/main/java/edu/wpi/first/wpilibj/RobotDrive.java
     * Configure the scaling factor for using RobotDrive with motor controllers in a mode other than
     * PercentVbus.
     *
     * @param maxOutput Multiplied with the output percentage computed by the drive functions.
     */
    public void setMaxOutput(double maxOutput) {
         this.mMotorMaxSpeed = maxOutput;
    }

    public double getMaxOutput() {
        return this.mMotorMaxSpeed;
    }
}

