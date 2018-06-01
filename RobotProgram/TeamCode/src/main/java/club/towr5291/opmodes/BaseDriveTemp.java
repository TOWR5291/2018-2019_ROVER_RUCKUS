package club.towr5291.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import club.towr5291.robotconfig.HardwareDriveMotorsBaseConfig;

/**
 * Created by Andrew Haselton on 10/15/2016 at 9:53 AM.
 */
@TeleOp(name = "Base Drive Temp", group = "Andrew")
@Disabled
public class BaseDriveTemp extends OpMode{

    HardwareDriveMotorsBaseConfig robotDrive   = new HardwareDriveMotorsBaseConfig();   // Use base drive hardware configuration
//    DcMotor leftMotor1;
//    DcMotor leftMotor2;
//    DcMotor rightMotor1;
//    DcMotor rightMotor2;


    I2cAddr rangeLeftAddress = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    public static final int rangeLeft_reg_start = 0x28; //Register to start reading
    public static final int rangeLeft_read_length = 2; //Number of byte to read
    public I2cDevice rangeLeft;
    public I2cDeviceSynch rangeLeftReader;

    I2cAddr rangeRightAddress = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    public static final int rangeRight_reg_start = 0x30; //Register to start reading
    public static final int rangeRight_read_length = 2; //Number of byte to read
    public I2cDevice rangeRight;
    public I2cDeviceSynch rangeRightReader;

    byte[] rangeLeftCache; //The read will return an array of bytes. They are stored in this variable
    byte[] rangeRightCache; //The read will return an array of bytes. They are stored in this variable

    double counter = 0;


    double max = 1;
    double leftPow = 0;
    boolean leftNegative = false;
    double rightPow = 0;
    boolean rightNegative = false;
    double distance = 8; // range sensor distance, in feet
    double distanceLeft = 8;
    double distanceRight = 8;



    @Override
    public void init() {
//        leftMotor1.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
//        leftMotor1   = hardwareMap.dcMotor.get("leftmotor1");
//        leftMotor2   = hardwareMap.dcMotor.get("leftmotor2");
//        rightMotor1  = hardwareMap.dcMotor.get("rightmotor1");
//        rightMotor2  = hardwareMap.dcMotor.get("rightmotor2");
//        leftMotor2.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
//        rightMotor1.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
//        rightMotor2.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
//
//        // Set all motors to zero power
//        leftMotor1.setPower(0);
//        leftMotor2.setPower(0);
//        rightMotor1.setPower(0);
//        rightMotor2.setPower(0);


    }

    @Override
    public void loop() {

        hardwareMap.logDevices();

        rangeLeft = hardwareMap.i2cDevice.get("rangeLeft");
        rangeLeftReader = new I2cDeviceSynchImpl(rangeLeft, rangeLeftAddress, false);
        rangeLeftReader.engage();

        rangeRight = hardwareMap.i2cDevice.get("rangeRight");
        rangeRightReader = new I2cDeviceSynchImpl(rangeRight, rangeRightAddress, false);
        rangeRightReader.engage();

        rangeLeftCache = rangeLeftReader.read(rangeLeft_reg_start, rangeLeft_read_length);
        rangeRightCache = rangeRightReader.read(rangeRight_reg_start, rangeRight_read_length);

        leftPow =  gamepad1.left_stick_y;
        rightPow =  gamepad1.right_stick_y;

//        distanceLeft = (rangeLeft.cmUltrasonic()/2.54) / 12;

        if (distanceLeft < distanceRight) {
            distance = distanceLeft;
        } else {
            distance = distanceRight;
        }

        // calculate the new max
        if (distance >= 4) {
            max = 1;
        } else if (distance < 4 && distance >= 2) {
            max = distance * 0.3 - 0.2;
        } else if (distance < 2 && distance >= 1.5) {
            max = distance * 0.4 - 0.4;
        } else if (distance < 1.5 && distance >= 1) {
            max = distance * 0.2 - 0.1;
        } else if (distance < 0.01) {
            max = distance * 0.05 - 0.05;
        }


        if (leftPow < 0) {
            leftNegative = true;
        } else {
            leftNegative = false;
        }

        if (rightPow < 0) {
            rightNegative = true;
        } else {
            rightNegative = false;
        }

        if (leftPow >= max && leftNegative == false) {
            leftPow = max;
        } else if (leftPow <= -max && leftNegative == true) {
            leftPow = -max;
        }

        if (rightPow >= max && rightNegative == false) {
            rightPow = max;
        } else if (rightPow <= -max && rightNegative == true) {
            rightPow = -max;
        }
        //leftMotor1.setPower(-leftPow);
        //leftMotor2.setPower(-leftPow);
        //rightMotor1.setPower(-rightPow);
        //rightMotor2.setPower(-rightPow);


        counter++;

        telemetry.addData("Max:", max);
        telemetry.addData("Left Power", leftPow);
        telemetry.addData("Right Power",rightPow);
        telemetry.addData("Distance", distance);
//        telemetry.addData("raw ultrasonic left", rangeLeft.rawUltrasonic());
//        telemetry.addData("raw optical left", rangeLeft.rawOptical());
//        telemetry.addData("raw ultrasonic right", rangeRight.rawUltrasonic());
//        telemetry.addData("raw optical right", rangeRight.rawOptical());
     //   telemetry.addData("cm optical", "%.2f cm", range1.cmOptical());
     //   telemetry.addData("cm", "%.2f cm", range1.getDistance(DistanceUnit.CM));

        telemetry.addData("Ultra Sonic", rangeRightCache[0] & 0xFF);
        telemetry.addData("ODS", rangeRightCache[1] & 0xFF);



        telemetry.addData("Counter:", counter);


    }
}
