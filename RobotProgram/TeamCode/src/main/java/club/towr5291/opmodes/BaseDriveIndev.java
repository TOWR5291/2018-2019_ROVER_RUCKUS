package club.towr5291.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import club.towr5291.robotconfig.HardwareDriveMotorsBaseConfig;

/**
 * Created by kids on 10/15/2016 at 9:53 AM.
 */
@TeleOp(name = "Base Drive In Dev", group = "Andrew")
@Disabled
public class BaseDriveIndev extends OpMode{

    HardwareDriveMotorsBaseConfig robotDrive   = new HardwareDriveMotorsBaseConfig();   // Use base drive hardware configuration
//    DcMotor leftMotor1;
//    DcMotor leftMotor2;
//    DcMotor rightMotor1;
//    DcMotor rightMotor2;


//    I2cAddr rangeLeftAddress = new I2cAddr(0x28); //Default I2C address for MR Range (7-bit)
//    public static final int rangeLeft_reg_start = 0x28; //Register to start reading
//    public static final int rangeLeft_read_length = 4; //Number of byte to read
//    public I2cDevice rangeLeft;
//    public I2cDeviceSynch rangeLeftReader;

//    I2cAddr rangeRightAddress = new I2cAddr(0x30); //Default I2C address for MR Range (7-bit)
//    public static final int rangeRight_reg_start = 0x30; //Register to start reading
//    public static final int rangeRight_read_length = 4; //Number of byte to read
//    public I2cDevice rangeRight;
//    public I2cDeviceSynch rangeRightReader;
//
//    byte[] rangeLeftCache; //The read will return an array of bytes. They are stored in this variable
//    byte[] rangeRightCache; //The read will return an array of bytes. They are stored in this variable

    double counter = 0;
    boolean slowdown = false;
    boolean reverse = false;

    double max = 1;
    double leftPow = 0;
    boolean leftNegative = false;
    double rightPow = 0;
    boolean rightNegative = false;
//    double distance = 8; // range sensor distance, in feet
//    double distanceLeft = 8;
//    double distanceRight = 8;



    @Override
    public void init() {
//        leftMotor1   = hardwareMap.dcMotor.get("leftmotor1");
//        leftMotor2   = hardwareMap.dcMotor.get("leftmotor2");
//        rightMotor1  = hardwareMap.dcMotor.get("rightmotor1");
//        rightMotor2  = hardwareMap.dcMotor.get("rightmotor2");
//        leftMotor1.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
//        leftMotor2.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
//        rightMotor1.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
//        rightMotor2.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
//
//        // Set all motors to zero power
//        leftMotor1.setPower(0);
//        leftMotor2.setPower(0);
//        rightMotor1.setPower(0);
//        rightMotor2.setPower(0);



//        rangeLeft = hardwareMap.i2cDevice.get("rangeLeft");
//        rangeLeftReader = new I2cDeviceSynchImpl(rangeLeft, rangeLeftAddress, false);
//        rangeLeftReader.engage();

//        rangeRight = hardwareMap.i2cDevice.get("rangeRight");
//        rangeRightReader = new I2cDeviceSynchImpl(rangeRight, rangeRightAddress, false);
//        rangeRightReader.engage();
    }

    @Override
    public void loop() {

        hardwareMap.logDevices();

//        rangeLeftCache = rangeLeftReader.read(rangeLeft_reg_start, rangeLeft_read_length);
//        rangeRightCache = rangeRightReader.read(rangeRight_reg_start, rangeRight_read_length);



//        distanceLeft = (rangeLeft.cmUltrasonic()/2.54) / 12;

//        if (distanceLeft < distanceRight) {
//            distance = distanceLeft;
//        } else {
//            distance = distanceRight;
//        }

        // calculate the new max
//        if (distance >= 4) {
//            max = 1;
//        } else if (distance < 4 && distance >= 2) {
//            max = distance * 0.3 - 0.2;
//        } else if (distance < 2 && distance >= 1.5) {
//            max = distance * 0.4 - 0.4;
//        } else if (distance < 1.5 && distance >= 1) {
//            max = distance * 0.2 - 0.1;
//        } else if (distance < 0.01) {
//            max = distance * 0.05 - 0.05;
//        }

        leftPow =  gamepad1.left_stick_y;
        rightPow =  gamepad1.right_stick_y;

//        if(reverse = false) {
//            leftPow =  gamepad1.left_stick_y;
//            rightPow =  gamepad1.right_stick_y;
//
//            //leftMotor1.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
//            //leftMotor2.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
//            //rightMotor1.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
//            //rightMotor2.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
//
//            if(gamepad1.left_bumper) {
//                reverse = true;
//
//                rightPow =  gamepad1.left_stick_y;
//                leftPow =  gamepad1.right_stick_y;
//
//                //leftMotor1.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
//                //leftMotor2.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
//                //rightMotor1.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
//                //rightMotor2.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
//            }
//
//        } else if(reverse = true) {
//
//            rightPow =  gamepad1.left_stick_y;
//            leftPow =  gamepad1.right_stick_y;
//
//            //leftMotor1.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
//            //leftMotor2.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
//            //rightMotor1.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
//            //rightMotor2.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
//            if(gamepad1.left_bumper) {
//                reverse = false;
//                //leftMotor1.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
//                //leftMotor2.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
//                //rightMotor1.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
//                //rightMotor2.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
//
//                leftPow =  gamepad1.left_stick_y;
//                rightPow =  gamepad1.right_stick_y;
//            }
//        }
//
//        if(slowdown) {
//            max = 0.5;
//            if(gamepad1.right_bumper) {
//                slowdown = false;
//            }
//        } else if (!slowdown) {
//            max = 1;
//            if(gamepad1.right_bumper) {
//                slowdown = true;
//            }
//        }


        if(gamepad1.left_bumper) {
            //leftMotor1.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
            //leftMotor2.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
            //rightMotor1.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
            //rightMotor2.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

            rightPow =  gamepad1.left_stick_y;
            leftPow =  gamepad1.right_stick_y;

            reverse = true;
        } else {
            leftPow =  gamepad1.left_stick_y;
            rightPow =  gamepad1.right_stick_y;

//                //leftMotor1.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
//                //leftMotor2.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
//                //rightMotor1.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
//                //rightMotor2.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

            reverse = false;
        }

        if(gamepad1.right_bumper) {
            max = 0.5;
            slowdown = true;
        } else {
            max = 1;
            slowdown = false;
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

//        telemetry.addData("Max:", max);
//        telemetry.addData("Left Power", leftPow);
//        telemetry.addData("Right Power",rightPow);
//        telemetry.addData("Distance", distance);
////        telemetry.addData("raw ultrasonic left", rangeLeft.rawUltrasonic());
////        telemetry.addData("raw optical left", rangeLeft.rawOptical());
////        telemetry.addData("raw ultrasonic right", rangeRight.rawUltrasonic());
////        telemetry.addData("raw optical right", rangeRight.rawOptical());
//     //   telemetry.addData("cm optical", "%.2f cm", range1.cmOptical());
//     //   telemetry.addData("cm", "%.2f cm", range1.getDistance(DistanceUnit.CM));
//
//        telemetry.addData("Ultra Sonic", rangeRightCache[0] & 0xFF);
//        telemetry.addData("ODS", rangeRightCache[1] & 0xFF);


        telemetry.addData("Slow?", slowdown);
        telemetry.addData("Reverse?", reverse);
        telemetry.addData("Counter:", counter);


    }
}
