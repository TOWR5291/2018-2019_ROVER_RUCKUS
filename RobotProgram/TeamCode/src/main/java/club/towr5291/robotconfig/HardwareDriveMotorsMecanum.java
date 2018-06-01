package club.towr5291.robotconfig;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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
 * Motor channel:  Left  drive motor:        "leftmotor1"
 * Motor channel:  Left  drive motor:        "leftmotor2"
 * Motor channel:  Right drive motor:        "rightmotor1"
 * Motor channel:  Right drive motor:        "rightmotor2"
 */
public class HardwareDriveMotorsMecanum
{
    /* Public OpMode members. */
    public DcMotor  leftFront   = null;
    public DcMotor  leftBack   = null;
    public DcMotor  rightFront  = null;
    public DcMotor  rightBack  = null;

    /* local OpMode members. */
    HardwareMap hwMap            =  null;
    private ElapsedTime period   = new ElapsedTime();

    /* Constructor */
    public HardwareDriveMotorsMecanum(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFront   = hwMap.dcMotor.get("leftfront");
        leftBack   = hwMap.dcMotor.get("leftback");
        rightFront  = hwMap.dcMotor.get("rightfront");
        rightBack  = hwMap.dcMotor.get("rightback");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void HardwareDriveResetEncoders() {

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void HardwareDriveRunUsingEncoders() {

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    public void zeroMotors() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    //set the drive motors power, both left and right
//    public void setDriveMotorPower (double power) {
//        setDriveRightMotorPower(power);
//        setDriveLeftMotorPower(power);
//    }

    //set the right drive motors power
//    public void setDriveRightMotorPower (double power) {
//        rightMotor1.setPower(power);
//        rightMotor2.setPower(power);
//    }
//
//    //set the left motors drive power
//    public void setDriveLeftMotorPower (double power) {
//        leftMotor1.setPower(power);
//        leftMotor2.setPower(power);
//    }

}

