package club.towr5291.robotconfig;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


import club.towr5291.functions.FileLogger;
import club.towr5291.libraries.robotConfigSettings;

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
public class HardwareArmMotors
{
    /* Public OpMode members. */
    public DcMotor  armMotor1  = null;
    public DcMotor  armMotor2  = null;
    //public DcMotor  armMotor3  = null;
    //public DcMotor  armMotor4  = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareArmMotors(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, robotConfigSettings.robotConfigChoice baseConfig) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        armMotor1  = hwMap.dcMotor.get("lifttop");
        armMotor2  = hwMap.dcMotor.get("liftbot");
        //armMotor3  = hwMap.dcMotor.get("relicslide");
        //armMotor4  = hwMap.dcMotor.get("rightMotor2");

        setHardwareArmDirections();

        // Set all motors to zero power
        setHardwareArmPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        setHardwareArmResetEncoders();

        setHardwareArmRunWithoutEncoders();

        setHardwareArmLiftRunUsingEncoders();
    }

    /* Initialize standard Hardware interfaces */
    public void init(FileLogger fileloggerhandle, HardwareMap ahwMap, robotConfigSettings.robotConfigChoice baseConfig) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        armMotor1  = hwMap.dcMotor.get("lifttop");
        armMotor2  = hwMap.dcMotor.get("liftbot");
        //armMotor3  = hwMap.dcMotor.get("relicslide");
        //armMotor4  = hwMap.dcMotor.get("rightMotor2");

        setHardwareArmDirections();

        // Set all motors to zero power
        setHardwareArmPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        setHardwareArmResetEncoders();

        setHardwareArmRunWithoutEncoders();

        setHardwareArmLiftRunUsingEncoders();
    }

    public void setHardwareArmDirections(){

        armMotor1.setDirection(DcMotor.Direction.REVERSE);
        armMotor2.setDirection(DcMotor.Direction.REVERSE);
        //armMotor3.setDirection(DcMotor.Direction.FORWARD);
        //armMotor4.setDirection(DcMotor.Direction.REVERSE);
    }

    public int getHardwareArmIsBusy() {
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

        int myInt1 = (armMotor1.isBusy()) ? 1 : 0;
        int myInt2 = (armMotor2.isBusy()) ? 1 : 0;
        int myInt3 = 0;//(armMotor3.isBusy()) ? 1 : 0;
        int myInt4 = 0;//(armMotor4.isBusy()) ? 1 : 0;

        return (myInt1) + (2 * myInt2) + (4 * myInt3)  + (8 * myInt4);

    }

    public void setHardwareArmResetEncoders() {
        armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armMotor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setHardwareArmRunUsingEncoders() {
        setHardwareArmLiftRunUsingEncoders();
        setHardwareArmRelicRunUsingEncoders();
    }

    public void setHardwareArmLiftRunUsingEncoders() {
        setHardwareArmLift1RunUsingEncoders();
        setHardwareArmLift2RunUsingEncoders();
    }

    public void setHardwareArmRelicRunUsingEncoders() {
        setHardwareArmRelic1RunUsingEncoders();
        setHardwareArmRelic2RunUsingEncoders();
    }

    public void setHardwareArmLift1RunUsingEncoders() {
        armMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setHardwareArmLift2RunUsingEncoders() {
        armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setHardwareArmRelic1RunUsingEncoders() {
        //armMotor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setHardwareArmRelic2RunUsingEncoders() {
        //armMotor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setHardwareArmRunWithoutEncoders() {
        setHardwareArmLiftRunWithoutEncoders();
        setHardwareArmRelicRunWithoutEncoders();
    }

    public void setHardwareArmLiftRunWithoutEncoders() {
        setHardwareArmLift1RunWithoutEncoders();
        setHardwareArmLift2RunWithoutEncoders();
    }

    public void setHardwareArmRelicRunWithoutEncoders() {
        setHardwareArmRelic1RunWithoutEncoders();
        setHardwareArmRelic2RunWithoutEncoders();
    }

    public void setHardwareArmLift1RunWithoutEncoders() {
        armMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setHardwareArmLift2RunWithoutEncoders() {
        armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setHardwareArmRelic1RunWithoutEncoders() {
        //armMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setHardwareArmRelic2RunWithoutEncoders() {
        //armMotor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setHardwareArmRunToPosition() {
        setHardwareArmLiftRunToPosition();
        setHardwareArmRelicRunToPosition();
    }

    public void setHardwareArmLiftRunToPosition() {
        setHardwareArmLift1RunToPosition();
        setHardwareArmLift2RunToPosition();
    }

    public void setHardwareArmRelicRunToPosition() {
        setHardwareArmRelic1RunToPosition();
        setHardwareArmRelic2RunToPosition();
    }

    public void setHardwareArmLift1RunToPosition() {
        armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);;
    }

    public void setHardwareArmLift2RunToPosition() {
        armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);;
    }

    public void setHardwareArmRelic1RunToPosition() {
        //armMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);;
    }

    public void setHardwareArmRelic2RunToPosition() {
        //armMotor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);;
    }

    public void setHardwareArmPower (double lift1, double lift2, double relic1, double relic2) {
        armMotor1.setPower(lift1);
        armMotor2.setPower(lift2);
        //armMotor3.setPower(relic1);
        //armMotor4.setPower(relic2);
    }

    //set the drive motors power, both left and right
    public void setHardwareArmPower (double power) {
        setHardwareArmLiftMotorPower(power);
        setHardwareArmRelicMotorPower(power);
    }

    //set the left motors drive power
    public void setHardwareArmLiftMotorPower (double power) {
        setHardwareArmLift1MotorPower(power);
        setHardwareArmLift2MotorPower(power);
    }

    //set the right drive motors power
    public void setHardwareArmRelicMotorPower (double power) {
        setHardwareArmRelic1MotorPower(power);
        setHardwareArmRelic2MotorPower(power);
    }

    public void setHardwareArmLift1MotorPower (double power) {
        armMotor1.setPower(power);
    }

    public void setHardwareArmLift2MotorPower (double power) {
        armMotor2.setPower(power);
    }

    public void setHardwareArmRelic1MotorPower (double power) {
        //armMotor3.setPower(power);
    }

    public void setHardwareArmRelic2MotorPower (double power) {
        //armMotor4.setPower(power);
    }


}
