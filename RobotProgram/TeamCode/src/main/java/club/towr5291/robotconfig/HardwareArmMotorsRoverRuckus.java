package club.towr5291.robotconfig;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.function.Function;

import club.towr5291.functions.Constants;
import club.towr5291.functions.FileLogger;
import club.towr5291.libraries.TOWR5291LEDControl;
import club.towr5291.libraries.TOWRDashBoard;
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
public class HardwareArmMotorsRoverRuckus
{
    public ElapsedTime elapse = new ElapsedTime();
    /* Public OpMode members. */
    public DcMotor  liftMotor1      = null;
    public DcMotor  liftMotor2      = null;
    public DcMotor  tiltMotor1      = null;
    public DcMotor  tiltMotor2      = null;
    public Servo intakeServo1       = null;
    public Servo intakeServo2       = null;
    public Servo teamMarkerServo    = null;

    /* local OpMode members. */
    HardwareMap hwMap               =  null;
    private TOWRDashBoard dashBoard = null;

    /* Constructor */
    public HardwareArmMotorsRoverRuckus(){
        elapse.startTime();
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, TOWRDashBoard dash) {

        this.dashBoard = dash;
        this.hwMap = ahwMap;

        // Define and Initialize Motors
        this.liftMotor1         = hwMap.dcMotor.get("liftMotor1");
        this.liftMotor2         = hwMap.dcMotor.get("liftMotor2");
        this.tiltMotor1         = hwMap.dcMotor.get("tiltMotor1");
        this.tiltMotor2         = hwMap.dcMotor.get("tiltMotor2");
        this.intakeServo1        = hwMap.servo.get("intakeServo1");
        this.intakeServo2        = hwMap.servo.get("intakeServo2");
        this.teamMarkerServo    = hwMap.servo.get("teamMarkerServo");
        setHardwareArmDirections();

        liftMotor1.setPower(0);
        liftMotor2.setPower(0);
        setHardwareLiftMotorResetEncoders();
        setHardwareLiftMotorRunWithoutEncoders();
        setHardwareLiftMotorRunUsingEncoders();
    }

    public void setHardwareArmDirections(){
        liftMotor1.setDirection(DcMotor.Direction.FORWARD);
        liftMotor2.setDirection(DcMotor.Direction.FORWARD);
        tiltMotor1.setDirection(DcMotor.Direction.REVERSE);
        tiltMotor2.setDirection(DcMotor.Direction.FORWARD);
        intakeServo1.setDirection(Servo.Direction.FORWARD);
        intakeServo2.setDirection(Servo.Direction.REVERSE);
    }

    public void setHardwareArmDirections(DcMotor.Direction direction){
        liftMotor1.setDirection(direction);
        liftMotor2.setDirection(direction);
    }

    public void setHardwareLiftPower(double power){
        liftMotor1.setPower(power);
        liftMotor2.setPower(power);
    }
    public void setHardwareLiftMotorResetEncoders() {
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setHardwareLiftMotorRunUsingEncoders() {
        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setHardwareLiftMotorRunWithoutEncoders() {
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setHardwareLiftMotorRunToPosition(){
        liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setTiltLiftPower(double power){
        if (tiltMotor1.getCurrentPosition() > 0 && tiltMotor1.getCurrentPosition() < 120){
            tiltMotor1.setPower(power);
        } else {
            tiltMotor1.setPower(0);
        }
    }

    public int getTilt2LiftEncoderCount() {
        return this.tiltMotor1.getCurrentPosition();
    }
    public int getTilt1LiftEncoderCount(){
        return this.tiltMotor2.getCurrentPosition();
    }
    public int getLiftMotor1Encoder() {
        return liftMotor1.getCurrentPosition();
    }
    public int getLiftMotor2Encoder() {
        return liftMotor2.getCurrentPosition();
    }

    public double getTilt2Speed(){
        return this.tiltMotor2.getPower();
    }
    public double getTilt1Speed(){
        return this.tiltMotor1.getPower();
    }
    public double getLift1Speed(){
        return this.liftMotor1.getPower();
    }
    public double getLift2Speed(){
        return this.liftMotor2.getPower();
    }

    public void setTiltMotorSpeed(double speed){
        this.tiltMotor1.setPower(speed);
        this.tiltMotor2.setPower(speed);
    }
    public void setTiltMotorsRUNTOPOSITION(){
        this.tiltMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.tiltMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setTiltMotorRESETENCODERS(){
        this.tiltMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.tiltMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setTiltMotorRUNUSINGENCODERS(){
        this.tiltMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.tiltMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setTiltMotorRUNWITHOUTENCODERS(){
        this.tiltMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.tiltMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setLiftMotorSpeed(double speed) {
        this.liftMotor1.setPower(speed);
        this.liftMotor2.setPower(speed);
    }
    public void setLiftMotorsRUNTOPOSITION(){
        this.liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setLiftMotorRESETENCODERS(){
        this.liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setLiftMotorRUNUSINGENCODERS(){
        this.liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setLiftMotorRUNWITHOUTENCODERS(){
        this.liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void allMotorsStop(){
        this.tiltMotor1.setPower(0);
        this.tiltMotor2.setPower(0);
        this.liftMotor1.setPower(0);
        this.liftMotor2.setPower(0);
        this.intakeServo1.setPosition(0);
        this.intakeServo2.setPosition(0);
    }
}