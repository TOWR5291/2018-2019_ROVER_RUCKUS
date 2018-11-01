package club.towr5291.robotconfig;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.function.Function;

import club.towr5291.functions.FileLogger;
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
    /* Public OpMode members. */
    public DcMotor  liftMotor1      = null;
    public DcMotor  liftMotor2      = null;
    public DcMotor  tiltMotor1      = null;
    public Servo teamMarkerServo    = null;
    public Servo intakeServo        = null;

    /* local OpMode members. */
    HardwareMap hwMap               =  null;
    private TOWRDashBoard dashBoard = null;

    /* Constructor */
    public HardwareArmMotorsRoverRuckus(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, TOWRDashBoard dash) {

        this.dashBoard = dash;
        this.hwMap = ahwMap;

        // Define and Initialize Motors
        this.liftMotor1         = hwMap.dcMotor.get("liftMotor1");
        this.liftMotor2         = hwMap.dcMotor.get("liftMotor2");
        this.tiltMotor1         = hwMap.dcMotor.get("tiltMotor1");
        this.teamMarkerServo    = hwMap.servo.get("teamMarkerServo");
        this.intakeServo        = hwMap.servo.get("intakeServo");
        setHardwareArmDirections();

        liftMotor1.setPower(0);
        liftMotor2.setPower(0);
        setHardwareLiftMotorResetEncoders();
        setHardwareLiftMotorRunWithoutEncoders();
        setHardwareLiftMotorRunUsingEncoders();
    }

    public void setHardwareArmDirections(){
        liftMotor1.setDirection(DcMotor.Direction.REVERSE);
        liftMotor2.setDirection(DcMotor.Direction.REVERSE);
        tiltMotor1.setDirection(DcMotor.Direction.FORWARD);
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

    public void AdvancedOptionsForArms (Gamepad gamepad, int lineDisplay){
        dashBoard.displayPrintf(lineDisplay, "GamePad B Advanced Options");
        if (gamepad.a){
            dashBoard.displayPrintf(lineDisplay + 1, "RESETING LIFT STOP MOVING NOW!!!");
        }
    }

}
