package club.towr5291.functions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import club.towr5291.functions.FileLogger;

/**
 * Created by Wyatt on 6/27/2018.
 */

public class Motors {
    static double WHEEL_DIAMETER;
    static double COUNTS_PER_MOTOR_REV;
    public class setBaseConfig{
        public void Mecanum_TileRunner_Orbit_20(){
            lm1.setDirection(DcMotorSimple.Direction.REVERSE);
            lm2.setDirection(DcMotorSimple.Direction.REVERSE);
            rm1.setDirection(DcMotorSimple.Direction.FORWARD);
            rm2.setDirection(DcMotorSimple.Direction.FORWARD);
            WHEEL_DIAMETER = 4;
            COUNTS_PER_MOTOR_REV = 537.6;
        }

        public void Mecanum_TileRunner_Classic_20() {
            lm1.setDirection(DcMotorSimple.Direction.REVERSE);
            lm2.setDirection(DcMotorSimple.Direction.REVERSE);
            rm1.setDirection(DcMotorSimple.Direction.FORWARD);
            rm2.setDirection(DcMotorSimple.Direction.FORWARD);
            WHEEL_DIAMETER = 4;
            COUNTS_PER_MOTOR_REV = 140;
        }
        public void Mecanum_TileRunner_Classic_40(){
            lm1.setDirection(DcMotorSimple.Direction.REVERSE);
            lm2.setDirection(DcMotorSimple.Direction.REVERSE);
            rm1.setDirection(DcMotorSimple.Direction.FORWARD);
            rm2.setDirection(DcMotorSimple.Direction.FORWARD);
            WHEEL_DIAMETER = 4;
            COUNTS_PER_MOTOR_REV = 280;
        }
        public void tileRunner_40(){
            lm1.setDirection(DcMotorSimple.Direction.REVERSE);
            lm2.setDirection(DcMotorSimple.Direction.REVERSE);
            rm1.setDirection(DcMotorSimple.Direction.FORWARD);
            rm2.setDirection(DcMotorSimple.Direction.FORWARD);
            WHEEL_DIAMETER = 4;
            COUNTS_PER_MOTOR_REV = 280;
        }
        public void tileRunner_Classic_20(){
            lm1.setDirection(DcMotorSimple.Direction.REVERSE);
            lm2.setDirection(DcMotorSimple.Direction.REVERSE);
            rm1.setDirection(DcMotorSimple.Direction.FORWARD);
            rm2.setDirection(DcMotorSimple.Direction.FORWARD);
            WHEEL_DIAMETER = 4;
            COUNTS_PER_MOTOR_REV = 140;
        }
        public void tileRunner_Orbit_20(){
            lm1.setDirection(DcMotorSimple.Direction.REVERSE);
            lm2.setDirection(DcMotorSimple.Direction.REVERSE);
            rm1.setDirection(DcMotorSimple.Direction.FORWARD);
            rm2.setDirection(DcMotorSimple.Direction.FORWARD);
            WHEEL_DIAMETER = 4;
            COUNTS_PER_MOTOR_REV = 537.6;
        }
        public void tankDrive_40(){
            lm1.setDirection(DcMotorSimple.Direction.REVERSE);
            lm2.setDirection(DcMotorSimple.Direction.REVERSE);
            rm1.setDirection(DcMotorSimple.Direction.FORWARD);
            rm2.setDirection(DcMotorSimple.Direction.FORWARD);
            WHEEL_DIAMETER = 4;
            COUNTS_PER_MOTOR_REV = 280;
        }
        public void tankDrive_Orbit_20(){
            lm1.setDirection(DcMotorSimple.Direction.REVERSE);
            lm2.setDirection(DcMotorSimple.Direction.REVERSE);
            rm1.setDirection(DcMotorSimple.Direction.FORWARD);
            rm2.setDirection(DcMotorSimple.Direction.FORWARD);
            WHEEL_DIAMETER = 4;
            COUNTS_PER_MOTOR_REV = 537.6;
        }
        public void tankDrive_Classic_20(){
            lm1.setDirection(DcMotorSimple.Direction.REVERSE);
            lm2.setDirection(DcMotorSimple.Direction.REVERSE);
            rm1.setDirection(DcMotorSimple.Direction.FORWARD);
            rm2.setDirection(DcMotorSimple.Direction.FORWARD);
            WHEEL_DIAMETER = 4;
            COUNTS_PER_MOTOR_REV = 140;
        }
    }
    public FileLogger fileLogger;
    public static DcMotor lm1 = null;
    public static DcMotor lm2 = null;
    public static DcMotor rm1 = null;
    public static DcMotor rm2 = null;

    public void allMotorsStop(){
        lm1.setPower(0);
        lm2.setPower(0);
        rm1.setPower(0);
        rm2.setPower(0);
    }

    public void logEncoderCounts(){
        fileLogger.writeEvent(2,"LM1 Encoder Counts" + String.valueOf(lm1.getCurrentPosition()));
        fileLogger.writeEvent(2,"LM2 Encoder Counts" + String.valueOf(lm2.getCurrentPosition()));
        fileLogger.writeEvent(2,"RM1 Encoder Counts" + String.valueOf(rm1.getCurrentPosition()));
        fileLogger.writeEvent(2,"RM2 Encoder Counts" + String.valueOf(rm2.getCurrentPosition()));
    }

    public void runUsingEncoder(){
        lm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runToPosition(){
        lm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setTarget(int moveCounts){
        lm1.setTargetPosition(lm1.getCurrentPosition() + moveCounts);
        lm2.setTargetPosition(lm2.getCurrentPosition() + moveCounts);
        rm1.setTargetPosition(rm1.getCurrentPosition() + moveCounts);
        rm2.setTargetPosition(rm2.getCurrentPosition() + moveCounts);
    }

    public void setSpeed(double motorSpeed){
        lm1.setPower(motorSpeed);
        lm2.setPower(motorSpeed);
        rm1.setPower(motorSpeed);
        rm2.setPower(motorSpeed);
    }

    public void stopResetEncoders(){
        lm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void initHardware(FileLogger fileLoggerFromMaster, HardwareMap ahwMap, String lm1Name, String lm2Name, String rm1Name, String rm2Name){
        this.fileLogger = fileLoggerFromMaster;
        lm1 = ahwMap.get(DcMotor.class, lm1Name);
        lm2 = ahwMap.get(DcMotor.class, lm2Name);
        rm1 = ahwMap.get(DcMotor.class, rm1Name);
        rm2 = ahwMap.get(DcMotor.class, rm2Name);
    }
}
