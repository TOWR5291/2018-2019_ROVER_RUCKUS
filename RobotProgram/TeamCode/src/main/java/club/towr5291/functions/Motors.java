package club.towr5291.functions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import club.towr5291.libraries.robotConfigSettings;


/**
 * Created by Wyatt on 6/27/2018.
 */
public class Motors {

    public FileLogger fileLogger;
    public static DcMotor baseMotor1 = null;
    public static DcMotor baseMotor2 = null;
    public static DcMotor baseMotor3 = null;
    public static DcMotor baseMotor4 = null;

    private enum Bases {
        Mecanum_TileRunner_Orbit_20,
        Mecanum_TileRunner_Classic_20,
        Mecanum_TileRunner_Classic_40,
        TileRunner_40,
        TileRunner_Classic_20,
        TileRunner_Orbit_20,
        TankDrive_40,
        TankDrive_Orbit_20,
        TankDrive_Classic_20
    }

    public void setDirection(Bases base){
        switch (base) {
            case Mecanum_TileRunner_Orbit_20:
                baseMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
                baseMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
                baseMotor3.setDirection(DcMotorSimple.Direction.FORWARD);
                baseMotor4.setDirection(DcMotorSimple.Direction.FORWARD);
                break;

            case Mecanum_TileRunner_Classic_20:
                baseMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
                baseMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
                baseMotor3.setDirection(DcMotorSimple.Direction.FORWARD);
                baseMotor4.setDirection(DcMotorSimple.Direction.FORWARD);
                break;

            case Mecanum_TileRunner_Classic_40:
                baseMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
                baseMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
                baseMotor3.setDirection(DcMotorSimple.Direction.FORWARD);
                baseMotor4.setDirection(DcMotorSimple.Direction.FORWARD);
                break;

            case TileRunner_40:
                baseMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
                baseMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
                baseMotor3.setDirection(DcMotorSimple.Direction.FORWARD);
                baseMotor4.setDirection(DcMotorSimple.Direction.FORWARD);
                break;

            case TileRunner_Classic_20:
                baseMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
                baseMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
                baseMotor3.setDirection(DcMotorSimple.Direction.FORWARD);
                baseMotor4.setDirection(DcMotorSimple.Direction.FORWARD);
                break;

            case TileRunner_Orbit_20:
                baseMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
                baseMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
                baseMotor3.setDirection(DcMotorSimple.Direction.FORWARD);
                baseMotor4.setDirection(DcMotorSimple.Direction.FORWARD);
                break;

            case TankDrive_40:
                baseMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
                baseMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
                baseMotor3.setDirection(DcMotorSimple.Direction.FORWARD);
                baseMotor4.setDirection(DcMotorSimple.Direction.FORWARD);
                break;

            case TankDrive_Orbit_20:
                baseMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
                baseMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
                baseMotor3.setDirection(DcMotorSimple.Direction.FORWARD);
                baseMotor4.setDirection(DcMotorSimple.Direction.FORWARD);
                break;

            case TankDrive_Classic_20:
                baseMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
                baseMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
                baseMotor3.setDirection(DcMotorSimple.Direction.FORWARD);
                baseMotor4.setDirection(DcMotorSimple.Direction.FORWARD);
                break;
        }
    }

    public double getWheelDiameter(Bases base) {
        double wheel;
        switch (base) {
            case Mecanum_TileRunner_Orbit_20:
                wheel = 4;
                break;

            case Mecanum_TileRunner_Classic_20:
                wheel = 4;
                break;

            case Mecanum_TileRunner_Classic_40:
                wheel = 4;
                break;

            case TileRunner_40:
                wheel = 4;
                break;

            case TileRunner_Classic_20:
                wheel = 4;
                break;

            case TileRunner_Orbit_20:
                wheel = 4;
                break;

            case TankDrive_40:
                wheel = 4;
                break;

            case TankDrive_Orbit_20:
                wheel = 4;
                break;

            case TankDrive_Classic_20:
                wheel = 4;
                break;

            default:
                wheel = 4;
                break;
        }

        return wheel;
    }

    public double getCountsPerMotorRev(Bases base) {
        double Count;
        switch (base) {
            case Mecanum_TileRunner_Orbit_20:
                Count = 537.6;
                break;

            case Mecanum_TileRunner_Classic_20:
                Count = 140;
                break;

            case Mecanum_TileRunner_Classic_40:
                Count = 280;
                break;

            case TileRunner_40:
                Count = 280;
                break;

            case TileRunner_Classic_20:
                Count = 140;
                break;

            case TileRunner_Orbit_20:
                Count = 537.6;
                break;

            case TankDrive_40:
                Count = 280;
                break;

            case TankDrive_Orbit_20:
                Count = 537.6;
                break;

            case TankDrive_Classic_20:
                Count = 140;
                break;

            default:
                Count = 280;
                break;
        }

        return Count;
    }

    public boolean[] isBusy(){
        boolean MotorsBusy[] = new boolean[3];
        if (baseMotor1 != null) MotorsBusy[0] = baseMotor1.isBusy();
        if (baseMotor2 != null) MotorsBusy[1] = baseMotor2.isBusy();
        if (baseMotor3 != null) MotorsBusy[2] = baseMotor3.isBusy();
        if (baseMotor4 != null) MotorsBusy[3] = baseMotor4.isBusy();
        return MotorsBusy;
    }

    public void setHardwareDriveDirection(robotConfigSettings.robotConfigChoice baseConfig){
        switch (baseConfig) {
            case TileRunner2x20:
            case TileRunner2x40:
            case TileRunner2x60:
                if (baseMotor1 != null) baseMotor1.setDirection(DcMotor.Direction.FORWARD);
                if (baseMotor2 != null) baseMotor2.setDirection(DcMotor.Direction.REVERSE);
                if (baseMotor3 != null) baseMotor3.setDirection(DcMotor.Direction.FORWARD);
                if (baseMotor4 != null) baseMotor4.setDirection(DcMotor.Direction.REVERSE);
                break;
            case TileRunnerMecanum2x40:
                if (baseMotor1 != null) baseMotor1.setDirection(DcMotor.Direction.FORWARD);
                if (baseMotor2 != null) baseMotor2.setDirection(DcMotor.Direction.REVERSE);
                if (baseMotor3 != null) baseMotor3.setDirection(DcMotor.Direction.REVERSE);
                if (baseMotor4 != null) baseMotor4.setDirection(DcMotor.Direction.FORWARD);
                break;
            default:
                if (baseMotor1 != null) baseMotor1.setDirection(DcMotor.Direction.FORWARD);
                if (baseMotor2 != null) baseMotor2.setDirection(DcMotor.Direction.REVERSE);
                if (baseMotor3 != null) baseMotor3.setDirection(DcMotor.Direction.FORWARD);
                if (baseMotor4 != null) baseMotor4.setDirection(DcMotor.Direction.FORWARD);
                break;
        }
    }


    public void allMotorsStop(){
        baseMotor1.setPower(0);
        baseMotor2.setPower(0);
        baseMotor3.setPower(0);
        baseMotor4.setPower(0);
    }

    public void logEncoderCounts(){
        fileLogger.writeEvent(2,"baseMotor1 Encoder Counts" + String.valueOf(baseMotor1.getCurrentPosition()));
        fileLogger.writeEvent(2,"baseMotor2 Encoder Counts" + String.valueOf(baseMotor2.getCurrentPosition()));
        fileLogger.writeEvent(2,"baseMotor3 Encoder Counts" + String.valueOf(baseMotor3.getCurrentPosition()));
        fileLogger.writeEvent(2,"baseMotor4 Encoder Counts" + String.valueOf(baseMotor4.getCurrentPosition()));
    }

    public void runUsingEncoder(){
        baseMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        baseMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        baseMotor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        baseMotor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runToPosition(){
        baseMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        baseMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        baseMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        baseMotor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setTarget(int moveCounts){
        baseMotor1.setTargetPosition(baseMotor1.getCurrentPosition() + moveCounts);
        baseMotor2.setTargetPosition(baseMotor2.getCurrentPosition() + moveCounts);
        baseMotor3.setTargetPosition(baseMotor3.getCurrentPosition() + moveCounts);
        baseMotor4.setTargetPosition(baseMotor4.getCurrentPosition() + moveCounts);
    }

    public void setSpeed(double motorSpeed){
        baseMotor1.setPower(motorSpeed);
        baseMotor2.setPower(motorSpeed);
        baseMotor3.setPower(motorSpeed);
        baseMotor4.setPower(motorSpeed);
    }

    public void stopResetEncoders(){
        baseMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        baseMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        baseMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        baseMotor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void initHardware(FileLogger fileLoggerFromMaster, HardwareMap ahwMap, String baseMotor1Name, String baseMotor2Name, String baseMotor3Name, String baseMotor4Name){
        this.fileLogger = fileLoggerFromMaster;
        baseMotor1 = ahwMap.get(DcMotor.class, baseMotor1Name);
        baseMotor2 = ahwMap.get(DcMotor.class, baseMotor2Name);
        baseMotor3 = ahwMap.get(DcMotor.class, baseMotor3Name);
        baseMotor4 = ahwMap.get(DcMotor.class, baseMotor4Name);
    }

    public int getEncoderPosition(BaseMotors baseMotor){
        int position = 0;
        switch (baseMotor){
            case baseMotor1:
                position = baseMotor1.getCurrentPosition();
            break;

            case baseMotor2:
                position = baseMotor2.getCurrentPosition();
            break;

            case baseMotor3:
                position = baseMotor3.getCurrentPosition();
            break;

            case baseMotor4:
                position = baseMotor4.getCurrentPosition();
            break;
        }
        return position;
    }

    private enum BaseMotors {
        baseMotor1,
        baseMotor2,
        baseMotor3,
        baseMotor4
    }
}
