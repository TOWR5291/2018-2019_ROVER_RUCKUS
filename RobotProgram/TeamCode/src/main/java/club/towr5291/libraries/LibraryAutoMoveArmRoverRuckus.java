package club.towr5291.libraries;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import club.towr5291.functions.Constants;
import club.towr5291.functions.FileLogger;
import club.towr5291.robotconfig.HardwareArmMotorsRoverRuckus;
import club.towr5291.robotconfig.HardwareSensors;
import club.towr5291.robotconfig.HardwareSensorsRoverRuckus;

public class LibraryAutoMoveArmRoverRuckus {
    private Constants.stepState stepState = Constants.stepState.STATE_COMPLETE;

    private HardwareArmMotorsRoverRuckus armMotorsRoverRuckus;
    private HardwareSensorsRoverRuckus sensorsRoverRuckus;

    private boolean haveACurrentValue = false;
    private double currentTiltEncoderCountMotor1 = 0;
    private double currentTiltEncoderCountMotor2 = 0;
    private double startingAngleMotor1Count;
    private double startingAngleMotor2Count;
    private double targetDegreeMoveToPosition = 0;

    private club.towr5291.libraries.robotConfig config;
    private Gamepad game2;

    private static final double LIMITSWITCH1DEGREEMEASURE = 0;
    private static final double LIMITSWITCH2DEGREEMEASURE = 0;
    private static final double LIMITSWITCH3DEGREEMEASURE = 0;
    private static final double LIMITSWITCH4DEGREEMEASURE = 0;
    private FileLogger fileLogger;
    private boolean hold;

    private static final double ANGLETOSCOREDEGREE = 45;

    public LibraryAutoMoveArmRoverRuckus(HardwareArmMotorsRoverRuckus armHardware, HardwareSensorsRoverRuckus sensorsRoverRuckus, club.towr5291.libraries.robotConfig robotconfig, Gamepad g2, FileLogger fileLogger){
        this.armMotorsRoverRuckus = armHardware;
        this.sensorsRoverRuckus = sensorsRoverRuckus;
        this.config = robotconfig;
        this.game2 = g2;
        this.fileLogger = fileLogger;
        this.stepState = Constants.stepState.STATE_COMPLETE;
        armMotorsRoverRuckus.tiltMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotorsRoverRuckus.tiltMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private double[] checkTiltCounts(){
        if (sensorsRoverRuckus.getLimitSwitch1AngleMotorState()){
            currentTiltEncoderCountMotor1 = (config.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH1DEGREEMEASURE);
            currentTiltEncoderCountMotor2 = (config.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH1DEGREEMEASURE);

            if (!haveACurrentValue){
                startingAngleMotor1Count = (config.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH1DEGREEMEASURE) - armMotorsRoverRuckus.tiltMotor1.getCurrentPosition();
                startingAngleMotor2Count = (config.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH1DEGREEMEASURE) - armMotorsRoverRuckus.tiltMotor2.getCurrentPosition();
                haveACurrentValue = true;
            }

        } else if (sensorsRoverRuckus.getLimitSwitch2AngleMotorState()){
            currentTiltEncoderCountMotor1 = (config.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH2DEGREEMEASURE);
            currentTiltEncoderCountMotor2 = (config.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH2DEGREEMEASURE);

            if (!haveACurrentValue){
                startingAngleMotor1Count = (config.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH2DEGREEMEASURE) - armMotorsRoverRuckus.tiltMotor1.getCurrentPosition();
                startingAngleMotor2Count = (config.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH2DEGREEMEASURE) - armMotorsRoverRuckus.tiltMotor2.getCurrentPosition();
                haveACurrentValue = true;
            }

        } else if (sensorsRoverRuckus.getLimitSwitch3AngleMotorState()){
            currentTiltEncoderCountMotor1 = (config.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH3DEGREEMEASURE);
            currentTiltEncoderCountMotor2 = (config.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH3DEGREEMEASURE);

            if (!haveACurrentValue){
                startingAngleMotor1Count = (config.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH3DEGREEMEASURE) - armMotorsRoverRuckus.tiltMotor1.getCurrentPosition();
                startingAngleMotor2Count = (config.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH3DEGREEMEASURE) - armMotorsRoverRuckus.tiltMotor2.getCurrentPosition();
                haveACurrentValue = true;
            }

        } else if (sensorsRoverRuckus.getLimitSwitch4AngleMotorState()){
            currentTiltEncoderCountMotor1 = (config.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH4DEGREEMEASURE);
            currentTiltEncoderCountMotor2 = (config.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH4DEGREEMEASURE);

            if (!haveACurrentValue){
                startingAngleMotor1Count = (config.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH4DEGREEMEASURE) - armMotorsRoverRuckus.tiltMotor1.getCurrentPosition();
                startingAngleMotor2Count = (config.getCOUNTS_PER_DEGREE_TILT() * LIMITSWITCH4DEGREEMEASURE) - armMotorsRoverRuckus.tiltMotor2.getCurrentPosition();
                haveACurrentValue = true;
            }
        } else {
            if (haveACurrentValue){
                currentTiltEncoderCountMotor1 = armMotorsRoverRuckus.tiltMotor1.getCurrentPosition() - startingAngleMotor1Count;
                currentTiltEncoderCountMotor2 = armMotorsRoverRuckus.tiltMotor2.getCurrentPosition() - startingAngleMotor2Count;
            }
        }

        return new double[]{currentTiltEncoderCountMotor1, currentTiltEncoderCountMotor2};
    }

    public void runAutoMove(){
        switch(stepState) {
            case STATE_INIT:

                armMotorsRoverRuckus.tiltMotor1.setTargetPosition((int) ((targetDegreeMoveToPosition * config.getCOUNTS_PER_DEGREE_TILT()) - checkTiltCounts()[0]));
                armMotorsRoverRuckus.tiltMotor2.setTargetPosition((int) ((targetDegreeMoveToPosition * config.getCOUNTS_PER_DEGREE_TILT()) - checkTiltCounts()[1]));

                armMotorsRoverRuckus.tiltMotor1.setPower(1);
                armMotorsRoverRuckus.tiltMotor2.setPower(1);

                armMotorsRoverRuckus.tiltMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotorsRoverRuckus.tiltMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                stepState = Constants.stepState.STATE_RUNNING;
                break;

            case STATE_RUNNING:
                if (!armMotorsRoverRuckus.tiltMotor1.isBusy() && !armMotorsRoverRuckus.tiltMotor2.isBusy()) {
                    stepState = Constants.stepState.STATE_COMPLETE;
                    armMotorsRoverRuckus.tiltMotor1.setPower(0);
                    armMotorsRoverRuckus.tiltMotor2.setPower(0);

                }
                checkTiltCounts();
                break;

            case STATE_COMPLETE:

                if ((game2.left_stick_y == 0))
                {
                    if ((hold == false)) {
                        armMotorsRoverRuckus.tiltMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        armMotorsRoverRuckus.tiltMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                        armMotorsRoverRuckus.tiltMotor1.setTargetPosition(armMotorsRoverRuckus.tiltMotor1.getCurrentPosition());
                        armMotorsRoverRuckus.tiltMotor2.setTargetPosition(armMotorsRoverRuckus.tiltMotor2.getCurrentPosition());
                        hold = true;
                    }
                    armMotorsRoverRuckus.tiltMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotorsRoverRuckus.tiltMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotorsRoverRuckus.tiltMotor1.setPower(.2);
                    armMotorsRoverRuckus.tiltMotor2.setPower(.2);

                } else {
                    armMotorsRoverRuckus.tiltMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    armMotorsRoverRuckus.tiltMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    hold = false;
                    armMotorsRoverRuckus.tiltMotor1.setPower(-game2.left_stick_y);
                    armMotorsRoverRuckus.tiltMotor2.setPower(-game2.left_stick_y);
                }

                break;
        }
    }

    public void runNewJOB(double targetDegree){
        stepState = Constants.stepState.STATE_INIT;
        targetDegreeMoveToPosition = targetDegree;
    }
}