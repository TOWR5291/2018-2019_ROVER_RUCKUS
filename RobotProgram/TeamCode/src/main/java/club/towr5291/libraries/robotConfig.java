package club.towr5291.libraries;

import java.util.HashMap;

import club.towr5291.functions.ReadStepFileRoverRuckus;

/**
 * Created by ianhaden on 2/6/18.
 */

public class robotConfig {

    private String teamNumber;
    private String allianceColor;
    private String allianceStartPosition;
    private int delay;
    private String robotConfig;

    private ReadStepFileRoverRuckus autonomousStepsFile = new ReadStepFileRoverRuckus();

    //set up robot variables
    private double COUNTS_PER_MOTOR_REV;                                        // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
    private double DRIVE_GEAR_REDUCTION;                                        // This is < 1.0 if geared UP, Tilerunner is geared up
    private double WHEEL_DIAMETER_INCHES;                                       // For figuring circumference
    private double WHEEL_ACTUAL_FUDGE;                                          // Fine tuning amount
    private double COUNTS_PER_INCH;
    private double ROBOT_TRACK;                                                 //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
    private double COUNTS_PER_DEGREE;
    private double WHEEL_TURN_FUDGE;
    private double REVERSE_DIRECTION;                                           // determines which directin the robot runs when FW is positive or negative when commanded to move a direction
    private int    LIFTMAIN_COUNTS_PER_INCH;                                    //number of encoder counts oer inch
    private int    LIFTTOP_COUNTS_PER_INCH;                                     //number of encoder counts oer inch
    private double COUNTS_PER_INCH_STRAFE;
    private double COUNTS_PER_INCH_STRAFE_FRONT_OFFSET;
    private double COUNTS_PER_INCH_STRAFE_REAR_OFFSET;
    private double COUNTS_PER_INCH_STRAFE_LEFT_OFFSET;
    private double COUNTS_PER_INCH_STRAFE_RIGHT_OFFSET;
    private double MECANUM_TURN_OFFSET;


    public double getCOUNTS_PER_INCH() {
        return COUNTS_PER_INCH;
    }

    public double getCOUNTS_PER_DEGREE() {
        return COUNTS_PER_DEGREE;
    }
    public double getROBOT_TRACK() {
        return ROBOT_TRACK;
    }
    public double getCOUNTS_PER_INCH_STRAFE() {
        return COUNTS_PER_INCH_STRAFE;
    }
    public double getCOUNTS_PER_INCH_STRAFE_FRONT_OFFSET() {
        return COUNTS_PER_INCH_STRAFE_FRONT_OFFSET;
    }
    public double getCOUNTS_PER_INCH_STRAFE_REAR_OFFSET() {
        return COUNTS_PER_INCH_STRAFE_REAR_OFFSET;
    }
    public double getCOUNTS_PER_INCH_STRAFE_LEFT_OFFSET() {
        return COUNTS_PER_INCH_STRAFE_LEFT_OFFSET;
    }
    public double getCOUNTS_PER_INCH_STRAFE_RIGHT_OFFSET() {
        return COUNTS_PER_INCH_STRAFE_RIGHT_OFFSET;
    }

    public robotConfig() {
        this.teamNumber = "";
        this.allianceColor = "";
        this.allianceStartPosition = "";
        this.delay = 0;
        this.robotConfig = "";
    }

    public boolean initConfig() {
        boolean configLoaded = false;

        switch (robotConfig) {
            case "TileRunner-2x40":   //Velocity Vortex Competition Base
                REVERSE_DIRECTION = 1;                                                       // Reverse the direction without significant code changes, (using motor FORWARD REVERSE will affect the driver station as we use same robotconfig file
                COUNTS_PER_MOTOR_REV = 1120;                                                    // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
                DRIVE_GEAR_REDUCTION = 0.7;                                                    // This is < 1.0 if geared UP, Tilerunner is geared up
                WHEEL_DIAMETER_INCHES = 4.0;                                                     // For figuring circumference
                WHEEL_ACTUAL_FUDGE = 1;                                                        // Fine tuning amount
                COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)) * WHEEL_ACTUAL_FUDGE * REVERSE_DIRECTION;
                ROBOT_TRACK = 16.5;                                                     //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
                WHEEL_TURN_FUDGE = 1.0;                                                        // Fine tuning amount
                COUNTS_PER_DEGREE = (((2 * 3.1415 * ROBOT_TRACK) * COUNTS_PER_INCH) / 360) * WHEEL_TURN_FUDGE;
                configLoaded = true;
                //load the power table
                break;
            case "5291 Tank Tread-2x40 Custom":   //for tank tread base
                REVERSE_DIRECTION = 1;
                COUNTS_PER_MOTOR_REV = 1120;                                                    // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
                DRIVE_GEAR_REDUCTION = 1.0;                                                     // Tank Tread is 1:1 ration
                WHEEL_DIAMETER_INCHES = 3.75;                                                     // For figuring circumference
                WHEEL_ACTUAL_FUDGE = 1;                                                        // Fine tuning amount
                COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)) * WHEEL_ACTUAL_FUDGE * REVERSE_DIRECTION;
                ROBOT_TRACK = 18;                                                     //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
                WHEEL_TURN_FUDGE = 1.12;                                                        // Fine tuning amount
                COUNTS_PER_DEGREE = (((2 * 3.1415 * ROBOT_TRACK) * COUNTS_PER_INCH) / 360) * WHEEL_TURN_FUDGE;
                MECANUM_TURN_OFFSET = 0;
                configLoaded = true;                                                      //load the power table
                break;
            case "TileRunnerMecanum2x40":
                REVERSE_DIRECTION = 1;                                                        // Reverse the direction without significant code changes, (using motor FORWARD REVERSE will affect the driver station as we use same robotconfig file
                COUNTS_PER_MOTOR_REV = 1120;                                                    // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
                DRIVE_GEAR_REDUCTION = 1.0;                                                     // This is < 1.0 if geared UP, Tilerunner is geared up
                WHEEL_DIAMETER_INCHES = 4.0;                                                     // For figuring circumference
                WHEEL_ACTUAL_FUDGE = 1.02;                                                     // Fine tuning amount
                COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)) * WHEEL_ACTUAL_FUDGE * REVERSE_DIRECTION;
                COUNTS_PER_INCH_STRAFE_FRONT_OFFSET = 1;
                COUNTS_PER_INCH_STRAFE_REAR_OFFSET = 1;
                COUNTS_PER_INCH_STRAFE_LEFT_OFFSET = 1;
                COUNTS_PER_INCH_STRAFE_RIGHT_OFFSET = .85;
                COUNTS_PER_INCH_STRAFE = COUNTS_PER_INCH * 1.65;
                ROBOT_TRACK = 16.5;                                                     //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
                WHEEL_TURN_FUDGE = 1.0;                                                        // Fine tuning amount
                COUNTS_PER_DEGREE = (((2 * 3.1415 * ROBOT_TRACK) * COUNTS_PER_INCH) / 360) * WHEEL_TURN_FUDGE;
                LIFTMAIN_COUNTS_PER_INCH = 420;                                                   //number of encoder counts per inch
                LIFTTOP_COUNTS_PER_INCH = -420;                                                   //number of encoder counts per inch
                MECANUM_TURN_OFFSET = 1.72;
                configLoaded = true;
                break;
            case "11231 2016 Custom": //2016 - 11231 Drivetrain
                COUNTS_PER_MOTOR_REV = 1120;                                                     // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
                DRIVE_GEAR_REDUCTION = .667;                                                   // (.665) UP INCREASES THE DISTANCE This is < 1.0 if geared UP, Tilerunner is geared up
                WHEEL_DIAMETER_INCHES = 4.0;                                                     // For figuring circumference
                WHEEL_ACTUAL_FUDGE = 1;                                                        // Fine tuning amount
                COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415926535)) * WHEEL_ACTUAL_FUDGE;
                ROBOT_TRACK = 18;                                                     //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
                COUNTS_PER_DEGREE = ((2 * 3.1415926535 * ROBOT_TRACK) * COUNTS_PER_INCH) / 360;
                MECANUM_TURN_OFFSET = 0;
                //loadPowerTableTileRunner();                                                         //load the power table
                configLoaded = true;
                break;
            default:  //default for competition TileRunner-2x40
                REVERSE_DIRECTION = 1;
                COUNTS_PER_MOTOR_REV = 1120;                                                     // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
                DRIVE_GEAR_REDUCTION = 1.28;                                                    // This is < 1.0 if geared UP, Tilerunner is geared up
                WHEEL_DIAMETER_INCHES = 4.0;                                                     // For figuring circumference
                WHEEL_ACTUAL_FUDGE = 1;                                                        // Fine tuning amount
                COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)) * WHEEL_ACTUAL_FUDGE * REVERSE_DIRECTION;
                ROBOT_TRACK = 16.5;                                                     //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
                COUNTS_PER_DEGREE = ((2 * 3.1415 * ROBOT_TRACK) * COUNTS_PER_INCH) / 360;
                MECANUM_TURN_OFFSET = 0;
                configLoaded = false;
                break;
        }
        return configLoaded;
    }

    public String getTeamNumber() {
        return teamNumber;
    }

    public void setTeamNumber(String teamNumber) {
        this.teamNumber = teamNumber;
    }

    public String getAllianceColor() {
        return allianceColor;
    }

    public void setAllianceColor(String allianceColor) {
        this.allianceColor = allianceColor;
    }

    public String getAllianceStartPosition() {
        return allianceStartPosition;
    }

    public void setAllianceStartPosition(String allianceStartPosition) {
        this.allianceStartPosition = allianceStartPosition;
    }

    public int getDelay() {
        return delay;
    }

    public void setDelay(int delay) {
        this.delay = delay;
    }

    public String getRobotConfig() {
        return this.robotConfig;
    }

    public void setRobotConfig(String robotConfig) {
        this.robotConfig = robotConfig;
    }



}
