package club.towr5291.libraries;

import android.content.SharedPreferences;

import java.util.HashMap;

import club.towr5291.functions.FileLogger;
import club.towr5291.functions.ReadStepFileRoverRuckus;

import static club.towr5291.functions.Constants.SharedPreferencesValues.ALLIANCE_COLOR;
import static club.towr5291.functions.Constants.SharedPreferencesValues.ALLIANCE_START_POSITION;
import static club.towr5291.functions.Constants.SharedPreferencesValues.DEBUG;
import static club.towr5291.functions.Constants.SharedPreferencesValues.ROBOT_BASE_CONFIG;
import static club.towr5291.functions.Constants.SharedPreferencesValues.ROBOT_MOTOR_TYPE;
import static club.towr5291.functions.Constants.SharedPreferencesValues.START_DELAY;
import static club.towr5291.functions.Constants.SharedPreferencesValues.TEAM_NUMBER;

/**
 * Created by Ian Haden on 02/06/2018.
 *
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Modification history
 * Edited by:
 * Ian Haden    02/06/2018  -> Initial creation
 * Wyatt Ashley 03/05/2019  -> Changed A LOT configuration is now different have another class for motor types
 * Wyatt Ashley 04/06/2019  -> Cleaned all of the code and is now easier to use
 */

public class robotConfig {

    private String teamNumber;
    private String allianceColor;
    private String allianceStartPosition;
    private int delay;
    private int debug;
    private String robotConfigBase;
    private String robotMotorType;

    private LibraryMotorType libraryMotorType = new LibraryMotorType();

    //set up robot variables
    private double COUNTS_PER_MOTOR_REV;                                        // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
    private double DRIVE_GEAR_REDUCTION;                                        // This is < 1.0 if geared UP, Tilerunner is geared up
    private double WHEEL_DIAMETER_INCHES;                                       // For figuring circumference
    private double WHEEL_ACTUAL_FUDGE;                                          // Fine tuning amount
    private double COUNTS_PER_INCH;
    private double ROBOT_TRACK;                                                 //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
    private double COUNTS_PER_DEGREE;
    private double WHEEL_TURN_FUDGE;
    private double REVERSE_DIRECTION;                                           // determines which direction the robot runs when FW is positive or negative when commanded to move a direction
    private int    LIFTMAIN_COUNTS_PER_INCH;                                    // number of encoder counts oer inch
    private double COUNTS_PER_INCH_STRAFE;
    private double COUNTS_PER_INCH_STRAFE_FRONT_OFFSET;
    private double COUNTS_PER_INCH_STRAFE_REAR_OFFSET;
    private double COUNTS_PER_INCH_STRAFE_LEFT_OFFSET;
    private double COUNTS_PER_INCH_STRAFE_RIGHT_OFFSET;
    private double MECANUM_TURN_OFFSET;
    private double COUNTS_PER_DEGREE_TILT_MOTOR;

    public enum eyeServos {
        rightEYE ("rightEYE", 80, 120, 81),
        leftEYE ("leftEYE", 80, 120, 83);

        private final String name;
        private final double min;
        private final double max;
        private final double home;

        eyeServos (String name, double min, double max, double home) {
            this.name = name;
            this.min = min;
            this.max = max;
            this.home = home;
        }

        public String toString() {
            return name;
        }

        public double minPos() {
            return this.min;
        }
        public double maxPos() {
            return this.max;
        }
        public double homePos() {
            return this.home;
        }

    }

    public enum motors {
        leftMotor1 ("leftMotor1", 1),
        leftMotor2 ("leftMotor2", 2),
        rightMotor1 ("rightMotor1", 4),
        rightMotor2 ("rightMotor2", 8);

        private final String name;
        private final int value;

        motors (String name, int value) {
            this.name = name;
            this.value = value;
        }


        public String toString() {
            return name;
        }

        public int toInt() {
            return value;
        }
    }

    public enum LEDnames {
        leftGreen ("green1", 1),
        leftRed ("red1", 2),
        leftBlue ("blue1", 4),
        rightGreen ("green2", 8),
        rightRed ("red2", 16),
        rightBlue ("blue2", 32);

        private final String name;
        private final int value;

        LEDnames (String name, int value) {
            this.name = name;
            this.value = value;
        }

        public String toString() {
            return name;
        }

        public int toInt() {
            return value;
        }
    }

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

    public int getLIFTMAIN_COUNTS_PER_INCH(){return LIFTMAIN_COUNTS_PER_INCH;}

    @Deprecated
    public robotConfig() {
        this.teamNumber = "";
        this.allianceColor = "";
        this.allianceStartPosition = "";
        this.delay = 0;
        this.debug = 1;
        this.robotConfigBase = "";
        this.robotMotorType = "";
    }

    public robotConfig (SharedPreferences sharedPreferences){
        this.allianceColor = sharedPreferences.getString(ALLIANCE_COLOR.getSharedPrefString(), ALLIANCE_COLOR.getSharedPrefDefault());// Using a Function to Store The Robot Specification
        this.teamNumber = sharedPreferences.getString(TEAM_NUMBER.getSharedPrefString(), TEAM_NUMBER.getSharedPrefDefault());
        this.allianceStartPosition = sharedPreferences.getString(ALLIANCE_START_POSITION.getSharedPrefString(), ALLIANCE_START_POSITION.getSharedPrefDefault());
        this.delay = Integer.parseInt(sharedPreferences.getString(START_DELAY.getSharedPrefString(), START_DELAY.getSharedPrefDefault()));
        this.robotMotorType = sharedPreferences.getString(ROBOT_MOTOR_TYPE.getSharedPrefString(), ROBOT_MOTOR_TYPE.getSharedPrefDefault());
        this.robotConfigBase = sharedPreferences.getString(ROBOT_BASE_CONFIG.getSharedPrefString(), ROBOT_BASE_CONFIG.getSharedPrefDefault());
        this.debug = Integer.parseInt(sharedPreferences.getString(DEBUG.getSharedPrefString(), DEBUG.getSharedPrefDefault()));

        initConfig();
    }

    public void initConfig() {
        libraryMotorType.loadData(LibraryMotorType.MotorTypes.valueOf(robotMotorType));

        switch (robotConfigBase) {
            case "TileRunnerRegular":
                REVERSE_DIRECTION = 1;                                                       // Reverse the direction without significant code changes, (using motor FORWARD REVERSE will affect the driver station as we use same robotconfig file
                COUNTS_PER_MOTOR_REV = libraryMotorType.getCOUNTSPERROTATION();              // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
                DRIVE_GEAR_REDUCTION = 0.7;                                                  // This is < 1.0 if geared UP, Tilerunner is geared up
                WHEEL_DIAMETER_INCHES = 4.0;                                                 // For figuring circumference
                WHEEL_ACTUAL_FUDGE = 1;                                                      // Fine tuning amount
                COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)) * WHEEL_ACTUAL_FUDGE * REVERSE_DIRECTION;
                ROBOT_TRACK = 16.5;                                                     //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
                WHEEL_TURN_FUDGE = 1.0;                                                        // Fine tuning amount
                COUNTS_PER_DEGREE = (((2 * 3.1415 * ROBOT_TRACK) * COUNTS_PER_INCH) / 360) * WHEEL_TURN_FUDGE;
                break;
            case "TileRunnerRegularOrbital":
                REVERSE_DIRECTION = 1;                                                        // Reverse the direction without significant code changes, (using motor FORWARD REVERSE will affect the driver station as we use same robotconfig file
                COUNTS_PER_MOTOR_REV = libraryMotorType.getCOUNTSPERROTATION();               // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
                DRIVE_GEAR_REDUCTION = 1.0;                                                   // This is < 1.0 if geared UP, Tilerunner is geared up
                WHEEL_DIAMETER_INCHES = 4.0;                                                  // For figuring circumference
                WHEEL_ACTUAL_FUDGE = 1.02;                                                    // Fine tuning amount
                COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)) * WHEEL_ACTUAL_FUDGE * REVERSE_DIRECTION;
                COUNTS_PER_INCH_STRAFE_FRONT_OFFSET = 1;
                COUNTS_PER_INCH_STRAFE_REAR_OFFSET = 1;
                COUNTS_PER_INCH_STRAFE_LEFT_OFFSET = 1;
                COUNTS_PER_INCH_STRAFE_RIGHT_OFFSET = 1;
                COUNTS_PER_INCH_STRAFE = COUNTS_PER_INCH * 1.1;
                ROBOT_TRACK = 16.5;                                                     //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
                WHEEL_TURN_FUDGE = 1.0;                                                        // Fine tuning amount
                COUNTS_PER_DEGREE = (((2 * 3.1415 * ROBOT_TRACK) * COUNTS_PER_INCH) / 360) * WHEEL_TURN_FUDGE;
                LIFTMAIN_COUNTS_PER_INCH = 456;
                MECANUM_TURN_OFFSET = 1.72;
                COUNTS_PER_DEGREE_TILT_MOTOR = 30;
                //number of encoder counts per inch
                break;
            case "TileRunnerMecanum":
                REVERSE_DIRECTION = 1;                                                        // Reverse the direction without significant code changes, (using motor FORWARD REVERSE will affect the driver station as we use same robotconfig file
                COUNTS_PER_MOTOR_REV = libraryMotorType.getCOUNTSPERROTATION();               //eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
                DRIVE_GEAR_REDUCTION = 1.0;                                                   // This is < 1.0 if geared UP, Tilerunner is geared up
                WHEEL_DIAMETER_INCHES = 4.0;                                                  // For figuring circumference
                WHEEL_ACTUAL_FUDGE = 1.02;                                                    // Fine tuning amount
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
                MECANUM_TURN_OFFSET = 1.72;
                break;
            case "TileRunnerMecanumOrbital":
                REVERSE_DIRECTION = 1;                                                        // Reverse the direction without significant code changes, (using motor FORWARD REVERSE will affect the driver station as we use same robotconfig file
                COUNTS_PER_MOTOR_REV = libraryMotorType.getCOUNTSPERROTATION();               // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
                DRIVE_GEAR_REDUCTION = 1.0;                                                   // This is < 1.0 if geared UP, Tilerunner is geared up
                WHEEL_DIAMETER_INCHES = 4.0;                                                  // For figuring circumference
                WHEEL_ACTUAL_FUDGE = 1.02;                                                    // Fine tuning amount
                COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)) * WHEEL_ACTUAL_FUDGE * REVERSE_DIRECTION;
                COUNTS_PER_INCH_STRAFE_FRONT_OFFSET = 1.8;//1.25
                COUNTS_PER_INCH_STRAFE_REAR_OFFSET = 1;//1.8
                COUNTS_PER_INCH_STRAFE_LEFT_OFFSET = 1;
                COUNTS_PER_INCH_STRAFE_RIGHT_OFFSET = 1;
                COUNTS_PER_INCH_STRAFE = COUNTS_PER_INCH * 1.1;
                ROBOT_TRACK = 16.5;                                                     //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
                WHEEL_TURN_FUDGE = 1.15;                                                        // Fine tuning amount
                COUNTS_PER_DEGREE = (((2 * 3.1415 * ROBOT_TRACK) * COUNTS_PER_INCH) / 360) * WHEEL_TURN_FUDGE;
                LIFTMAIN_COUNTS_PER_INCH = 287;
                MECANUM_TURN_OFFSET = 1.72;
                COUNTS_PER_DEGREE_TILT_MOTOR = 48;
                break;
            case "TileRunnerOmni":
                REVERSE_DIRECTION = 1;                                                       // Reverse the direction without significant code changes, (using motor FORWARD REVERSE will affect the driver station as we use same robotconfig file
                COUNTS_PER_MOTOR_REV = libraryMotorType.getCOUNTSPERROTATION();              // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
                DRIVE_GEAR_REDUCTION = 0.7;                                                  // This is < 1.0 if geared UP, Tilerunner is geared up
                WHEEL_DIAMETER_INCHES = 4.0;                                                 // For figuring circumference
                WHEEL_ACTUAL_FUDGE = 1;                                                      // Fine tuning amount
                COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)) * WHEEL_ACTUAL_FUDGE * REVERSE_DIRECTION;
                ROBOT_TRACK = 16.5;                                                     //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
                WHEEL_TURN_FUDGE = 1.0;                                                        // Fine tuning amount
                COUNTS_PER_DEGREE = (((2 * 3.1415 * ROBOT_TRACK) * COUNTS_PER_INCH) / 360) * WHEEL_TURN_FUDGE;
                break;
            case "5291 Tank Tread-2x40 Custom":   //for tank tread base
                REVERSE_DIRECTION = 1;
                COUNTS_PER_MOTOR_REV = libraryMotorType.getCOUNTSPERROTATION();                 // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
                DRIVE_GEAR_REDUCTION = 1.0;                                                     // Tank Tread is 1:1 ration
                WHEEL_DIAMETER_INCHES = 3.75;                                                   // For figuring circumference
                WHEEL_ACTUAL_FUDGE = 1;                                                         // Fine tuning amount
                COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)) * WHEEL_ACTUAL_FUDGE * REVERSE_DIRECTION;
                ROBOT_TRACK = 18;                                                     //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
                WHEEL_TURN_FUDGE = 1.12;                                                        // Fine tuning amount
                COUNTS_PER_DEGREE = (((2 * 3.1415 * ROBOT_TRACK) * COUNTS_PER_INCH) / 360) * WHEEL_TURN_FUDGE;
                MECANUM_TURN_OFFSET = 0;
                break;
            case "11231 2016 Custom": //2016 - 11231 Drivetrain
                COUNTS_PER_MOTOR_REV = libraryMotorType.getCOUNTSPERROTATION();                // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
                DRIVE_GEAR_REDUCTION = .667;                                                   // (.665) UP INCREASES THE DISTANCE This is < 1.0 if geared UP, Tilerunner is geared up
                WHEEL_DIAMETER_INCHES = 4.0;                                                   // For figuring circumference
                WHEEL_ACTUAL_FUDGE = 1;                                                        // Fine tuning amount
                COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415926535)) * WHEEL_ACTUAL_FUDGE;
                ROBOT_TRACK = 18;                                                     //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
                COUNTS_PER_DEGREE = ((2 * 3.1415926535 * ROBOT_TRACK) * COUNTS_PER_INCH) / 360;
                MECANUM_TURN_OFFSET = 0;
                //loadPowerTableTileRunner();
                break;
            default:  //default for competition TileRunner-2x40
                REVERSE_DIRECTION = 1;
                COUNTS_PER_MOTOR_REV = libraryMotorType.getCOUNTSPERROTATION();               // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
                DRIVE_GEAR_REDUCTION = 1.28;                                                  // This is < 1.0 if geared UP, Tilerunner is geared up
                WHEEL_DIAMETER_INCHES = 4.0;                                                  // For figuring circumference
                WHEEL_ACTUAL_FUDGE = 1;                                                       // Fine tuning amount
                COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)) * WHEEL_ACTUAL_FUDGE * REVERSE_DIRECTION;
                ROBOT_TRACK = 16.5;                                                     //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
                COUNTS_PER_DEGREE = ((2 * 3.1415 * ROBOT_TRACK) * COUNTS_PER_INCH) / 360;
                MECANUM_TURN_OFFSET = 0;
                break;
        }
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

    public void setAllianceStartPosition(String allianceStartPositions) {
        this.allianceStartPosition = allianceStartPositions;
    }
    public String getAllianceStartPosition(){
        return this.allianceStartPosition;
    }

    public int getDelay() {
        return delay;
    }
    public void setDelay(int delay) {
        this.delay = delay;
    }

    public int getDebug(){
        return debug;
    }
    public void setDebug(int debug){
        this.debug = debug;
    }

    public String getRobotConfigBase() {
        return this.robotConfigBase;
    }
    public void setRobotConfigBase(String robotConfig) {
        this.robotConfigBase = robotConfig;
    }

    public void setRobotMotorType(String type){
        this.robotMotorType = type;
    }
    public String getRobotMotorType() {
        return this.robotMotorType;
    }

    public double getMecanumTurnOffset() { return this.MECANUM_TURN_OFFSET;}

    public double getCOUNTS_PER_DEGREE_TILT() {return this.COUNTS_PER_DEGREE_TILT_MOTOR;}
}
