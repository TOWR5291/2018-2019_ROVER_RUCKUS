package club.towr5291.libraries;

import java.util.HashMap;

import club.towr5291.functions.ReadStepFileRoverRuckus;

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
 * Ian Haden 02/06/2018 -> Initial creation
 */

public class robotConfig {

    private String teamNumber;
    private String allianceColor;
    private String allianceStartPosition;
    private String allianceParkPosition;
    private int delay;
    private String robotConfigBase;

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
    private double REVERSE_DIRECTION;                                           // determines which direction the robot runs when FW is positive or negative when commanded to move a direction
    private int    LIFTMAIN_COUNTS_PER_INCH;                                    // number of encoder counts oer inch
    private int    LIFTTOP_COUNTS_PER_INCH;                                     // number of encoder counts oer inch
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

    public enum armMotorsROVERRUCKUS {
        Motor1 ("liftMotor1", 1),
        Motor2 ("liftMotor2", 2),
        Motor3 ("tiltMotor1", 4),
        Motor4 ("intakeMotor", 8);

        private final String name;
        private final int value;

        armMotorsROVERRUCKUS (String name, int value) {
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

    public enum SensorNames {
        BNO0055 ("imu"),
        MRGYRO ("gyro");

        private final String name;

        SensorNames (String name) {
            this.name = name;
        }

        public String toString() {
            return name;
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
    public String getAllianceStartPosition() {
        return allianceStartPosition;
    }
    public int getLIFTMAIN_COUNTS_PER_INCH(){return LIFTMAIN_COUNTS_PER_INCH;}

    public robotConfig() {
        this.teamNumber = "";
        this.allianceColor = "";
        this.allianceStartPosition = "";
        this.allianceParkPosition = "";
        this.delay = 0;
        this.robotConfigBase = "";
    }

    public boolean initConfig() {
        boolean configLoaded = false;

        switch (robotConfigBase) {
            case "TileRunner2x60Andy":
                REVERSE_DIRECTION = 1;                                                       // Reverse the direction without significant code changes, (using motor FORWARD REVERSE will affect the driver station as we use same robotconfig file
                COUNTS_PER_MOTOR_REV = 1680;                                                    // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
                DRIVE_GEAR_REDUCTION = 0.7;                                                    // This is < 1.0 if geared UP, Tilerunner is geared up
                WHEEL_DIAMETER_INCHES = 4.0;                                                     // For figuring circumference
                WHEEL_ACTUAL_FUDGE = 1;                                                        // Fine tuning amount
                COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)) * WHEEL_ACTUAL_FUDGE * REVERSE_DIRECTION;
                ROBOT_TRACK = 16.5;                                                     //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
                WHEEL_TURN_FUDGE = 1.0;                                                        // Fine tuning amount
                COUNTS_PER_DEGREE = (((2 * 3.1415 * ROBOT_TRACK) * COUNTS_PER_INCH) / 360) * WHEEL_TURN_FUDGE;
                configLoaded = true;
                break;
            case "TileRunner2x40Andy":   //Velocity Vortex Competition Base
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
            case "TileRunner2x20Andy":
                REVERSE_DIRECTION = 1;                                                       // Reverse the direction without significant code changes, (using motor FORWARD REVERSE will affect the driver station as we use same robotconfig file
                COUNTS_PER_MOTOR_REV = 560;                                                    // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
                DRIVE_GEAR_REDUCTION = 0.7;                                                    // This is < 1.0 if geared UP, Tilerunner is geared up
                WHEEL_DIAMETER_INCHES = 4.0;                                                     // For figuring circumference
                WHEEL_ACTUAL_FUDGE = 1;                                                        // Fine tuning amount
                COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)) * WHEEL_ACTUAL_FUDGE * REVERSE_DIRECTION;
                ROBOT_TRACK = 16.5;                                                     //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
                WHEEL_TURN_FUDGE = 1.0;                                                        // Fine tuning amount
                COUNTS_PER_DEGREE = (((2 * 3.1415 * ROBOT_TRACK) * COUNTS_PER_INCH) / 360) * WHEEL_TURN_FUDGE;
                configLoaded = true;
                break;
            case "TileRunnerOrbital2x20Andy":
                REVERSE_DIRECTION = 1;                                                        // Reverse the direction without significant code changes, (using motor FORWARD REVERSE will affect the driver station as we use same robotconfig file
                COUNTS_PER_MOTOR_REV = 537.6;                                                    // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
                DRIVE_GEAR_REDUCTION = 1.0;                                                     // This is < 1.0 if geared UP, Tilerunner is geared up
                WHEEL_DIAMETER_INCHES = 4.0;                                                     // For figuring circumference
                WHEEL_ACTUAL_FUDGE = 1.02;                                                     // Fine tuning amount
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
                configLoaded = true;
                break;
            case "TileRunnerMecanum2x20Andy":
                REVERSE_DIRECTION = 1;                                                        // Reverse the direction without significant code changes, (using motor FORWARD REVERSE will affect the driver station as we use same robotconfig file
                COUNTS_PER_MOTOR_REV = 560;                                                    // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
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
            case "TileRunnerMecanum2x40Andy":
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
            case "TileRunnerMecanum2x60Andy":
                REVERSE_DIRECTION = 1;                                                        // Reverse the direction without significant code changes, (using motor FORWARD REVERSE will affect the driver station as we use same robotconfig file
                COUNTS_PER_MOTOR_REV = 1680;                                                    // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
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
            case "TileRunnerMecanumOrbital2x20Andy":
                REVERSE_DIRECTION = 1;                                                        // Reverse the direction without significant code changes, (using motor FORWARD REVERSE will affect the driver station as we use same robotconfig file
                COUNTS_PER_MOTOR_REV = 537.6;                                                    // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
                DRIVE_GEAR_REDUCTION = 1.0;                                                     // This is < 1.0 if geared UP, Tilerunner is geared up
                WHEEL_DIAMETER_INCHES = 4.0;                                                     // For figuring circumference
                WHEEL_ACTUAL_FUDGE = 1.02;                                                     // Fine tuning amount
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
                configLoaded = true;
                break;


            case "TileRunner2x40REV":
                REVERSE_DIRECTION = 1;                                                       // Reverse the direction without significant code changes, (using motor FORWARD REVERSE will affect the driver station as we use same robotconfig file
                //TODO find and Fix encoder Count for REV 40 Motor
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
            case "TileRunner2x20REV":
                REVERSE_DIRECTION = 1;                                                       // Reverse the direction without significant code changes, (using motor FORWARD REVERSE will affect the driver station as we use same robotconfig file
                //TODO find and ix encoder counts for REV 20 Motor
                COUNTS_PER_MOTOR_REV = 560;                                                    // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
                DRIVE_GEAR_REDUCTION = 0.7;                                                    // This is < 1.0 if geared UP, Tilerunner is geared up
                WHEEL_DIAMETER_INCHES = 4.0;                                                     // For figuring circumference
                WHEEL_ACTUAL_FUDGE = 1;                                                        // Fine tuning amount
                COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)) * WHEEL_ACTUAL_FUDGE * REVERSE_DIRECTION;
                ROBOT_TRACK = 16.5;                                                     //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
                WHEEL_TURN_FUDGE = 1.0;                                                        // Fine tuning amount
                COUNTS_PER_DEGREE = (((2 * 3.1415 * ROBOT_TRACK) * COUNTS_PER_INCH) / 360) * WHEEL_TURN_FUDGE;
                configLoaded = true;
                break;
            case "TileRunnerOrbital2x20REV":
                REVERSE_DIRECTION = 1;                                                       // Reverse the direction without significant code changes, (using motor FORWARD REVERSE will affect the driver station as we use same robotconfig file
                //TODO find and ix encoder counts for REV 20 Orbital Motor
                COUNTS_PER_MOTOR_REV = 560;                                                    // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
                DRIVE_GEAR_REDUCTION = 0.7;                                                    // This is < 1.0 if geared UP, Tilerunner is geared up
                WHEEL_DIAMETER_INCHES = 4.0;                                                     // For figuring circumference
                WHEEL_ACTUAL_FUDGE = 1;                                                        // Fine tuning amount
                COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)) * WHEEL_ACTUAL_FUDGE * REVERSE_DIRECTION;
                ROBOT_TRACK = 16.5;                                                     //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
                WHEEL_TURN_FUDGE = 1.0;                                                        // Fine tuning amount
                COUNTS_PER_DEGREE = (((2 * 3.1415 * ROBOT_TRACK) * COUNTS_PER_INCH) / 360) * WHEEL_TURN_FUDGE;
                configLoaded = true;
                break;
            case "TileRunnerMecanum2x20REV":
                REVERSE_DIRECTION = 1;                                                        // Reverse the direction without significant code changes, (using motor FORWARD REVERSE will affect the driver station as we use same robotconfig file
                //TODO Fix and Find the encoder counts For REV 20
                COUNTS_PER_MOTOR_REV = 560;                                                    // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
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
            case "TileRunnerMecanum2x40REV":
                REVERSE_DIRECTION = 1;                                                        // Reverse the direction without significant code changes, (using motor FORWARD REVERSE will affect the driver station as we use same robotconfig file
                //TODO Fix and Find the encoder counts For REV 40
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
            case "TileRunnerMecanumOrbital2x20REV":
                REVERSE_DIRECTION = 1;                                                        // Reverse the direction without significant code changes, (using motor FORWARD REVERSE will affect the driver station as we use same robotconfig file
                //TODO Find the encoder counts for Orbital 20 REV
                COUNTS_PER_MOTOR_REV = 537.6;                                                    // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
                DRIVE_GEAR_REDUCTION = 1.0;                                                     // This is < 1.0 if geared UP, Tilerunner is geared up
                WHEEL_DIAMETER_INCHES = 4.0;                                                     // For figuring circumference
                WHEEL_ACTUAL_FUDGE = 1.02;                                                     // Fine tuning amount
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
                configLoaded = true;
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

    public void setAllianceStartPosition(String allianceStartPositions) {
        this.allianceStartPosition = allianceStartPositions;
    }

    public String getAllianceParkPosition() {
        return allianceParkPosition;
    }

    public void setAllianceParkPosition(String allianceParkPositions) {
        this.allianceParkPosition = allianceParkPositions;
    }

    public int getDelay() {
        return delay;
    }

    public void setDelay(int delay) {
        this.delay = delay;
    }

    public String getRobotConfigBase() {
        return this.robotConfigBase;
    }

    public void setRobotConfigBase(String robotConfig) {
        this.robotConfigBase = robotConfig;
    }

    public double getMecanumTurnOffset() { return this.MECANUM_TURN_OFFSET;}

    public double getCOUNTS_PER_DEGREE_TILT() {return this.COUNTS_PER_DEGREE_TILT_MOTOR;}
}
