package club.towr5291.functions;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.math.BigDecimal;
import java.math.RoundingMode;

/**
 * Created by Ian Haden on 11/04/2017.
 * This class is a collection of utilities that are useful to reuse
 *
 * Some were taken from Titan Robotics Titan Robotics Club (http://www.titanrobotics.com)
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
 * Wyatt Ashley 07/20/2018 ->  Initial creation
 * Ian Haden 07/22/2018 -> Added comments and removed soem unneed steps, renamed some variables so they represent their respective functions
 */


public class TOWR5291Utils {

    /**
     * This method clips the given value to the range limited by the given low and high limits.
     * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
     *
     * @param value specifies the value to be clipped
     * @param lowLimit specifies the low limit of the range.
     * @param highLimit specifies the high limit of the range.
     * @return the result of the clipped value.
     */
    public static int clipRange(int value, int lowLimit, int highLimit)
    {
        return (value < lowLimit)? lowLimit: (value > highLimit)? highLimit: value;
    }   //clipRange


    /**
     * This method clips the given value to the range limited by the given low and high limits.
     * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
     *
     * @param value specifies the value to be clipped
     * @param lowLimit specifies the low limit of the range.
     * @param highLimit specifies the high limit of the range.
     * @return the result of the clipped value.
     */
    public static double clipRange(double value, double lowLimit, double highLimit)
    {
        return (value < lowLimit)? lowLimit: (value > highLimit)? highLimit: value;
    }   //clipRange

    /**
     * This method clips the given value to the range between -1.0 and 1.0.
     * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
     *
     * @param value specifies the value to be clipped
     * @return the result of the clipped value.
     */
    public static double clipRange(double value)
    {
        return clipRange(value, -1.0, 1.0);
    }   //clipRange

    /**
     * This method rounds the double
     *
     * @param value specifies the value to be clipped
     * @param places specifies how many places the value should be rounded
     */
    public static double round(double value, int places) {
        if (places < 0) throw new IllegalArgumentException();

        BigDecimal bd = new BigDecimal(value);
        bd = bd.setScale(places, RoundingMode.HALF_UP);
        return bd.doubleValue();
    }

    /**
     * Computes the current battery voltage
     *
     * @param hardwareMap specifies the hardwareMap to get the sensor from.
     * @return the result of the battery sensor.
     */
    public static double getBatteryVoltage(HardwareMap hardwareMap) {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    /**
     * This method scales the given value from the source range to the target range.
     *
     * @param value specifies the value to be scaled.
     * @param lowSrcRange specifies the low limit of the source range.
     * @param highSrcRange specifies the high limit of the source range.
     * @param lowDstRange specifies the low limit of the target range.
     * @param highDstRange specifies the high limit of the target range
     * @return the result of the scaled value.
     */
    public static int scaleRange(int value, int lowSrcRange, int highSrcRange, int lowDstRange, int highDstRange)
    {
        return lowDstRange + (value - lowSrcRange)*(highDstRange - lowDstRange)/(highSrcRange - lowSrcRange);
    }   //scaleRange

    /**
     * This method scales the given value from the source range to the target range.
     *
     * @param value specifies the value to be scaled.
     * @param lowSrcRange specifies the low limit of the source range.
     * @param highSrcRange specifies the high limit of the source range.
     * @param lowDstRange specifies the low limit of the target range.
     * @param highDstRange specifies the high limit of the target range
     * @return the result of the scaled value.
     */
    public static double scaleRange(
            double value, double lowSrcRange, double highSrcRange, double lowDstRange, double highDstRange)
    {
        return lowDstRange + (value - lowSrcRange)*(highDstRange - lowDstRange)/(highSrcRange - lowSrcRange);
    }   //scaleRange

    /**
     * This method returns the change in direction required based on a desired heading and actual heading.
     *
     * @param currentHeading specifies current heading.
     * @param desiredHeading specifies desired heading.
     * @return the result of the scaled value.
     */
    public static int getNewHeading(int currentHeading, int desiredHeading) {
        int intNewAngle;

        if ((currentHeading + 180) < desiredHeading) {
            intNewAngle = desiredHeading - currentHeading - 360;
        } else {
            intNewAngle = desiredHeading - currentHeading;
        }

        if (Math.abs(intNewAngle) > 180) {
            return desiredHeading + 360 - currentHeading;
        } else {
            return intNewAngle;
        }
    }

    /**
     * This method moves the servo to the desired position in degrees.
     *
     * @param servo specifies servo to move.
     * @param Position specifies the position to move to.
     * @param RangeMin specifies the minimum position the servo can move.
     * @param RangeMax specifies the maximum position the servo can move.
     */
    public static boolean moveServo (Servo servo, double Position, double RangeMin, double RangeMax ) {
        //set right position
        if ((scaleRange(Position, 0, 180, 0, 1) < RangeMin ) ||
                (scaleRange(Position, 0, 180, 0, 1) > RangeMax )) {
            return false;
        }
        servo.setPosition(scaleRange(Position, 0, 180, 0, 1));
        return true;
    }
    /**
     * This method moves the servo to the desired position in degrees.
     *
     * @param servoIn specifies TOWR5291 servo to move.
     * @param Position specifies the position to move to in degrees.
     */
    public static boolean moveServo (Constants.TOWR5291Servo servoIn, double Position) {
        //set right position
        if ((Position < servoIn.minimumPosition ) || (Position > servoIn.maximumPosition )) {
            return false;
        }
        servoIn.servo.setPosition(scaleRange(Position, 0, 180, 0, 1));

        return true;
    }
    /**
     * This method moves the servo to the home position in degrees.
     * @param servoIn specifies TOWR5291 servo to move.
     */
    public static boolean moveServo (Constants.TOWR5291Servo servoIn) {
        return moveServo(servoIn, servoIn.homePosition);
    }

    /**
     * This method moves the servo an offset from the current position.
     *
     * @param servoIn specifies TOWR5291 servo to move.
     * @param offset specifies the offset to move in degrees.
     */
    public static boolean moveServoOffset (Constants.TOWR5291Servo servoIn, double offset) {
        double currentPosition = scaleRange(servoIn.servo.getPosition(), 0, 1, 0, 180);
        Log.d("Here1", "currentPosition: " + currentPosition);
        if (((offset + currentPosition) < servoIn.minimumPosition ) || ((offset + currentPosition) > servoIn.maximumPosition )) {
            Log.d("Here2", "desiredPosition: " + (offset + currentPosition));
            return false;
        }
        Log.d("Here3", "desiredPosition: " + (offset + currentPosition));
        servoIn.servo.setPosition(scaleRange(offset + currentPosition, 0, 180, 0, 1));
        return true;
    }


    /**
     * This method puts the current thread to sleep for the given time in msec. It handles InterruptException where
     * it recalculates the remaining time and calls sleep again repeatedly until the specified sleep time has past.
     *
     * @param milliTime specifies sleep time in msec.
     */
    public static void sleep(long milliTime)
    {
        long wakeupTime = System.currentTimeMillis() + milliTime;

        while (milliTime > 0)
        {
            try
            {
                Thread.sleep(milliTime);
                break;
            }
            catch (InterruptedException e)
            {
                milliTime = wakeupTime - System.currentTimeMillis();
            }
        }
    }   //sleep


}
