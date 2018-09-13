package club.towr5291.sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import club.towr5291.libraries.robotConfig;


/**
 * Created by Ian Haden TOWR5291 on 07/30/2018.
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
 * Ian Haden 07/30/2018 -> Initial creation
 */
public class TOWR5291IMU {
    //adafruit IMU
    // The IMU sensor object
    private BNO055IMU imu;
    // State used for updating telemetry
    private boolean useAdafruitIMU = false;

    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    private final double GYRO_CORRECTION_MULTIPLIER = 0.9833;

    public TOWR5291IMU (HardwareMap hardwareMap, String IMUType) {
        //check if IMUType has MR in it for Modern Robotics
        if (IMUType.toUpperCase().contains("MR")) {
            this.useAdafruitIMU = false;
        }

        if (this.useAdafruitIMU) {
            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to the logcat log; it doesn't actually
            // provide positional information.
            BNO055IMU.Parameters parametersAdafruitImu = new BNO055IMU.Parameters();
            parametersAdafruitImu.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parametersAdafruitImu.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parametersAdafruitImu.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
            parametersAdafruitImu.loggingEnabled = true;
            parametersAdafruitImu.loggingTag = "IMU";
            parametersAdafruitImu.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU"
            this.imu = hardwareMap.get(BNO055IMU.class, robotConfig.SensorNames.BNO0055.toString());
            this.imu.initialize(parametersAdafruitImu);
        } else {
            // Get a reference to a Modern Robotics gyro object. We use several interfaces
            // on this object to illustrate which interfaces support which functionality.
            this.modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, robotConfig.SensorNames.MRGYRO.toString());
            this.gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;
            // If you're only interested in the IntegratingGyroscope interface, the following will suffice.
            // gyro = hardwareMap.get(IntegratingGyroscope.class, "gyro");
            // A similar approach will work for the Gyroscope interface, if that's all you need.

            // Start calibrating the gyro. This takes a few seconds and is worth performing
            // during the initialization phase at the start of each opMode.
            this.modernRoboticsI2cGyro.calibrate();
        }
    }

    public boolean start () {
        if (this.useAdafruitIMU) {
            this.imu.startAccelerationIntegration(new Position(), new Velocity(), 200);
            return true;
        } else {
            if (this.modernRoboticsI2cGyro.isCalibrating()) {
                return false;
            } else {
                return true;
            }
        }
    }

    public void resetHeading() {
        if (this.useAdafruitIMU) {
            this.imu.stopAccelerationIntegration();
            this.start();
        } else {
            this.modernRoboticsI2cGyro.resetZAxisIntegrator();
        }
    }

    public Double getHeading () {
        if (this.useAdafruitIMU) {
            return getAdafruitHeading();
        } else {
            //Moder Robotics GYRO has an error
            return (this.modernRoboticsI2cGyro.getHeading() * GYRO_CORRECTION_MULTIPLIER);
        }
    }

    private Double getAdafruitHeading()
    {
        Orientation angles;
        angles = this.imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return angleToHeading(formatAngle(angles.angleUnit, angles.firstAngle));
    }

    //for adafruit IMU
    private Double formatAngle(AngleUnit angleUnit, double angle) {
        return AngleUnit.DEGREES.fromUnit(angleUnit, angle);
    }

    //for adafruit IMU as it returns z angle only
    private double angleToHeading(double z) {
        double angle = -z;
        if (angle < 0)
            return angle + 360;
        else if (angle > 360)
            return angle - 360;
        else
            return angle;
    }
}
