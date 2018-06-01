/*
Modern Robotics Range Sensor Example
Created 9/8/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 2.x Beta
Reuse permitted with credit where credit is due

Configuration:
I2cDevice on an Interface Module named "range" at the default address of 0x28 (0x14 7-bit)

This program can be run without a battery and Power Destitution Module.

For more information, visit modernroboticsedu.com.
Support is available by emailing support@modernroboticsinc.com.
*/

package club.towr5291.Concepts;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Written by Ian Haden
// Coach 1 for 5291
// Based of the single MR Range Sensor Code

@Disabled
@TeleOp(name = "Concept Range Sensor Dual MR", group = "5291Concept")
public class ConceptMRIRangeSensorDual2 extends OpMode {

    //set up range sensor1
    ModernRoboticsI2cRangeSensor rangeSensor1;

    //set up rangesensor 2
    ModernRoboticsI2cRangeSensor rangeSensor2;

    @Override
    public void init() {
        // get a reference to our compass
        rangeSensor1 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range1");
        rangeSensor2 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range2");

    }

    @Override
    public void loop() {
        telemetry.addData("R1 raw ultrasonic", rangeSensor1.rawUltrasonic());
        telemetry.addData("R1 raw optical", rangeSensor1.rawOptical());
        telemetry.addData("R1 cm optical", "%.2f cm", rangeSensor1.cmOptical());
        telemetry.addData("R1 cm", "%.2f cm", rangeSensor1.getDistance(DistanceUnit.CM));
        telemetry.addData("R2 raw ultrasonic", rangeSensor2.rawUltrasonic());
        telemetry.addData("R2 raw optical", rangeSensor2.rawOptical());
        telemetry.addData("R2 cm optical", "%.2f cm", rangeSensor2.cmOptical());
        telemetry.addData("R2 cm", "%.2f cm", rangeSensor2.getDistance(DistanceUnit.CM));
        telemetry.update();



    }

    @Override
    public void stop() {

    }

}