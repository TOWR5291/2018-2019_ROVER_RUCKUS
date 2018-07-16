/* Copyright (c) 2016 PSM

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package club.towr5291.Concepts;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

/*
 * This is an example LinearOpMode that shows how to use the digital inputs and outputs on the
 * the Modern Robotics Device Interface Module.  In addition, it shows how to use the Red and Blue LED
 *
 * This op mode assumes that there is a Device Interface Module attached, named 'dim'.
 * On this DIM there is a digital input named 'digin' and an output named 'digout'
 *
 * To fully exercise this sample, connect pin 3 of the digin connector to pin 3 of the digout.
 * Note: Pin 1 is indicated by the black stripe, so pin 3 is at the opposite end.
 *
 * The X button on the gamepad will be used to activate the digital output pin.
 * The Red/Blue LED will be used to indicate the state of the digital input pin.
 * Blue = false (0V), Red = true (5V)
 * If the two pins are linked, the gamepad will change the LED color.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
*/
@TeleOp(name = "Sensor: Analog Inputs", group = "5291Concept")
@Disabled
public class ConceptSensorAnalogInput extends LinearOpMode {

  @Override
  public void runOpMode() {

    double                inputLineSensor1;     // Input State
    double                inputLineSensor2;     // Input State
    double                inputLineSensor3;     // Input State
    AnalogInput           LineSensor1;          // Device Object
    AnalogInput           LineSensor2;          // Device Object
    AnalogInput           LineSensor3;          // Device Object

    // get a reference to a Modern Robotics DIM, and IO channels.
    //dim = hardwareMap.get(DeviceInterfaceModule.class, "dim");   //  Use generic form of device mapping

    LineSensor1 = hardwareMap.get(AnalogInput.class, "linesensor1");
    LineSensor2 = hardwareMap.get(AnalogInput.class, "linesensor2");
    LineSensor3 = hardwareMap.get(AnalogInput.class, "linesensor3");

    // wait for the start button to be pressed.
    telemetry.addData(">", "Press play, and then user X button to set DigOut");
    telemetry.update();
    waitForStart();

    while (opModeIsActive())  {

        inputLineSensor1 = LineSensor1.getVoltage();    //  Read the input pin
        inputLineSensor2 = LineSensor2.getVoltage();    //  Read the input pin
        inputLineSensor3 = LineSensor3.getVoltage();    //  Read the input pin

        // Display input pin state on LEDs
        telemetry.addData("Line1", inputLineSensor1 );
        telemetry.addData("Line2", inputLineSensor2 );
        telemetry.addData("Line2", inputLineSensor3 );
        telemetry.update();
    }
  }
}
