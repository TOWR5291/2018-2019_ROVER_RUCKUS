package club.towr5291.Concepts;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import club.towr5291.functions.MultiplexAdaFruitColorSensor;


/**
 * Created by Ian Haden on 12/19/2016.
 * Requires Adafruit I2C Multiplexer
 */


/*
 * MIT License
 *
 * Copyright (c) 2016 Chris D
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
 */


/**
 * Created by Chris D on 10/5/2016
 *
 * In this example, you need to create a device configuration that lists two
 * "I2C Device"s, one named "mux" and the other named "ada". There are two
 * Adafruit color sensors plugged into the I2C multiplexer on ports 0 and 3.
 */
@TeleOp(name="MultiplexColorSensorTest", group="5291Concept")
@Disabled
public class ConceptDualColourSensor extends OpMode
{
    MultiplexAdaFruitColorSensor muxColor;
    int[] ports = {0, 3};

    @Override
    public void init() {
        int milliSeconds = 48;
        muxColor = new MultiplexAdaFruitColorSensor(hardwareMap, "mux", "ada",
                ports, milliSeconds,
                MultiplexAdaFruitColorSensor.GAIN_16X);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        muxColor.startPolling();
    }

    @Override
    public void loop() {
        for (int i=0; i<ports.length; i++) {
            int[] crgb = muxColor.getCRGB(ports[i]);

            telemetry.addLine("Sensor " + ports[i]);
            telemetry.addData("CRGB", "%5d %5d %5d %5d",
                    crgb[0], crgb[1], crgb[2], crgb[3]);
        }

        telemetry.update();
    }

    @Override
    public void stop() {

    }
}
