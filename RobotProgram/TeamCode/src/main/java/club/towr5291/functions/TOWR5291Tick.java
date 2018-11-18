package club.towr5291.functions;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import club.towr5291.functions.TOWR5291Utils;

/**
 * Created by Wyatt Ashley TOWR5291 on 7/19/2018.
 * This class creates an object that increments up to a set point then rolls back over to the minimum setpoint
 * as well as can decrement down to a minimum set point then roll over to a maximum setpoint
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
 * Wyatt Ashley 07/20/2018 ->  Initial creation
 * Ian Haden 07/22/2018 -> Added comments and removed some unneed steps, renamed some variables so they represent their respective functions
 */
public class TOWR5291Tick {

    private double TOWR5291ToggleTickNumber = 0;
    private double increment = 0.1;
    private double TOWR5291ToggleTickMax = 1;
    private double TOWR5291ToggleTickMin = 0;
    private boolean lastStateIncrement = false;
    private boolean lastStateDecrement = false;
    private int debounceTime = 100;
    private ElapsedTime debounceTimerIncrement;
    private ElapsedTime debounceTimerDecrement;
    boolean rollover = true;

    /**
     * Constructor, initialises all variables
     *
     */
    public TOWR5291Tick(){
        this.debounceTimerIncrement = new ElapsedTime();
        this.debounceTimerDecrement = new ElapsedTime();
    }

    /**
     * Constructor, initialises all variables
     * over rides the initial states in the event the initial states could be TRUE rather than th default FALSE
     */
    public TOWR5291Tick(boolean initialStateIncrement, boolean initialStateDecrement){
        this.debounceTimerIncrement = new ElapsedTime();
        this.debounceTimerDecrement = new ElapsedTime();
        this.lastStateIncrement = initialStateIncrement;
        this.lastStateDecrement = initialStateDecrement;
    }

    /**
     * Increment the value of the object and check if it has reached its maximum value
     * If the maximum value has been reached then roll over to the minimum
     *
     * @param input Takes a true or false value to increment the value.
     */
    public void incrementTick(boolean input){
        if (this.lastStateIncrement != input && debounceTimerIncrement.milliseconds() > this.debounceTime) {
            this.lastStateIncrement = input;
            debounceTimerIncrement.reset();
            if (input == true) {
                this.TOWR5291ToggleTickNumber += this.increment;
                if (this.TOWR5291ToggleTickNumber > this.TOWR5291ToggleTickMax) {
                    if (rollover) {
                        this.TOWR5291ToggleTickNumber = TOWR5291ToggleTickMin;
                    } else {
                        //no rollover so return it to to max
                        this.TOWR5291ToggleTickNumber = TOWR5291ToggleTickMax;
                    }

                }
            }
        }
    }

    /**
     * Decrement the value of the object and check if it has reached the minimum value
     * if the minimum has been reached roll back over to the maximum
     *
     * @param input Takes a true or false value to decrement that value
     */
    public void decrementTick(boolean input){
        if (this.lastStateDecrement != input && debounceTimerDecrement.milliseconds() > this.debounceTime) {
            this.lastStateDecrement = input;
            debounceTimerDecrement.reset();
            if (input == true) {
                this.TOWR5291ToggleTickNumber -= this.increment;
                if (this.TOWR5291ToggleTickNumber < TOWR5291ToggleTickMin) {
                    if (rollover) {
                        this.TOWR5291ToggleTickNumber = TOWR5291ToggleTickMax;
                    } else {
                        this.TOWR5291ToggleTickNumber = TOWR5291ToggleTickMin;
                    }
                }
            }
        }
    }

    /**
     * Set the value of the increment and decrement
     *
     * @param onoff Set whether the object should rollover or stay at Max or Min
     */
    public void setRollOver(boolean onoff){
        this.rollover = onoff;
    }

    /**
     * Set the value of the debounce time
     *
     * @param time set the debounce time
     */
    public void setDebounceTime (int time) {this.debounceTime = time;}

    /**
     * Set the value of the increment and decrement
     *
     * @param incrementAmount Value each time a change occurs
     */
    public void setTickIncrement(double incrementAmount){
        this.increment = incrementAmount;
    }

    /**
     * Set the maximum value of the object
     *
     * @param maximum Maximum Value before rolling over
     */
    public void setTickMax(double maximum){
        this.TOWR5291ToggleTickMax = maximum;
    }

    /**
     * Set the minimum value of the object
     *
     * @param minimum Minimum Value before rolling over
     */
    public void setTickMin(double minimum){
        this.TOWR5291ToggleTickMin = minimum;
        if (this.TOWR5291ToggleTickNumber < this.TOWR5291ToggleTickMin)
            this.TOWR5291ToggleTickNumber = this.TOWR5291ToggleTickMin;
    }

    /**
     * Get the current value
     */
    public double getTickCurrValue(){
        return TOWR5291Utils.round(this.TOWR5291ToggleTickNumber,(int)(1/this.increment)/10);
    }

    /**
     * Get the minimum setting value
     */
    public double getTickMin() {
        return this.TOWR5291ToggleTickMin;
    }

    /**
     * Get the maximum setting value
     */
    public double getTickMax() {
        return this.TOWR5291ToggleTickMax;
    }

    /**
     * Set or Over ride the current value
     */
    public void setTickValue(double value){
        if ((value <= TOWR5291ToggleTickMax) && (value >= TOWR5291ToggleTickMin)){
            this.TOWR5291ToggleTickNumber = value;
        }
    }

}
