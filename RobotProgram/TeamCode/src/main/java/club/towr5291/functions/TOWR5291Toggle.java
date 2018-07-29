package club.towr5291.functions;

/**
 * Created by Ian Haden TOWR5291 on 7/15/2018.
 * This class was created as a demonstration on how to create and use a class especially when code is re used
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
 * Ian Haden 07/15/2018 -> Initial creation
 * Ian Haden 07/18/2018 -> Added comments, renamed some variables so they represent their respective functions
 */

public class TOWR5291Toggle {
    private double debounceTimer = 100;
    private boolean state = false;
    private boolean lastState;
    private long timer;

    /**
     * Constructor, initialises all variables
     * sets the initial state to FALSE
     */
    public TOWR5291Toggle(){
        this.timer = System.currentTimeMillis() - (long)this.debounceTimer;
        this.lastState = false;
    }

    /**
     * Constructor, initialises all variables
     * over rides the initial states in the event the initial states could be TRUE rather than the default FALSE
     * @param inputState Takes a true or false value to set the last state.
     */
    public TOWR5291Toggle(boolean inputState){
        this.timer = System.currentTimeMillis() - (long)this.debounceTimer;
        this.lastState = inputState;
    }

    /**
     * Toggle the current state of the input if the debounce has expired and the state has changed
     * @param input Takes a true or false value to toggle the state.
     */
    public boolean toggleState(boolean input) {
        if (this.lastState != input) {
            this.lastState = input;
            if ((this.timer + (long)this.debounceTimer ) < ( System.currentTimeMillis())) {
                if (input == true) {
                    this.state = !this.state;
                    this.timer = System.currentTimeMillis();
                }
            }
        }
        return this.state;
    }

    /**
     * Set the value of the debounce
     *
     * @param value Set the debounce value
     */
    public void setDebounce(double value){
        debounceTimer = value;
    }

}
