package club.towr5291.functions;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by Wyatt on 7/19/2018.
 */

public class TOWR5291ToggleTick {

    public int TOWR5291ToggleTickNumber;
    private double increment = 1;
    private int TOWR5291ToggleTickMax = 10;
    private int TOWR5291ToggleTickMin = 1;
    private int TOWR5291ToggleTickReset = 1;
    private long timer;
    private double debounceTimer = 100;
    private boolean lastState;

    public TOWR5291ToggleTick(boolean inputState){
        this.timer = System.currentTimeMillis() - (long)this.debounceTimer;
        this.lastState = inputState;
    }

    public void addTick(boolean input){
        if (this.lastState != input) {
            this.lastState = input;
            if (input == true) {
                this.TOWR5291ToggleTickNumber += increment;
                if (this.TOWR5291ToggleTickNumber > this.TOWR5291ToggleTickMax) {
                    this.TOWR5291ToggleTickNumber = TOWR5291ToggleTickReset;
                }
                if (this.TOWR5291ToggleTickNumber <= TOWR5291ToggleTickMin) {
                    this.TOWR5291ToggleTickNumber = TOWR5291ToggleTickMin;
                }
            }
        }
    }

    public void subtractTick(boolean input){
        if (this.lastState != input) {
            this.lastState = input;
            if (input == true) {
                this.TOWR5291ToggleTickNumber -= increment;
                if (this.TOWR5291ToggleTickNumber > this.TOWR5291ToggleTickMax) {
                    this.TOWR5291ToggleTickNumber = TOWR5291ToggleTickReset;
                }
                if (this.TOWR5291ToggleTickNumber <= TOWR5291ToggleTickMin) {
                    this.TOWR5291ToggleTickNumber = TOWR5291ToggleTickMin;
                }
            }
        }
    }


    public void setDebounceTimerTime(int Mytimer){
        this.debounceTimer = Mytimer;
    }

    public void setIncrement(int incrementAmount){
        this.increment = incrementAmount;
    }

    public void setMaxTick(int maxTick){
        this.TOWR5291ToggleTickMax = maxTick;
    }

    public void setTOWR5291ToggleTickReset(int resetTo){
        this.TOWR5291ToggleTickReset = resetTo;
    }

    public void setTOWR5291ToggleTickMin(int Minimum){
        this.TOWR5291ToggleTickMin = Minimum;
    }

    public int getNumberTick(){
        return this.TOWR5291ToggleTickNumber;
    }

    public int getTOWR5291ToggleTickMin() {
        return this.TOWR5291ToggleTickMin;
    }

    public int getTOWR5291ToggleTickMax() {
        return this.TOWR5291ToggleTickMax;
    }

    public void restTick(){
        this.TOWR5291ToggleTickNumber = this.TOWR5291ToggleTickReset;
    }

    public void setTOWR5291ToggleTickNumberValue(int Value){
        this.TOWR5291ToggleTickNumber = Value;
    }
}
