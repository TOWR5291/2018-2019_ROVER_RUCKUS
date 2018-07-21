package club.towr5291.functions;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Wyatt on 7/19/2018.
 */

public class TOWR5291Tick {

    public double TOWR5291ToggleTickNumber = 0.1;
    private double increment = 0.1;
    private double TOWR5291ToggleTickMax = 1;
    private double TOWR5291ToggleTickMin = 0;
    private boolean lastState;
    private boolean lastState1;
    private int debounceTime = 1000;
    private ElapsedTime runtime;
    private ElapsedTime runtime2;

    public TOWR5291Tick(){
         runtime = new ElapsedTime();
         runtime2 = new ElapsedTime();
    }

    public void incrementTick(boolean input){
        if (this.lastState != input && runtime.milliseconds() > debounceTime) {
            this.lastState = input;
            runtime.reset();
            if (input == true) {
                this.TOWR5291ToggleTickNumber += increment;
                if (this.TOWR5291ToggleTickNumber > this.TOWR5291ToggleTickMax) {
                    this.TOWR5291ToggleTickNumber = TOWR5291ToggleTickMin;
                }
                if (this.TOWR5291ToggleTickNumber <= TOWR5291ToggleTickMin) {
                    this.TOWR5291ToggleTickNumber = 0.0;
                }
            }
        }
    }

    public void decrementTick(boolean input){
        if (this.lastState1 != input || runtime2.milliseconds() > debounceTime) {
            this.lastState1 = input;
            runtime2.reset();
            if (input == true) {
                this.TOWR5291ToggleTickNumber -= increment;
                if (this.TOWR5291ToggleTickNumber > this.TOWR5291ToggleTickMax) {
                    this.TOWR5291ToggleTickNumber = TOWR5291ToggleTickMin;
                }
                if (this.TOWR5291ToggleTickNumber <= TOWR5291ToggleTickMin) {
                    this.TOWR5291ToggleTickNumber = 1.0;
                }
            }
        }
    }

    public void setIncrement(double incrementAmount){
        this.increment = incrementAmount;
    }

    public void setMaxTick(double maxTick){
        this.TOWR5291ToggleTickMax = maxTick;
    }

    public void setTOWR5291ToggleTickMin(double Minimum){
        this.TOWR5291ToggleTickMin = Minimum;
    }

    public double getNumberTick(){
        return roundDown(this.TOWR5291ToggleTickNumber);
    }

    private static double roundDown(double d) {
        return (long) (d * 1e1) / 1e1;
    }

    public double getTOWR5291ToggleTickMin() {
        return this.TOWR5291ToggleTickMin;
    }

    public double getTOWR5291ToggleTickMax() {
        return this.TOWR5291ToggleTickMax;
    }

    public void setTOWR5291ToggleTickNumberValue(double Value){
        this.TOWR5291ToggleTickNumber = Value;
    }
}
