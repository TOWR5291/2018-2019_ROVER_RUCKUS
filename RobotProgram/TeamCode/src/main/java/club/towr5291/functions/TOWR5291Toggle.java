package club.towr5291.functions;

public class TOWR5291Toggle {
    private double debounceTimer = 100;
    private boolean state = false;
    private boolean lastState;
    private long timer;

    public TOWR5291Toggle(boolean inputState){
        this.timer = System.currentTimeMillis() - (long)this.debounceTimer;
        this.lastState = inputState;
    }

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

    public void setDebounce(double value){
        debounceTimer = value;
    }

}
