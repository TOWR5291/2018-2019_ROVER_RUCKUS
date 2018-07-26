package club.towr5291.functions;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by ianhaden on 2/6/18.
 * https://www.codeproject.com/Articles/36459/PID-process-control-a-Cruise-Control-example
 * and initial concept from wizards 9794
 *
 * Modification history
 * Edited by:
 * Ian Haden 07/15/2018 -> Initial creation
 * Ian Haden 07/26/2018 -> Added comments, renamed some variables so they represent their respective functions
 */

public class TOWR5291PID {
    private double PIDPreviousDifference;
    private double PIDPreviousTime;
    private double ProportionGain;
    private double IntergralGain;
    private double DerivativeGain;
    private double Integral;

    /**
     * Constructor, initialises all variables
     *
     */
    public TOWR5291PID (){
        this.PIDPreviousTime = 0;
        this.PIDPreviousDifference = 0;
        this.ProportionGain = 0;
        this.IntergralGain = 0;
        this.DerivativeGain = 0;
    }

    /**
     * Constructor, initialises all variables
     * over rides the initial states
     * @param PIDcurrentStartTime sets the time the PID should start measuring.
     * @param ActualValue The current Value of the reading.
     * @param DesiredValue The desired Value.
     * @param pGain The proportional Gain Setting.
     * @param iGain The integral Gain Setting.
     * @param dGain The differential Gain Setting.
     */
    public TOWR5291PID (ElapsedTime PIDcurrentStartTime, double ActualValue, double DesiredValue, double pGain, double iGain, double dGain){
        this.PIDPreviousTime = PIDcurrentStartTime.milliseconds() / 1000;
        this.PIDPreviousDifference = ActualValue - DesiredValue;
        this.ProportionGain = pGain;
        this.IntergralGain = iGain;
        this.DerivativeGain = dGain;
    }

    /**
     * Reset the PID and start measuring again
     * over rides the initial states
     * @param PIDcurrentStartTime sets the time the PID should start measuring.
     * @param ActualValue The current Value of the reading.
     * @param DesiredValue The desired Value.
     * @param pGain The proportional Gain Setting.
     * @param iGain The integral Gain Setting.
     * @param dGain The differential Gain Setting.
     */
    public void PIDReset (ElapsedTime PIDcurrentStartTime, double ActualValue, double DesiredValue, double pGain, double iGain, double dGain){
        this.PIDPreviousTime = PIDcurrentStartTime.milliseconds() / 1000;
        this.PIDPreviousDifference = ActualValue - DesiredValue;
        this.ProportionGain = pGain;
        this.IntergralGain = iGain;
        this.DerivativeGain = dGain;
    }

    /**
     * Calculate the current error suing PID Control
     * Returns the error
     * @param PIDcurrentTime sets the current time of the measurement.
     * @param ActualValue The current Value of the reading.
     * @param DesiredValue The desired Value.
     */
    public double PIDCorrection (ElapsedTime PIDcurrentTime, double ActualValue, double DesiredValue) {
        double PIDCurrentDifference, pCorrection, iCorrection, dCorrection, Derivative, correction, Dt;

        //determine how off the robot is from the desired position
        PIDCurrentDifference = ActualValue - DesiredValue;

        //calculate P Correction
        pCorrection = PIDCurrentDifference * ProportionGain;

        //calculate Delta Time
        Dt = ((PIDcurrentTime.milliseconds() - PIDPreviousTime) / 1000);

        //calculate I Correction
        //determine the area under the curve
        //integral = integral + (error * Dt);
        Integral += ((PIDPreviousDifference + PIDCurrentDifference) / 2) * Dt;
        iCorrection = Integral * IntergralGain;

        //Calculate D Correction
        //determine slope of corrections
        //derivative = (error - preError) / Dt;
        Derivative = (PIDPreviousDifference - PIDCurrentDifference) / Dt;
        dCorrection = Derivative * DerivativeGain;

        //calculate overall correction
        correction =  pCorrection + iCorrection + dCorrection;

        return correction;
    }

}
