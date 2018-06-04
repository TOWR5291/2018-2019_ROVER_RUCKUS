package club.towr5291.libraries;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by ianhaden on 2/6/18.
 * https://www.codeproject.com/Articles/36459/PID-process-control-a-Cruise-Control-example
 * and initial concept from wizards 9794
 */

public class TOWR5291PID {
    private double PIDPreviousDifference;
    private double PIDPreviousTime;
    private double ProportionGain;
    private double IntergralGain;
    private double DerivativeGain;
    private double Integral;

    public TOWR5291PID (ElapsedTime PIDcurrentStartTime, double ActualValue, double DesiredValue, double pGain, double iGain, double dGain){
        this.PIDPreviousTime = PIDcurrentStartTime.milliseconds() / 1000;
        this.PIDPreviousDifference = ActualValue - DesiredValue;
        this.ProportionGain = pGain;
        this.IntergralGain = iGain;
        this.DerivativeGain = dGain;
    }

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
