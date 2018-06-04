package club.towr5291.Concepts;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import android.util.Log;

/**
 * Created by ianhaden on 30/10/2016.
 */
@Autonomous(name="Concept: Autonomous Menu", group="5291Concept")
@Disabled
public class ConceptAutonomousOptions extends LinearOpMode{

    SharedPreferences sharedPreferences;

    @Override
    public void runOpMode() throws InterruptedException {
        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);

        String allianceColor = sharedPreferences.getString("club.towr5291.Autonomous.Color", "null");
        String alliancePosition = sharedPreferences.getString("club.towr5291.Autonomous.Position", "null");
        int delay = sharedPreferences.getInt("club.towr5291.Autonomous.Delay", 0);

        Log.d("GetStuff", "allianceColor " + allianceColor);
        Log.d("GetStuff", "alliancePosition " + alliancePosition);
        Log.d("GetStuff", "delay " + delay);
    }



}
