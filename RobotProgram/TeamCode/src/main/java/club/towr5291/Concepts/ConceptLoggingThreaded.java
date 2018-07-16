package club.towr5291.Concepts;

import android.content.Context;
import android.os.AsyncTask;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import club.towr5291.functions.FileLogger;

/**
 * Created by Ian Haden on 11/7/2016.
 */

@TeleOp(name = "Concept Logging Thread", group = "5291Concept")
@Disabled
public class ConceptLoggingThreaded extends OpMode {

    //set up the variables for the logger
    final String TAG = "Concept Logging";
    private ElapsedTime runtime = new ElapsedTime();
    private FileLogger fileLogger;
    private int debug = 3;

    private int loop = 0;
    int progressValue  = 0;
    ConceptLoggingThreadedAsync backgroundTask = new ConceptLoggingThreadedAsync();

    public class ConceptLoggingThreadedAsync extends AsyncTask<Void,Integer,Void> {
        Context context;
        //int progressValue, result;
        int i = 0;

        @Override
        protected Void doInBackground(Void... params) {
            synchronized (this) {
                while (true) {
                    try {
                        if (i>100000) i = 0;
                        wait(500);
                        i++;
                        publishProgress(i);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                }
            }


        }

        @Override
        protected void onPreExecute() {
        }

        @Override
        protected void onPostExecute(Void aVoid) {
        }

        @Override
        protected void onProgressUpdate(Integer... values) {
            progressValue = values[0];
        }

        @Override
        protected void onCancelled(Void aVoid) {
        }

        @Override
        protected void onCancelled() {
        }


    }

    @Override
    public void init() {
        //start the log
        if (debug >= 1) {
            fileLogger = new FileLogger(runtime);
            fileLogger.open();
            fileLogger.write("Time,SysMS,Thread,Event,Desc");
            fileLogger.writeEvent(TAG, "Log Started");
        }
        backgroundTask.execute();
    }

    @Override
    public void loop() {

        //this will log only when debug is at level 3 or above
        fileLogger.writeEvent(1, TAG, "In Forground Loop # " + loop + " Background Loop # " + progressValue);
        loop++;
    }

    @Override
    public void stop() {
        //stop the log
        if (fileLogger != null) {
            fileLogger.writeEvent(TAG, "Stopped");
            fileLogger.close();
            fileLogger = null;
        }
        backgroundTask.cancel(true);
    }
}


