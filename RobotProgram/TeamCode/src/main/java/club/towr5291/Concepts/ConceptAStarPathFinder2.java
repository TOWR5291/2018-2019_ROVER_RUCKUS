package club.towr5291.Concepts;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;

import club.towr5291.astarpathfinder.A0Star;
import club.towr5291.astarpathfinder.sixValues;
import club.towr5291.functions.AStarGetPathBasic;
import club.towr5291.functions.AStarGetPathEnhanced;
import club.towr5291.functions.AStarGetPathVer2;
import club.towr5291.functions.FileLogger;


/**
 * Created by ianhaden on 2/09/16.
 *
 *  TOWR 5291
 Copyright (c) 2016 TOWR5291
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.

 Written by Ian Haden October 2016
 */

@Autonomous(name="Concept: A Star Path Finder 2", group="5291Concept")
@Disabled
public class ConceptAStarPathFinder2 extends OpMode {


    private static final String TAG = "ConceptAStarPathFinder2";

    //set up the variables for the logger
    private String startDate;
    private ElapsedTime runtime = new ElapsedTime();
    private FileLogger fileLogger;

    public sixValues[] pathValues = new sixValues[1000];

    private A0Star a0Star = new A0Star();
    String fieldOutput = "";

    //public AStarGetPathBasic pathValues2 = new AStarGetPathBasic();
    //public AStarGetPathEnhanced pathValues2 = new AStarGetPathEnhanced();
    public AStarGetPathVer2 pathValues2 = new AStarGetPathVer2();

    public int pathIndex = 0;

    int debug = 3;
    /*
    * Code to run ONCE when the driver hits INIT
    */
    @Override
    public void init()
    {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());
        runtime.reset();
        telemetry.addData("FileLogger: ", runtime.toString());
        fileLogger = new FileLogger(runtime);
        fileLogger.open();
        telemetry.addData("FileLogger Op Out File: ", fileLogger.getFilename());
        fileLogger.write("Time,SysMS,Thread,Event,Desc");
        fileLogger.writeEvent("init()","Log Started");

        // Send telemetry message to signify robot waiting;
        telemetry.update();

        fileLogger.writeEvent("init()","Field");
        //outputing field
        fileLogger.writeEvent("init()", "Writing Field Array");

        //load start point
        int startX = 84;  //122
        int startY = 120;  //122
        int startZ = 0;
        int endX = 12;     //12
        int endY = 24;     //80
        int endDir = 270;
        int BlueRed;

        pathValues = pathValues2.findPathAStar(startX, startY, startZ, endX, endY, endDir);  //for enhanced

        String[][] mapComplete = new String[A0Star.FIELDWIDTH][A0Star.FIELDWIDTH];
        fieldOutput = "";

        if (startX < startY)
        {
            BlueRed = 2;  //RED
        }
        else
        {
            BlueRed = 1;  //BLUE
        }

        for (int y = 0; y < a0Star.fieldLength; y++)
        {
            for (int x = 0; x < a0Star.fieldWidth; x++)
            {
                if (BlueRed == 2) {
                    if (a0Star.walkableRed[y][x]) {
                        if ((x == startX) && (y == startY)) {
                            mapComplete[y][x] = "S";
                        } else {
                            mapComplete[y][x] = "1";
                        }
                    } else {
                        if ((x == startX) && (y == startY)) {
                            mapComplete[y][x] = "1";
                        } else {
                            mapComplete[y][x] = "0";
                        }
                    }
                }
                else
                {
                    if (a0Star.walkableBlue[y][x]) {
                        if ((x == startX) && (y == startY)) {
                            mapComplete[y][x] = "S";
                        } else {
                            mapComplete[y][x] = "1";
                        }
                    } else {
                        if ((x == startX) && (y == startY)) {
                            mapComplete[y][x] = "1";
                        } else {
                            mapComplete[y][x] = "0";
                        }
                    }
                }
            }
        }

        fieldOutput = "MAP INIT COMPLETE";

        fileLogger.writeEvent("loop()", "Plotting results");

        //plot out path..
        for (int i = 0; i < pathValues.length; i++)
        {
            //fileLogger.writeEvent("init()","Path " + pathValues[i].val1 + " " + pathValues[i].val2 + " " + pathValues[i].val3 + " Dir:= " + pathValues[i].val4 );
            if (pathValues[i].val1 != 0)
                mapComplete[(int)pathValues[i].val3][(int)pathValues[i].val2] = "P";
            if ((pathValues[i].val2 == startX) && (pathValues[i].val3 == startY))
            {
                mapComplete[(int) pathValues[i].val3][(int) pathValues[i].val2] = "S";
            }
        }
        mapComplete[endY][endX] = "E";
        fieldOutput ="";
        for (int y = 0; y < a0Star.fieldLength; y++)
        {
            for (int x = 0; x < a0Star.fieldWidth; x++)
            {
                fieldOutput = "" + fieldOutput + mapComplete[y][x];
            }
            fileLogger.writeEvent("loop()", fieldOutput);
            Log.d(TAG, "Path Through Field " + fieldOutput);
            fieldOutput = "";
        }

//decode the output to commands



        //load path in Hashmap
        boolean dirChanged;
        boolean processingAStarSteps = true;
        int startSegment = 0;
        int startStraightSection = 0;
        int numberOfMoves = 0;
        int key = 0;
        int lastDirection = 0;
        int lasti =0;
        String strAngleChange = "RT00";
        boolean endOfAStarSequenceFound = false;

        while (processingAStarSteps)
        {
            numberOfMoves = 0;
            for (int i = startSegment; i < pathValues.length; i++)
            {
                numberOfMoves ++;
                if (debug >= 2)
                {
                    fileLogger.writeEvent(TAG,"Path " + pathValues[i].val1 + " " + pathValues[i].val2 + " " + pathValues[i].val3 + " Dir:= " + pathValues[i].val4 );
                }

                if (((int)pathValues[i].val1 == 0) && ((int)pathValues[i].val2 == 0) && ((int)pathValues[i].val3 == 0))
                {
                    if (debug >= 2)
                    {
                        fileLogger.writeEvent(TAG,"End Detected" );
                    }
                    //end of the sequence,
                    lastDirection = (int)pathValues[i-1].val4;
                    processingAStarSteps = false;
                    lasti = i;
                }
                //need to check if the first step is in a different direction that the start
                if (i == 0) {
                    if (startZ != pathValues[i].val4) {  //need to turn

                        strAngleChange = getAngle(startZ, (int) pathValues[i].val4);
                        if (debug >= 2) {
                            fileLogger.writeEvent(TAG, "First Step Need to turn Robot " + strAngleChange + " Path " + pathValues[i].val1 + " " + pathValues[i].val2 + " " + pathValues[i].val3 + " Dir:= " + pathValues[i].val4);
                            fileLogger.writeEvent(TAG, "Adding Command (" + key + ", 10, " + strAngleChange + ", 0, 0, 0, 0, 0, 0, 1, false) ");
                        }
                        //autonomousStepsAStar.put(String.valueOf(key), new LibraryStateSegAuto(key, 10, strAngleChange, 0, 0, 0, 0, 0, 0, 1, false));
                        key++;
                        dirChanged = true;
                    } else {
                        dirChanged = false;    //no change in direction
                    }
                }
                else  //work out the sequnce not the first step
                {
                    if (pathValues[i-1].val4 != pathValues[i].val4) {  //need to turn
                        strAngleChange = getAngle((int)pathValues[i-1].val4, (int)pathValues[i].val4);
                        dirChanged = true;
                    }
                    else
                    {
                        dirChanged = false;    //no change in direction
                    }
                }
                if ((dirChanged) || (!processingAStarSteps))  //found end of segment
                {
                    int AStarPathAngle;
                    if (i == 0)
                    {
                        AStarPathAngle = startZ;
                    }
                    else
                    {
                        AStarPathAngle = (int)pathValues[i-1].val4;
                    }
                    switch (AStarPathAngle)
                    {
                        case 0:
                        case 90:
                        case 180:
                        case 270:
                            if (debug >= 2)
                            {
                                fileLogger.writeEvent(TAG,"Heading on a Straight line " + (numberOfMoves-1) + " Path");
                                fileLogger.writeEvent(TAG,"Adding Command (" + key +", 10, " + "FW" + (numberOfMoves-1) + ", 0, 0, 0, 0, 0, 0, 1, false) ");
                            }
                            key++;
                            //autonomousStepsAStar.put(String.valueOf(key), new LibraryStateSegAuto (key,   10, "FW" + numberOfMoves , 0,    0,    0,    0,    0,    0,    1,    false));
                            numberOfMoves = 0;
                            break;
                        case 45:
                        case 135:
                        case 225:
                        case 315:
                            if (debug >= 2)
                            {
                                fileLogger.writeEvent(TAG,"Heading on a Straight line " + (int) ((numberOfMoves-1) * 1.4142) + " Path");
                                fileLogger.writeEvent(TAG, "Adding Command (" + key + ", 10, " + "FW" + (int) ((numberOfMoves-1) * 1.4142) + ", 0, 0, 0, 0, 0, 0, 1, false) ");
                            }
                            //autonomousStepsAStar.put(String.valueOf(key), new LibraryStateSegAuto (key,   10, "FW" + (int)(numberOfMoves * 1.4142 ) , 0,    0,    0,    0,    0,    0,    1,    false));
                            numberOfMoves = 0;
                            key++;
                            break;
                    }
                    if (debug >= 2)
                    {
                        fileLogger.writeEvent(TAG,"Need to turn Robot " + strAngleChange + " Path " + pathValues[i].val1 + " " + pathValues[i].val2 + " " + pathValues[i].val3 + " Dir:= " + pathValues[i].val4 );
                        fileLogger.writeEvent(TAG,"Adding Command (" + key +", 10, "+ strAngleChange + ", 0, 0, 0, 0, 0, 0, 1, false) ");
                    }
                    //autonomousStepsAStar.put(String.valueOf(key), new LibraryStateSegAuto (key,   10,  strAngleChange, 0,    0,    0,    0,    0,    0,    1,    false));
                    key++;
                }
                if (!processingAStarSteps)
                    break;

            }
            //need to work out the direction we are facing and the required direction
            if ((lastDirection != endDir) && (processingAStarSteps == false)) {
                if (debug >= 2)
                {
                    fileLogger.writeEvent(TAG,"Sraight Moves Robot End Of Sequence - Need to Trun Robot");
                }
                strAngleChange = getAngle((int)pathValues[lasti - 1].val4, endDir);
                fileLogger.writeEvent(TAG,"Adding Command (" + key +", 10, " + strAngleChange + ", 0, 0, 0, 0, 0, 0, 1, false) ");
                //autonomousStepsAStar.put(String.valueOf(key), new LibraryStateSegAuto (key,   10,  strAngleChange, 0,    0,    0,    0,    0,    0,    1,    false));
                key++;
            }

        }
        endOfAStarSequenceFound = false;
        //mCurrentStepState = stepState.STATE_ASTAR_INIT;









        fileLogger.writeEvent("init()","Init Complete");
    }





    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop()
    {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start()
    {
        fileLogger.writeEvent("start()","START PRESSED: ");
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop()
    {

        telemetry.update();
    }


    private String getAngle(int angle1, int angle2)
    {
        if (debug >= 2)
        {
            fileLogger.writeEvent(TAG, "Getangle - Current Angle1:= " + angle1 + " Desired Angle2:= " + angle2);
        }

        switch (angle1)
        {
            case 0:
                switch (angle2)
                {
                    case 45:
                        return "RT45";
                    case 90:
                        return "RT90";
                    case 135:
                        return "RT135";
                    case 180:
                        return "RT180";
                    case 225:
                        return "LT135";
                    case 270:
                        return "LT90";
                    case 315:
                        return "LT45";
                }
                break;
            case 45:
                switch (angle2)
                {
                    case 0:
                        return "LT45";
                    case 90:
                        return "RT45";
                    case 135:
                        return "RT90";
                    case 180:
                        return "RT135";
                    case 225:
                        return "RT180";
                    case 270:
                        return "LT135";
                    case 315:
                        return "LT90";
                }
                break;
            case 90:
                switch (angle2)
                {
                    case 0:
                        return "LT90";
                    case 45:
                        return "LT45";
                    case 135:
                        return "RT45";
                    case 180:
                        return "RT90";
                    case 225:
                        return "RT135";
                    case 270:
                        return "RT180";
                    case 315:
                        return "LT135";
                }
                break;
            case 135:
                switch (angle2)
                {
                    case 0:
                        return "LT135";
                    case 45:
                        return "LT90";
                    case 90:
                        return "LT45";
                    case 180:
                        return "RT45";
                    case 225:
                        return "RT90";
                    case 270:
                        return "RT135";
                    case 315:
                        return "RT180";
                }
                break;
            case 180:
                switch (angle2)
                {
                    case 0:
                        return "LT180";
                    case 45:
                        return "LT135";
                    case 90:
                        return "LT90";
                    case 135:
                        return "LT45";
                    case 225:
                        return "RT45";
                    case 270:
                        return "RT90";
                    case 315:
                        return "RT135";
                }
                break;
            case 225:
                switch (angle2)
                {
                    case 0:
                        return "RT135";
                    case 45:
                        return "LT180";
                    case 90:
                        return "LT135";
                    case 135:
                        return "LT90";
                    case 180:
                        return "LT45";
                    case 270:
                        return "RT45";
                    case 315:
                        return "RT90";
                }
                break;
            case 270:
                switch (angle2)
                {
                    case 0:
                        return "RT90";
                    case 45:
                        return "RT135";
                    case 90:
                        return "LT180";
                    case 135:
                        return "LT135";
                    case 180:
                        return "LT90";
                    case 225:
                        return "LT45";
                    case 315:
                        return "RT45";
                }
                break;
            case 315:
                switch (angle2)
                {
                    case 0:
                        return "RT45";
                    case 45:
                        return "RT90";
                    case 90:
                        return "RT135";
                    case 135:
                        return "LT180";
                    case 180:
                        return "LT135";
                    case 225:
                        return "LT90";
                    case 270:
                        return "LT45";
                }
                break;
        }
        return "ERROR";
    }



    /*
    * Code to run ONCE after the driver hits STOP
    */
    @Override
    public void stop()
    {
        telemetry.addData("FileLogger Op Stop: ", runtime.toString());
        if (fileLogger != null)
        {
            fileLogger.writeEvent("stop()","Stopped");
            fileLogger.close();
            fileLogger = null;
        }
    }

    //--------------------------------------------------------------------------
    // User Defined Utility functions here....
    //--------------------------------------------------------------------------


}
