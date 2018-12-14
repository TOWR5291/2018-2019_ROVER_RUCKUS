package club.towr5291.functions;

import android.os.Environment;
import android.util.Log;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.HashMap;

import club.towr5291.libraries.LibraryStateSegAutoRoverRuckus;
import club.towr5291.libraries.robotConfig;

/**
 * Created by Ian Haden on 3/12/17.
 * TOWR 5291 ReadStepsFile
 Copyright (c) 2017 TOWR5291
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
2017-03-11 - Ian Haden - Initial creation

 */

public class ReadStepFileRoverRuckus {

    private HashMap<String,LibraryStateSegAutoRoverRuckus> autonomousStepsFromFile;
    private int numberLoadedSteps = 1;
    private final int positionTimeout      = 0;
    private final int positionComnmand     = 1;
    private final int positionDistance     = 2;
    private final int positionPower        = 3;
    private final int positionParallel     = 4;
    private final int positionLastPosition = 5;
    private final int positionParm1        = 6;
    private final int positionParm2        = 7;
    private final int positionParm3        = 8;
    private final int positionParm4        = 9;
    private final int positionParm5        = 10;
    private final int positionParm6        = 11;

    public int getNumberLoadedSteps() {
        return numberLoadedSteps;
    }

    public void setNumberLoadedSteps(int numberLoadedSteps) {
        this.numberLoadedSteps = numberLoadedSteps;
    }

    public ReadStepFileRoverRuckus() {
        this.numberLoadedSteps = 1;
        this.autonomousStepsFromFile = new HashMap<>();
    }

    public HashMap<String,LibraryStateSegAutoRoverRuckus> activeSteps() {
        return autonomousStepsFromFile;
    }

    private void loadSteps(int timeOut, String command, double distance, double power, boolean parallel, boolean lastPos, double parm1, double parm2, double parm3, double parm4, double parm5, double parm6)
    {
        autonomousStepsFromFile.put(String.valueOf(this.getNumberLoadedSteps()), new LibraryStateSegAutoRoverRuckus (this.getNumberLoadedSteps(), timeOut, command, distance, power, parallel, lastPos, parm1, parm2, parm3, parm4, parm5, parm6));
        this.numberLoadedSteps++;
    }

    public HashMap<String,LibraryStateSegAutoRoverRuckus> insertSteps(int timeOut, String command, double distance,  double power, boolean parallel, boolean lastPos, double parm1, double parm2, double parm3, double parm4, double parm5, double parm6, int insertlocation)
    {
        Log.d("insertSteps", " timout " + timeOut + " command " + command + " distance " + distance + "  power " + power + " parallel " + parallel + " lastPos " + lastPos + " parm1 " + parm1 + " parm2 " + parm2 + " parm3 " + parm3 + " parm4 " + parm4 + " parm5 " + parm5 + " parm6 " + parm6);
        HashMap<String,LibraryStateSegAutoRoverRuckus> autonomousStepsTemp = new HashMap<String,LibraryStateSegAutoRoverRuckus>();
        LibraryStateSegAutoRoverRuckus processingStepsTemp;

        //move all the steps from current step to a temp location
        for (int loop = insertlocation; loop < this.getNumberLoadedSteps(); loop++)
        {
            processingStepsTemp = autonomousStepsFromFile.get(String.valueOf(loop));
            Log.d("insertSteps", "Reading all the next steps " + loop + " timout " + processingStepsTemp.getmRobotTimeOut() + " command " + processingStepsTemp.getmRobotCommand());
            autonomousStepsTemp.put(String.valueOf(loop), autonomousStepsFromFile.get(String.valueOf(loop)));
        }
        Log.d("insertSteps", "All steps loaded to a temp hasmap");

        //insert the step we want

        autonomousStepsFromFile.put(String.valueOf(insertlocation), new LibraryStateSegAutoRoverRuckus (this.getNumberLoadedSteps(), timeOut, command, distance, power, parallel, lastPos, parm1, parm2, parm3, parm4, parm5, parm6));
        Log.d("insertSteps", "Inserted New step");

        //move all the other steps back into the sequence
        for (int loop = insertlocation; loop < this.getNumberLoadedSteps(); loop++)
        {
            processingStepsTemp = autonomousStepsTemp.get(String.valueOf(loop));
            Log.d("insertSteps", "adding these steps back steps " + (loop + 1) + " timout " + processingStepsTemp.getmRobotTimeOut() + " command " + processingStepsTemp.getmRobotCommand());
            autonomousStepsFromFile.put(String.valueOf(loop + 1), autonomousStepsTemp.get(String.valueOf(loop)));
        }
        Log.d("insertSteps", "Re added all the previous steps");
        //increment the step counter as we inserted a new step
        //mValueSteps.add(loadStep, new LibraryStateTrack(false,false));
        this.numberLoadedSteps++;
        return autonomousStepsFromFile;
    }

    //Case 1 could be alliance park position at end of auton
    //Case 2 could be how many beacons to try and do during auton
    private HashMap<String,LibraryStateSegAutoRoverRuckus> loadSteps (String Filename, String case1, String case2) {

        boolean blnIfActive = false;
        boolean blnIfStart = false;


        try {
            File f = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS + "/Sequences"), Filename);
            BufferedReader reader = new BufferedReader(new FileReader(f));

            String csvLine;
            while((csvLine = reader.readLine()) != null) {
                //check if line is a comment and ignore it
                if (!((csvLine.substring(0, 2).equals("//")) || (csvLine.substring(0, 2).equals("\"")))) {
                    Log.d("readStepsFromFile", "Line " + csvLine);
                    String[] row = csvLine.split(",");
                    //check if the first value in CSV has a length larger than the expected value of a timout, a timeout would have no more than 2 characters
                    if (row[0].length() > 6) {
                        if (row[0].substring(0, 5).equalsIgnoreCase("START")) {
                            Log.d("readStepsFromFile", "Found START IF");
                            blnIfStart = true;
                            Log.d("readStepsFromFile", "Found In File " + row[0].substring(6));
                            Log.d("readStepsFromFile", " -- Setting   " + case1);
                            Log.d("readStepsFromFile", " -- Setting   " + case2);
                            if ((row[0].substring(6).equalsIgnoreCase(case1)) || (row[0].substring(6).equalsIgnoreCase(case2))) {
                                Log.d("readStepsFromFile", "Found ACTIVE IF");
                                blnIfActive = true;
                            }
                        } else if (row[0].substring(0, 3).equalsIgnoreCase("END")) {
                            Log.d("readStepsFromFile", "Found END IF");
                            blnIfActive = false;
                            blnIfStart = false;
                        }
                    } else {
                        if ((blnIfActive && blnIfStart) || (!blnIfActive && !blnIfStart)) {
                            //String[] row = csvLine.split(",");
                            //1,EYE,1,30,0.5,FALSE,FALSE,0,0,0,0,0,0
                            Log.d("readStepsFromFile", "CSV Value " + row[0].trim() + "," + row[1].trim() + "," + row[2].trim() + "," + row[3].trim() + "," + row[4].trim() + "," + row[5].trim() + "," + row[6].trim() + "," + row[7].trim() + "," + row[8].trim() + "," + row[9].trim() + "," + row[10].trim());

                            loadSteps(Integer.parseInt(row[positionTimeout].trim()),
                                    row[positionComnmand].trim().toUpperCase(),
                                    Double.parseDouble(row[positionDistance].trim()),
                                    Double.parseDouble(row[positionPower].trim()),
                                    Boolean.parseBoolean(row[positionParallel].trim()),
                                    Boolean.parseBoolean(row[positionLastPosition].trim()),
                                    Double.parseDouble(row[positionParm1].trim()),
                                    Double.parseDouble(row[positionParm2].trim()),
                                    Double.parseDouble(row[positionParm3].trim()),
                                    Double.parseDouble(row[positionParm4].trim()),
                                    Double.parseDouble(row[positionParm5].trim()),
                                    Double.parseDouble(row[positionParm6].trim()));
                        }
                    }
                }
            }
        } catch(IOException ex) {
            //throw new RuntimeException("Error in reading CSV file:" + ex);
            Log.d("readStepsFromFile", "Error in reading CSV file:" + ex);
        }
        return autonomousStepsFromFile;
    }

    public HashMap<String,LibraryStateSegAutoRoverRuckus> ReadStepFile(robotConfig robotconfig) {
        HashMap<String,LibraryStateSegAutoRoverRuckus> autonomousSteps = new HashMap<String,LibraryStateSegAutoRoverRuckus>();
        //load the sequence based on alliance colour and team
        switch (robotconfig.getAllianceColor()) {
            case "Red":
                switch (robotconfig.getAllianceStartPosition()) {
                    case "Left":
                        switch (robotconfig.getTeamNumber()) {
                            case "5291":
                                autonomousSteps = loadSteps("5291RedLeftRoverRuckus.csv", "none", "none");
                                break;
                            case "11230":
                                autonomousSteps = loadSteps("11230RedLeftRoverRuckus.csv", "none", "none");
                                break;
                            case "11231":
                                autonomousSteps = loadSteps("11231RedLeftRoverRuckus.csv", "none", "none");
                                break;
                        }
                        break;
                    case "Right":
                        switch (robotconfig.getTeamNumber()) {
                            case "5291":
                                autonomousSteps = loadSteps("5291RedRightRoverRuckus.csv", "none", "none");
                                break;
                            case "11230":
                                autonomousSteps = loadSteps("11230RedRightRoverRuckus.csv", "none", "none");
                                break;
                            case "11231":
                                autonomousSteps = loadSteps("11231RedRightRoverRuckus.csv", "none", "none");
                                break;
                        }
                        break;
                }
                break;
            case "Blue":
                switch (robotconfig.getAllianceStartPosition()) {
                    case "Left":
                        switch (robotconfig.getTeamNumber()) {
                            case "5291":
                                autonomousSteps = loadSteps("5291BlueLeftRoverRuckus.csv", "none", "none");
                                break;
                            case "11230":
                                autonomousSteps = loadSteps("11230BlueLeftRoverRuckus.csv", "none", "none");
                                break;
                            case "11231":
                                autonomousSteps = loadSteps("11231BlueLeftRoverRuckus.csv", "none", "none");
                                break;
                        }
                        break;
                    case "Right":
                        switch (robotconfig.getTeamNumber()) {
                            case "5291":
                                autonomousSteps = loadSteps("5291BlueRightRoverRuckus.csv", "none", "none");
                                break;
                            case "11230":
                                autonomousSteps = loadSteps("11230BlueRightRoverRuckus.csv", "none", "none");
                                break;
                            case "11231":
                                autonomousSteps = loadSteps("11231BlueRightRoverRuckus.csv", "none", "none");
                                break;
                        }
                        break;
                }
                break;
            case "Test":
                switch (robotconfig.getTeamNumber()) {
                    case "5291":
                        autonomousSteps = loadSteps("5291TestRoverRuckus.csv", "none", "none");
                        break;
                    case "11230":
                        autonomousSteps = loadSteps("11230TestRoverRuckus.csv", "none", "none");
                        break;
                    case "11231":
                        autonomousSteps = loadSteps("11231TestRoverRuckus.csv", "none", "none");
                        break;
                }

                break;
        }
        autonomousStepsFromFile = autonomousSteps;
        return autonomousStepsFromFile;
    }


}
