package club.towr5291.functions;

import android.os.Environment;
import android.util.Log;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.HashMap;

import club.towr5291.libraries.LibraryStateSegAuto;

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

public class ReadStepFile {

    private HashMap<String,LibraryStateSegAuto> autonomousStepsFromFile;
    private int numberLoadedSteps = 1;

    public int getNumberLoadedSteps() {
        return numberLoadedSteps;
    }

    public void setNumberLoadedSteps(int numberLoadedSteps) {
        this.numberLoadedSteps = numberLoadedSteps;
    }

    private void initReadStepsFile() {
        this.setNumberLoadedSteps(1);
        autonomousStepsFromFile = new HashMap<>();
    }

    private void loadSteps(int timeOut, String command, boolean parallel, boolean lastPos, double parm1, double parm2, double parm3, double parm4, double parm5, double parm6, double power)
    {
        autonomousStepsFromFile.put(String.valueOf(this.getNumberLoadedSteps()), new LibraryStateSegAuto (this.getNumberLoadedSteps(), timeOut, command, parallel, lastPos, parm1, parm2, parm3, parm4, parm5, parm6, power));
        //mValueSteps.add(loadStep, new LibraryStateTrack(false,false));
        this.numberLoadedSteps++;
    }

    public void insertSteps(int timeOut, String command, boolean parallel, boolean lastPos, double parm1, double parm2, double parm3, double parm4, double parm5, double parm6, double power, int insertlocation)
    {
        Log.d("insertSteps", "insert location " + insertlocation + " timout " + timeOut + " command " + command + " parallel " + parallel + " lastPos " + lastPos + " parm1 " + parm1 + " parm2 " + parm2 + " parm3 " + parm3 + " parm4 " + parm4 + " parm5 " + parm5 + " parm6 " + parm6 + " power " + power);
        HashMap<String,LibraryStateSegAuto> autonomousStepsTemp = new HashMap<String,LibraryStateSegAuto>();
        LibraryStateSegAuto processingStepsTemp;

        //move all the steps from current step to a temp location
        for (int loop = insertlocation; loop < this.getNumberLoadedSteps(); loop++)
        {
            processingStepsTemp = autonomousStepsFromFile.get(String.valueOf(loop));
            Log.d("insertSteps", "Reading all the next steps " + loop + " timout " + processingStepsTemp.getmRobotTimeOut() + " command " + processingStepsTemp.getmRobotCommand());
            autonomousStepsTemp.put(String.valueOf(loop), autonomousStepsFromFile.get(String.valueOf(loop)));
        }
        Log.d("insertSteps", "All steps loaded to a temp hasmap");

        //insert the step we want
        autonomousStepsFromFile.put(String.valueOf(insertlocation), new LibraryStateSegAuto (this.getNumberLoadedSteps(), timeOut, command, parallel, lastPos, parm1, parm2, parm3, parm4, parm5, parm6, power));
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
    }

    public HashMap<String,LibraryStateSegAuto> ReadStepFileRelicRecovery(String Filename, String case1, String case2) {

        boolean blnIfActive = false;
        boolean blnIfStart = false;

        //reset the steps, and init the hashmap
        this.initReadStepsFile();

        try {
            File f = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS + "/Sequences"), Filename);
            BufferedReader reader = new BufferedReader(new FileReader(f));

            String csvLine;
            while((csvLine = reader.readLine()) != null) {
                //check if line is a comment and ignore it
                if (!((csvLine.substring(0, 2).equals("//")) || (csvLine.substring(0, 2).equals("\"")))) {
                    String[] row = csvLine.split(",");
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
                        } else if (row[0].substring(0, 3).equals("END")) {
                            Log.d("readStepsFromFile", "Found END IF");
                            blnIfActive = false;
                            blnIfStart = false;
                        }
                    } else {
                        if ((blnIfActive && blnIfStart) || (!blnIfActive && !blnIfStart)) {
                            //String[] row = csvLine.split(",");

                            Log.d("readStepsFromFile", "CSV Value " + row[0].trim() + "," + row[1].trim() + "," + row[2].trim() + "," + row[3].trim() + "," + row[4].trim() + "," + row[5].trim() + "," + row[6].trim() + "," + row[7].trim() + "," + row[8].trim() + "," + row[9].trim() + "," + row[10].trim());
                            loadSteps(Integer.parseInt(row[0].trim()), row[1].trim(), Boolean.parseBoolean(row[2].trim()), Boolean.parseBoolean(row[3].trim()), Double.parseDouble(row[4].trim()), Double.parseDouble(row[5].trim()), Double.parseDouble(row[6].trim()), Double.parseDouble(row[7].trim()), Double.parseDouble(row[8].trim()), Double.parseDouble(row[9].trim()), Double.parseDouble(row[10].trim()));
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

    public HashMap<String,LibraryStateSegAuto> ReadStepFileVelocityVortex(String Filename, String allianceParkPosition, String numBeacons) {

        boolean blnIfActive = false;
        boolean blnIfStart = false;

        //reset the steps, and init the hashmap
        this.initReadStepsFile();

        try {
            File f = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS + "/Sequences"), Filename);
            BufferedReader reader = new BufferedReader(new FileReader(f));

            String csvLine;
            while((csvLine = reader.readLine()) != null) {
                //check if line is a comment and ignore it
                if (!((csvLine.substring(0, 2).equals("//")) || (csvLine.substring(0, 2).equals("\"")))) {
                    String[] row = csvLine.split(",");
                    if (row[0].length() > 6) {
                        if (row[0].substring(0, 5).equalsIgnoreCase("START")) {
                            Log.d("readStepsFromFile", "Found START IF");
                            blnIfStart = true;
                            Log.d("readStepsFromFile", "Found In File " + row[0].substring(6));
                            Log.d("readStepsFromFile", " -- Setting   " + allianceParkPosition);
                            Log.d("readStepsFromFile", " -- Setting   " + numBeacons);
                            if ((row[0].substring(6).equalsIgnoreCase(allianceParkPosition)) || (row[0].substring(6).equalsIgnoreCase(numBeacons))) {
                                Log.d("readStepsFromFile", "Found ACTIVE IF");
                                blnIfActive = true;
                            }
                        } else if (row[0].substring(0, 3).equals("END")) {
                            Log.d("readStepsFromFile", "Found END IF");
                            blnIfActive = false;
                            blnIfStart = false;
                        }
                    } else {
                        if ((blnIfActive && blnIfStart) || (!blnIfActive && !blnIfStart)) {
                            //String[] row = csvLine.split(",");

                            Log.d("readStepsFromFile", "CSV Value " + row[0].trim() + "," + row[1].trim() + "," + row[2].trim() + "," + row[3].trim() + "," + row[4].trim() + "," + row[5].trim() + "," + row[6].trim() + "," + row[7].trim() + "," + row[8].trim() + "," + row[9].trim() + "," + row[10].trim());
                            loadSteps(Integer.parseInt(row[0].trim()), row[1].trim(), Boolean.parseBoolean(row[2].trim()), Boolean.parseBoolean(row[3].trim()), Double.parseDouble(row[4].trim()), Double.parseDouble(row[5].trim()), Double.parseDouble(row[6].trim()), Double.parseDouble(row[7].trim()), Double.parseDouble(row[8].trim()), Double.parseDouble(row[9].trim()), Double.parseDouble(row[10].trim()));
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
}
