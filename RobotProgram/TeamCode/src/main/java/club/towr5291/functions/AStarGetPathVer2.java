package club.towr5291.functions;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.HashMap;

import club.towr5291.astarpathfinder.A0Star;
import club.towr5291.astarpathfinder.AStarValue;
import club.towr5291.astarpathfinder.sixValues;

/**
 * Created by ianhaden on 5/09/16.
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
 *
 * //creates a path from (startX,startY) to (endX,endY)
 // param startFieldX = startX : starting x position
 // param startFieldY = startY : starting y position
 // param startFieldZ = startZ : starting Direction
 // param targetFieldX = endX : ending x position
 // param targetFieldY = endY : ending y position
 // param targetFieldZ = endZ : ending direction

 //NOTE : Inputs are in terms of room positions.
 //       All other positions will be in terms of grid

 */

public class AStarGetPathVer2 {
    private static final String TAG = "AStarGetPathVer2";
//    private AStarValue AStarValues = new AStarValue();
    private HashMap<String,AStarValue> AStarValueMap = new HashMap<String, AStarValue>();
    private HashMap<String,AStarValue> AStarClosedMap = new HashMap<String,AStarValue>();
    private HashMap<String,AStarValue> AStarOpenMap = new HashMap<String,AStarValue>();
    private A0Star a0Star = new A0Star();
    private sixValues[] pathValues = new sixValues[500];

    private int BlueRed = 0;  //0 is not defined, 1 is blue, 2 is red

    //set up the variables for the logger
    private ElapsedTime runtime = new ElapsedTime();
    private FileLogger fileLogger;
    private int debug = 2;

    public AStarGetPathVer2()
    {
        for ( int i = 0; i < pathValues.length; i++)
        {
            pathValues[i] = new sixValues();
        }
    }

    private int getKey(int xPos, int yPos, int width, int length)
    {
        int value;
        value = yPos*length + xPos;
        return value;
    }

    private int getXPos(int key, int width, int length)
    {
        int value;
        value = (int)key % width;  //modulo
        return value;
    }

    private int getYPos(int key, int width, int length)
    {
        int value;
        value = (int)key / width;  //integer division
        return value;
    }

    private void addToClosed (int xPos, int yPos)
    {
        final String TAG = "AStarGetPathVer2 addToClosed";
        int key;
        //add point to the closed list
        key = getKey(xPos,yPos, a0Star.fieldWidth, a0Star.fieldLength);
        AStarClosedMap.put(String.valueOf(key), new AStarValue(key, 0, 0, 0, 0, xPos, yPos, 0));
    }

    private void removeFromClosed (int xPos, int yPos)
    {
        final String TAG = "AStarGetPathVer2 removeFromClosed";
        int key;
        key = getKey(xPos,yPos, a0Star.fieldWidth, a0Star.fieldLength);
        //remmove point from the closed list
        //check that the map contains the key, if it is there then remove it
        if (AStarClosedMap.containsKey(String.valueOf(key))) {
            AStarClosedMap.remove(String.valueOf(key));
        }
    }

    private void addToOpen (int xPos, int yPos, int zPos, double fValue)   //this will overwrite items with same key
    {
        final String TAG = "AStarGetPathVer2 addToOpen";
        int key;
        //add point to the open list
        key = getKey(xPos,yPos, a0Star.fieldWidth, a0Star.fieldLength);
        AStarOpenMap.put(String.valueOf(key), new AStarValue(key, fValue, 0, 0, 0, xPos, yPos, zPos));
    }

    private void removeFromOpen (int xPos, int yPos)
    {
        final String TAG = "AStarGetPathVer2 removeFromOpen";
        int key;
        //remmove point from the open list
        key = getKey(xPos,yPos, a0Star.fieldWidth, a0Star.fieldLength);
        //check that the map contains the key, if it is there then remove it
        if (AStarOpenMap.containsKey(String.valueOf(key))) {
            AStarOpenMap.remove(String.valueOf(key));
        }
    }

    private void updateValuesMap (int x, int y, int z, double f, double g, double h, int parent)
    {
        final String TAG = "AStarGetPathVer2 updateValuesMap";
        int nodeKey;
        AStarValue temp = new AStarValue();

        nodeKey = getKey(x, y, a0Star.fieldWidth, a0Star.fieldLength);

        temp.xvalue = x;
        temp.yvalue = y;
        temp.zvalue = z;
        temp.FValue = f;
        temp.GValue = g;
        temp.HValue = h;
        temp.Parent = parent;
        temp.ID = nodeKey;

        if (debug >= 2) {
            if (nodeKey == 0) {
                Log.d(TAG, "NOOOOOOOOOOOOOOOOOOOOOOOOOO BAD");
            }
            fileLogger.writeEvent(TAG, "Updated Values " + nodeKey);
            Log.d(TAG, "Updated Values " + nodeKey + " x " + temp.xvalue + " y " + temp.yvalue + " ID " + temp.ID);
        }
        AStarValueMap.put(String.valueOf(nodeKey), temp);
    }

    private int getTurnCost (int xPos, int yPos, int currentX, int currentY, int currentDir) {
        final String TAG = "AStarGetPathVer2 getTurnCost";
        int costPenalty = 0;
        int cost = 1000;
        // /check direction change
        switch (currentDir)
        {
            case 0:
                if ((xPos == currentX) && (yPos == (currentY - 1)))
                {
                    costPenalty = 0;  //no turning
                }
                else
                {
                    costPenalty = cost;  //cost for turning
                }
                break;
            case 45:
                if ((xPos == (currentX + 1)) && (yPos == (currentY - 1)))
                {
                    costPenalty = 0;  //no turning
                }
                else
                {
                    costPenalty = cost;  //cost for turning
                }
                break;
            case 90:
                if ((xPos == (currentX + 1)) && (yPos == (currentY)))
                {
                    costPenalty = 0;  //no turning
                }
                else
                {
                    costPenalty = cost;  //cost for turning
                }
                break;
            case 135:
                if ((xPos == (currentX + 1)) && (yPos == (currentY + 1)))
                {
                    costPenalty = 0;  //no turning
                }
                else
                {
                    costPenalty = cost;  //cost for turning
                }
                break;
            case 180:
                if ((xPos == (currentX)) && (yPos == (currentY + 1)))
                {
                    costPenalty = 0;  //no turning
                }
                else
                {
                    costPenalty = cost;  //cost for turning
                }
                break;
            case 225:
                if ((xPos == (currentX + 1)) && (yPos == (currentY - 1)))
                {
                    costPenalty = 0;  //no turning
                }
                else
                {
                    costPenalty = cost;  //cost for turning
                }
                break;
            case 270:
                if ((xPos == (currentX - 1)) && (yPos == (currentY)))
                {
                    costPenalty = 0;  //no turning
                }
                else
                {
                    costPenalty = cost;  //cost for turning
                }
                break;
            case 315:
                if ((xPos == (currentX - 1)) && (yPos == (currentY - 1)))
                {
                    costPenalty = 0;  //no turning
                }
                else
                {
                    costPenalty = cost;  //cost for turning
                }
                break;
        }
        return costPenalty;
    }

    private int getNewDirection(int x, int y, int currentX, int currentY)
    {
        final String TAG = "AStarGetPathVer2 getNewDirection";
        int newDir = 0;

        if ((x == currentX) && (y == (currentY - 1))) //0 degrees
        {
            newDir = 0;
        } else if ((x == (currentX + 1)) && (y == (currentY - 1)))   //45 degrees
        {
            newDir = 45;
        } else if ((x == (currentX + 1)) && (y == (currentY)))   // 90 degrees
        {
            newDir = 90;
        } else if ((x == (currentX - 1)) && (y == (currentY + 1)))   // 135 degrees
        {
            newDir = 135;
        } else if ((x == (currentX)) && (y == (currentY + 1)))   // 180 degrees
        {
            newDir = 180;
        } else if ((x == (currentX - 1)) && (y == (currentY + 1)))   // 225 degrees
        {
            newDir = 225;
        } else if ((x == (currentX - 1)) && (y == (currentY)))  //  270 degrees
        {
            newDir = 270;
        } else if ((x == (currentX - 1)) && (y == (currentY - 1)))  //  315 degrees
        {
            newDir = 315;
        }

        if (debug >= 1)
        {
            fileLogger.writeEvent(TAG, "x " + x + " y " + y + " currentX " + currentX + " currentY " + currentY + " newDir " + newDir);
            Log.d(TAG, "x " + x + " y " + " currentX " + currentX + " currentY" + currentY + " newDir " + newDir);
        }

        return newDir;
    }

    private boolean getwalkable (int x, int y) {
        final String TAG = "AStarGetPathVer2 getwalkable";
        boolean canWalk;

        if (BlueRed == 2) {
            canWalk = a0Star.walkableRed[y][x];
        } else if (BlueRed == 1) {
            canWalk = a0Star.walkableBlue[y][x];
        } else {
            canWalk = a0Star.walkable[y][x];
        }

        return canWalk;
    }

    private sixValues ProcessCurrentNode (int currentX, int currentY, int currentDir, int endX, int endY, int endDir) {
        final String TAG = "AStarGetPathVer2 ProcessCurrentNode";
        boolean closed = false;
        boolean diagonal = false;
        boolean walkable = false;
        int searching = 1;
        int found = 0;
        double distFromCurrentToIJ = 0;
        double distFromStartToCurrent = 0;
        double tempF, tempG, tempH;
        double lowestF = -1;
        int lowestFKey = 9999;
        int costPenalty;
        int newDir;
        int pathIndex = 0;
//        HashMap<String,AStarValue> AStarCurrentProcess = new HashMap<String, AStarValue>();
        sixValues returnValue = new sixValues(0,0,0,0,0,0);
        AStarValue AStarValueCurrentXYNode = new AStarValue();      //The main node - the one in the center
        AStarValue AStarValueCurrentNode = new AStarValue();        //the nodes around the main node - 8 of them
        AStarValue AStarValueCurrentFNode = new AStarValue();        //the nodes around the main node - 8 of them
        AStarValue AStarValueGetLowestF = new AStarValue();


        if (debug >= 2)
        {
            fileLogger.writeEvent(TAG, "Starting AStar for point - Added to closed list point (" + currentX + "," + currentY + ") Key " +  getKey(currentX, currentY, a0Star.fieldWidth, a0Star.fieldLength));
            Log.d(TAG, "Starting AStar for point - Added to closed list point(" + currentX + "," + currentY + ") Key " + getKey(currentX, currentY, a0Star.fieldWidth, a0Star.fieldLength));
        }

        //analyze adjacent blocks/grid locations
        //key exists so get the values
        int nodeKey = getKey(currentX, currentY, a0Star.fieldWidth, a0Star.fieldLength);
        if (AStarValueMap.containsKey(String.valueOf(nodeKey))){
            AStarValueCurrentXYNode = AStarValueMap.get(String.valueOf(nodeKey));
            if (debug >= 3)
            {
                fileLogger.writeEvent("ProcessCurrentNode", "Getting Key - point (" + currentX + "," + currentY + ") Key " + nodeKey);
                Log.d(TAG, "Getting Key - point (" + currentX + "," + currentY +") Key " + nodeKey);
            }
        }
        else
        {
            AStarValueCurrentXYNode.xvalue = currentX;
            AStarValueCurrentXYNode.yvalue = currentY;
            AStarValueCurrentXYNode.zvalue = currentDir;
            AStarValueCurrentXYNode.FValue = 0;
            AStarValueCurrentXYNode.GValue = 0;
            AStarValueCurrentXYNode.HValue = 0;
            AStarValueCurrentXYNode.ID = nodeKey;
            if (debug >= 2)
            {
                fileLogger.writeEvent(TAG, "Key Doesn't Exist - point (" + currentX + "," + currentY + ") Key " + nodeKey);
                Log.d(TAG, "Key Doesn't Exist - point (" + currentX + "," + currentY + ") Key " + nodeKey);
            }
        }

        //add point to be process to the closed list
        addToClosed(currentX, currentY);
        removeFromOpen(currentX, currentY);

        //now check all nodes around the current one, ignore those on the closed list and that are not walkable
        for (int y = Math.max(0, (currentY - 1)); y <= Math.min(a0Star.fieldLength - 1, currentY + 1); y++ )
        {
            for (int x = Math.max(0, (currentX - 1)); x <= Math.min(a0Star.fieldWidth - 1, currentX + 1); x++ )
            {
                //check to see if its on the closed list, if its closed then do nothing, move to next node
                closed = AStarClosedMap.containsKey(String.valueOf(getKey(x, y, a0Star.fieldWidth, a0Star.fieldLength)));

                //check to see if the current node is walkable, if its not then do nothing, move to next node
                walkable = getwalkable(x,y);

                if (!walkable) {
                    //need to remove the value from the OpenList
                    removeFromOpen(x,y);
                }

                if (debug >= 3)
                {
                    fileLogger.writeEvent(TAG, "checking if on closed list and walkable - point (" + x + "," + y + ") Key " + getKey(x, y, a0Star.fieldWidth, a0Star.fieldLength) + " closed = " + closed + " Walkable= " + walkable);
                    Log.d(TAG, "checking if on closed list and walkable - point (" + x + "," + y + ") Key " + getKey(x, y, a0Star.fieldWidth, a0Star.fieldLength) + " closed = " + closed + " Walkable= " + walkable);
                }

                if (!closed && walkable)
                {
                    if (AStarValueMap.containsKey(String.valueOf(getKey(x, y, a0Star.fieldWidth, a0Star.fieldLength)))) {
                        AStarValueCurrentNode = AStarValueMap.get(String.valueOf(getKey(x, y, a0Star.fieldWidth, a0Star.fieldLength)));
                    } else {
                        AStarValueCurrentNode.ID = 0;
                        AStarValueCurrentNode.GValue = 0;
                        AStarValueCurrentNode.HValue = 0;
                    }

                    //get the new direction the robot is facing after the move if it needs to move
                    newDir = getNewDirection(x, y, currentX, currentY);

                    //determine additional cost for turning robot
                    costPenalty = getTurnCost(x, y, currentX, currentY, currentDir);
                    //costPenalty = 0;

                    //no room for turning if within 9 inches of the wall, robot will hit the wall so add even more cost
                    //if (((currentY > 124) && ((currentDir == 0) || (currentDir == 45) || (currentDir == 315))) || ((currentX > 124) && ((currentDir == 270) || (currentDir == 225) || (currentDir == 315)))) {
                    //    costPenalty += 1000;
                    //} else {
                    //    costPenalty += 0;
                    //}
                    //costPenalty = 0;

                    if (debug >= 3) {
                        fileLogger.writeEvent(TAG, "point (" + x + "," + y + ") costPenalty " + costPenalty + " newDir " + newDir + " Is Diagonal " + diagonal + " Key " + getKey(x, y, a0Star.fieldWidth, a0Star.fieldLength));
                        Log.d(TAG, "point (" + x + "," + y + ") costPenalty " + costPenalty + " newDir " + newDir + " Is Diagonal " + diagonal + " Key " + getKey(x, y, a0Star.fieldWidth, a0Star.fieldLength));
                    }

                    //checking the diagonal nodes
                    diagonal = ((x + y) % 2 == (currentX + currentY) % 2);
                    if (diagonal) {
                        distFromCurrentToIJ = 1410 + costPenalty;                                                           //G Value
                    } else {
                        distFromCurrentToIJ = 1000 + costPenalty;                                                           //G Value
                    }

                    //calculated G,H,and F
                    tempG = AStarValueCurrentXYNode.GValue + distFromCurrentToIJ;                                               // distance from starttocurrent + value just calculated
                    tempH = ((Math.abs(x - endX) * 1000 + Math.abs(y - endY) * 1000));                                      //insert heuristic of choice (we use manhattan)
                    tempF = tempG + tempH;
                    int parent = getKey(currentX, currentY, a0Star.fieldWidth, a0Star.fieldLength);

                    //see if key is in G Map, means already processed
                    if (AStarValueCurrentNode.ID != 0 )
                    {
                        if (debug >= 3)
                        {
                            fileLogger.writeEvent(TAG, "IF (AStarValueCurrentIJ.ID != 0)   ,  AStarValueCurrentIJ.GValue = " + AStarValueCurrentNode.GValue);
                            Log.d(TAG, "IF (AStarValueCurrentIJ.ID != 0)   ,  AStarValueCurrentIJ.GValue = " + AStarValueCurrentNode.GValue);
                        }
                        //update if lower value
                        if ((tempG < AStarValueCurrentNode.GValue) || (AStarValueCurrentNode.GValue == 0))
                        {
                            if (debug >= 2)
                            {
                                fileLogger.writeEvent(TAG, "Updating point (" + x + "," + y + ") Key " + getKey(x, y, a0Star.fieldWidth, a0Star.fieldLength) + "    G:" + tempG + "     H:" + tempH + "     F:" + tempF + " Parent " + parent);
                                Log.d(TAG, "Updating point (" + x + "," + y + ") Key " + getKey(x, y, a0Star.fieldWidth, a0Star.fieldLength) + "    G:" + tempG + "     H:" + tempH + "     F:" + tempF + " Parent " + parent);
                            }
                            //need to update GValue and parent only
                            //updateValuesMap(x, y, newDir, tempF, tempG, tempH, parent);
                            //int nodekey = getKey(x, y, a0Star.fieldWidth, a0Star.fieldLength);
                            //AStarValueMap.put(String.valueOf(nodekey), new AStarValue(nodekey, tempF, tempG, tempH, parent, x, y, newDir));

                        } else {  //previous values were better so keep them
                            tempF = AStarValueCurrentNode.FValue;
                            tempG = AStarValueCurrentNode.GValue;
                            parent = AStarValueCurrentNode.Parent;
                            newDir = AStarValueCurrentNode.zvalue;
                        }
                    }
                    else
                    {
                        if (debug >= 2)
                        {
                            fileLogger.writeEvent(TAG, "Adding point (" + x + "," + y + ") Key " + getKey(x, y, a0Star.fieldWidth, a0Star.fieldLength) + "    G:" + tempG + "     H:" + tempH + "     F:" + tempF + " Parent " + parent);
                            Log.d(TAG, "Adding point (" + x + "," + y + ") Key " + getKey(x, y, a0Star.fieldWidth, a0Star.fieldLength) + "    G:" + tempG + "     H:" + tempH + "     F:" + tempF + " Parent " + parent);
                        }
                        //add the new point to the map
                        //updateValuesMap(x, y, newDir, tempF, tempG, tempH, parent);
                        //int nodekey = getKey(x, y, a0Star.fieldWidth, a0Star.fieldLength);
                        //AStarValueMap.put(String.valueOf(nodekey), new AStarValue(nodekey, tempF, tempG, tempH, parent, x, y, newDir));
                    }
                    int nodekey = getKey(x, y, a0Star.fieldWidth, a0Star.fieldLength);
                    AStarValueMap.put(String.valueOf(nodekey), new AStarValue(nodekey, tempF, tempG, tempH, parent, x, y, newDir));
                    addToOpen(x, y, newDir, tempF);
                }
            }
        }

        lowestF = 99999999;

        //next node to check is the one with the lowest F and also on the OPEN list
        for (String nodeKeys: AStarOpenMap.keySet()) {
            AStarValueCurrentFNode = AStarOpenMap.get(nodeKeys);
            //load all the values and find the lowest fValue
            /*if (AStarValueMap.containsKey(nodeKeys)) {
                if (debug >= 3) {
                    fileLogger.writeEvent(TAG, "Key exists " + nodeKeys);
                    Log.d(TAG, "Key exists " + nodeKeys);
                }
            } else {
                if (debug >= 3) {
                    fileLogger.writeEvent(TAG, "No Such Key " + nodeKeys);
                    Log.d(TAG, "No Such Key " + nodeKeys);
                }
            }*/

            //AStarValueCurrentFNode = AStarValueMap.get(nodeKeys);
            if (AStarValueCurrentFNode.FValue < lowestF)
            {
                currentX = AStarValueCurrentFNode.xvalue;
                currentY = AStarValueCurrentFNode.yvalue;
                currentDir = AStarValueCurrentFNode.zvalue;
                lowestF = AStarValueCurrentFNode.FValue;
                //lowestFKey = AStarValueCurrentFNode.ID;
                lowestFKey =  getKey(currentX,currentY, a0Star.fieldWidth, a0Star.fieldLength);
                if (debug >= 2) {
                    fileLogger.writeEvent(TAG, "Found LowerF " + lowestF + " Key " + lowestFKey + " sourcekey " + nodeKeys + " x,y (" + AStarValueCurrentFNode.xvalue + "," + AStarValueCurrentFNode.yvalue + ")");
                    Log.d(TAG, "Found LowerF " + lowestF + " Key " + lowestFKey + " sourcekey " + nodeKeys + " x,y (" + AStarValueCurrentFNode.xvalue + "," + AStarValueCurrentFNode.yvalue + ")");
                }
            }
            if (debug >= 3) {
                fileLogger.writeEvent(TAG, "lowestF " + lowestF + " Key " + lowestFKey + " Next Key " + nodeKeys);
                Log.d(TAG, "lowestF " + lowestF + " Key " + lowestFKey + " Next Key " + nodeKeys);
            }
            searching = 1;
            found = 0;
        }

        if (lowestF != 99999999)
        {
            if (debug >= 3)
            {
                fileLogger.writeEvent(TAG, "Lowest FKey " + lowestFKey);
                Log.d(TAG, "Lowest FKey " + lowestFKey);
            }

            //process the lowest value next
            //AStarValueGetLowestF = AStarValueMap.get(String.valueOf(lowestFKey));
            //currentX = AStarValueGetLowestF.xvalue;
            //currentY = AStarValueGetLowestF.yvalue;
            //currentDir = AStarValueGetLowestF.zvalue;

            searching = 1;
            found = 0;
        }
        else
        {
            searching = 0;
            found = 0;
            if (debug >= 1)
            {
                fileLogger.writeEvent(TAG, "no Path Found = No More Nodes Left");
                Log.d(TAG, "no Path Found = No More Nodes Left");
            }
        }
        //check whether we're at the end
        if ((currentX == endX) && (currentY == endY))
        {
            searching = 0;
            found = 1;
            if (debug >= 1)
            {
                fileLogger.writeEvent(TAG, "You found me I'm the final block");
                Log.d(TAG, "You found me I'm the final block");
            }
        }
        returnValue.val1 = searching;
        returnValue.val2 = found;
        returnValue.val3 = currentX;
        returnValue.val4 = currentY;
        returnValue.val5 = currentDir;
        if (debug >= 3)
        {
            fileLogger.writeEvent(TAG, "Returning 1- " + searching + " 2- " + found + " 3- " + currentX + " 4- " + currentY + " 5- " + returnValue.val5 + "point (" + currentX + "," + currentY + ")");
            Log.d(TAG, "Returning 1- " + searching + " 2- " + found + " 3- " + currentX + " 4- " + currentY + " 5- " + returnValue.val5 + "point (" + currentX + "," + currentY + ")");
        }
        return returnValue;
    }

    public sixValues[] pathReOrder (int startX, int startY, int startZ, int endX, int endY) {
        final String TAG = "AStarGetPathVer2 pathReOrder";

        sixValues[] pathValuesReversed = new sixValues[1500];
        AStarValue AStarValueCurrentNode = new AStarValue();
        int stepCount = 1;
        int endKey;
        int startKey;
        boolean generatingPath = true;

        for ( int i = 0; i < pathValuesReversed.length; i++)
        {
            pathValuesReversed[i] = new sixValues();
        }

        //need to get path from this mess....work back through the parents
        endKey = getKey(endX, endY, a0Star.fieldWidth, a0Star.fieldLength);
        startKey = getKey(startX, startY, a0Star.fieldWidth, a0Star.fieldLength);

        int pathKey = endKey;

        if (debug >= 2)
        {
            fileLogger.writeEvent(TAG, "pathKey " + pathKey + " startKey " + startKey);
            Log.d(TAG, "pathKey " + pathKey + " startKey " + startKey);
        }

        while (generatingPath) {

            AStarValueCurrentNode = AStarValueMap.get(String.valueOf(pathKey));
            pathValuesReversed[stepCount].val1 = (double)stepCount;
            pathValuesReversed[stepCount].val2 = AStarValueCurrentNode.xvalue;
            pathValuesReversed[stepCount].val3 = AStarValueCurrentNode.yvalue;
            pathValuesReversed[stepCount].val4 = AStarValueCurrentNode.zvalue;
            pathKey = AStarValueCurrentNode.Parent;
            stepCount++;
            if (AStarValueCurrentNode.Parent == startKey) {
                //found start point add it to the list
                pathValuesReversed[stepCount].val1 = (double)stepCount;
                pathValuesReversed[stepCount].val2 = startX;
                pathValuesReversed[stepCount].val3 = startY;
                pathValuesReversed[stepCount].val4 = startZ;
                generatingPath = false;
            }
        }

        for (int loop = 0; loop < stepCount; loop++) {
            pathValues[loop].val1 = loop + 1;
            pathValues[loop].val2 = pathValuesReversed[stepCount - loop].val2;
            pathValues[loop].val3 = pathValuesReversed[stepCount - loop].val3;
            pathValues[loop].val4 = pathValuesReversed[stepCount - loop].val4;
            if (debug >= 2)
            {
                fileLogger.writeEvent(TAG, "The Path - " + pathValues[loop].val1 + " " + pathValues[loop].val2 + " " + pathValues[loop].val3 + " " + pathValues[loop].val4 );
                Log.d(TAG, "The Path - " + pathValues[loop].val1 + " " + pathValues[loop].val2 + " " + pathValues[loop].val3 + " " + pathValues[loop].val4 );
            }
        }

        return pathValues;
    }

    public sixValues[] findPathAStar (int startX, int startY, int startZ, int endX, int endY, int endDir)
    {
        final String TAG = "AStarGetPathVer2 findPathAStar";
        boolean searching = true;

        if (debug >= 1)
        {
            fileLogger = new FileLogger(runtime);
            fileLogger.open();
            fileLogger.write("Time,SysMS,Thread,Event,Desc");
            fileLogger.writeEvent(TAG, "Log Started");
        }

        if (startX < startY)
        {
            BlueRed = 2;  //RED
        }
        else
        {
            BlueRed = 1;  //BLUE
        }

        //add startpoint to the ValuesMap
        //updateValuesMap(startX, startY, startZ, 0, 0, 0, 0);
        int nodekey = getKey(startX, startY, a0Star.fieldWidth, a0Star.fieldLength);
        AStarValueMap.put(String.valueOf(nodekey), new AStarValue(nodekey, 0, 0, 0, 0, startX, startY, startZ));

        //removeFromOpen(startX, startY);
        //addToClosed(startX, startY);

        if (debug >= 3)
        {
            for (String nodeKey: AStarValueMap.keySet())
            {
                fileLogger.writeEvent(TAG, "Keys " + nodeKey + ": x:" + AStarValueMap.get(nodeKey).xvalue + " y:" + AStarValueMap.get(nodeKey).yvalue);
                Log.d(TAG, "Keys " + nodeKey + ": x:" + AStarValueMap.get(nodeKey).xvalue + " y:" + AStarValueMap.get(nodeKey).yvalue);
            }
        }

        sixValues currentResult = new sixValues(1, 0, startX, startY, startZ, 0);

        //process the map and find the path
        while (searching)
        {
            currentResult = (ProcessCurrentNode((int)currentResult.val3, (int)currentResult.val4, (int)currentResult.val5 , endX, endY, endDir));
            searching = (currentResult.val1 == 1);
        }

        boolean found = (currentResult.val2 == 1);

        if (found)
        {
            //need to find the final path
            pathValues = pathReOrder(startX, startY, startZ, endX, endY);

//            for ( int i = 0; i < pathValues.length; i++)
//            {
//                //fileLogger.writeEvent("init()","Path " + pathValues[i].val1 + " " + pathValues[i].val2 + " " + pathValues[i].val3 );
//                Log.d(TAG, "Path Step:= " + pathValues[i].val1 + " x:- " + pathValues[i].val2 + " y:= " + pathValues[i].val3  + " Dir:= " + pathValues[i].val4);
//                if ((pathValues[i].val2 == endX) && (pathValues[i].val3 == endY))
//                {
//                    break;
//                }
//            }
        }
        if (debug >= 1)
        {
            if (fileLogger != null)
            {
                fileLogger.writeEvent(TAG, "Stopped");
                fileLogger.close();
                fileLogger = null;
            }
        }
        return pathValues;
    }
}
