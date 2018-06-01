package club.towr5291.astarpathfinder;

/**
 * Created by ianhaden on 7/09/16.

 TOWR 5291
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

public class A0Star {

    public static final int FIELDWIDTH = 144;
    public static final int FIELDLENGTH = 144;
    public int blockSize = 1;
    public boolean[][] walkable = new boolean[FIELDWIDTH][FIELDLENGTH];
    public boolean[][] walkableBlue = new boolean[FIELDWIDTH][FIELDLENGTH];
    public boolean[][] walkableRed = new boolean[FIELDWIDTH][FIELDLENGTH];
    public int fieldWidth;
    public int fieldLength;


    public A0Star (){
        fieldWidth = (int)(FIELDWIDTH / blockSize);
        fieldLength = (int)(FIELDLENGTH / blockSize);

        for (int y = 0; y < fieldLength; y++) {
            for (int x = 0; x < fieldWidth; x++) {
                walkable[y][x] = true;  //used for test program, not used for anything else
                if (y >= x) {
                    //map half the field as walkable
                    walkableRed[y][x] = true;
                    walkableBlue[y][x] = false;
                    //mark corner ramps as unwalkable
                    if ( y < (x + 96)) {
                        walkableRed[y][x] = true;
                    } else {
                        walkableRed[y][x] = false;
                    }

                    //mark center as unwalkable 1foot square
                    if (((x >= 66 ) && (x <= 78)) && ((y >= 66 ) && (y <= 78))) {
                        walkableRed[y][x] = false;
                        walkableBlue[y][x] = false;
                    }
                } else {
                    //map half the field as walkable
                    walkableRed[y][x] = false;
                    walkableBlue[y][x] = true;
                    //mark corner ramps as unwalkable
                    if ( x < (y + 96)) {
                        walkableBlue[y][x] = true;
                    } else {
                        walkableBlue[y][x] = false;
                    }
                    //mark center as unwalkable 1foot square
                    if (((x >= 66 ) && (x <= 78)) && ((y >= 66 ) && (y <= 78))) {
                        walkableRed[y][x] = false;
                        walkableBlue[y][x] = false;
                    }

                }
            }
        }
    }
}
