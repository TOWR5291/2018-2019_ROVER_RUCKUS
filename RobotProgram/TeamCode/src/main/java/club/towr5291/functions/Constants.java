package club.towr5291.functions;

import android.content.SharedPreferences;
import android.content.res.Configuration;

import com.qualcomm.robotcore.hardware.Servo;

import club.towr5291.libraries.LibraryMotorType;
import club.towr5291.libraries.robotConfig;
import club.towr5291.libraries.robotConfigSettings;

/**
 * Created by Ian Haden on 11/7/2016.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 *  *
 * Modification history
 * Edited by:
 * Ian Haden 11/07/2016 -> Initial creation
 * Ian Haden 27/07/2018 -> Added Comments
 */


public abstract class Constants {
    //BEACON
    private static final double BEACON_WIDTH = 21.8;     //entire beacon width
    private static final double BEACON_HEIGHT = 14.5;    //entire beacon height
    private static final double BEACON_WH_RATIO = BEACON_WIDTH / BEACON_HEIGHT; //entire beacon ratio

    public static class TOWR5291Servo {
        public Servo servo;
        public double minimumPosition;
        public double maximumPosition;
        public double homePosition;

        public TOWR5291Servo(Servo servoName, double minPos, double maxPos, double homePos) {
            this.servo = servoName;
            this.minimumPosition = minPos;
            this.maximumPosition = maxPos;
            this.homePosition = homePos;
        }
    }

    public enum stepState {
        STATE_INIT ("STATE_INIT"),
        STATE_START ("STATE_START"),
        STATE_RUNNING ("STATE_RUNNING"),
        STATE_PAUSE ("STATE_PAUSE"),
        STATE_COMPLETE ("STATE_COMPLETE"),
        STATE_TIMEOUT ("STATE_TIMEOUT"),
        STATE_ERROR ("STATE_ERROR"),
        STATE_FINISHED ("STATE_FINISHED");

        private final String name;

        stepState (String name) {
            this.name = name;
        }

        public String toString() {
            return name;
        }
    }

    public enum LEDColours {
        LED_WHITE ("LED_WHITE"),        //all on
        LED_RED ("LED_RED"),            //red on
        LED_BLUE ("LED_BLUE"),          //blue on
        LED_CYAN ("LED_CYAN"),          //blue and green on
        LED_GREEN ("LED_GREEN"),        //green on
        LED_MAGENTA ("LED_MAGENTA"),    //red and blue on
        LED_YELLOW ("LED_YELLOW"),      //green and red on
        LED_OFF ("LED_OFF");

        private final String name;

        LEDColours (String name) {
            this.name = name;
        }

        public String toString() {
            return name;
        }
    }

    public enum LEDState {
        STATE_UPDATE ("STATE_UPDATE"),
        STATE_NULL ("STATE_NULL"),
        STATE_ERROR ("STATE_ERROR"),
        STATE_TEAM ("STATE_TEAM"),
        STATE_MOVING ("STATE_MOVING"),
        STATE_OBJECT ("STATE_OBJECT"),
        STATE_SUCCESS ("STATE_SUCCESS"),
        STATE_COLOUR ("STATE_COLOUR"),
        STATE_FLASHC_COLOUR ("STATE_FLASHC_COLOUR"),
        STATE_FLASHV_COLOUR ("STATE_FLASHV_COLOUR"),
        STATE_FINISHED ("STATE_FINISHED");

        private final String name;

        LEDState (String name) {
            this.name = name;
        }

        public String toString() {
            return name;
        }
    }

    public enum EYEState {
        STATE_ERROR ("STATE_ERROR"),
        STATE_MOVING ("STATE_MOVING"),
        STATE_OBJECT ("STATE_OBJECT"),
        STATE_SUCCESS ("STATE_SUCCESS"),
        STATE_WINKLEFT ("STATE_WINKLEFT"),
        STATE_WINKRIGHT ("STATE_WINKRIGHT"),
        STATE_BLINK ("STATE_BLINK"),
        STATE_OPEN ("STATE_OPEN"),
        STATE_ANGRY ("STATE_ANGRY"),
        STATE_CLOSED ("STATE_CLOSED"),
        STATE_FINISHED ("STATE_FINISHED");

        private final String name;

        EYEState (String name) {
            this.name = name;
        }

        public String toString() {
            return name;
        }
    }

    public enum RobotSide {
        LEFT ("LEFT"),
        RIGHT ("RIGHT"),
        BOTH ("BOTH");

        private final String name;

        RobotSide (String name) {
            this.name = name;
        }

        public String toString() {
            return name;
        }
    }

    public enum ObjectColours {
        OBJECT_RED ("OBJECT_RED"),
        OBJECT_BLUE ("OBJECT_BLUE"),
        OBJECT_WHITE ("OBJECT_WHITE"),
        OBJECT_RED_BLUE ("OBJECT_RED_BLUE"),
        OBJECT_BLUE_RED ("OBJECT_BLUE_RED"),
        OBJECT_RED_LEFT ("OBJECT_RED_LEFT"),
        OBJECT_RED_RIGHT ("OBJECT_RED_RIGHT"),
        OBJECT_RED_CENTER ("OBJECT_RED_CENTER"),
        OBJECT_BLUE_LEFT ("OBJECT_BLUE_LEFT"),
        OBJECT_BLUE_RIGHT ("OBJECT_BLUE_RIGHT"),
        OBJECT_NONE ("OBJECT_NONE"),
        UNKNOWN ("OBJECT_?????");

        private final String name;

        ObjectColours (String name) {
            this.name = name;
        }

        public String toString() {
            return name;
        }
    }

    public enum SharedPreferencesValues {
        ALLIANCE_COLOR ("club.towr5291.Autonomous.Color", "Red"),
        TEAM_NUMBER ("club.towr5291.Autonomous.TeamNumber", "0000"),
        ALLIANCE_START_POSITION ("club.towr5291.Autonomous.Position", "Left"),
        START_DELAY ("club.towr5291.Autonomous.Delay", "0"),
        ROBOT_MOTOR_TYPE ("club.towr5291.Autonomous.RobotMotorChoice", LibraryMotorType.MotorTypes.REV20ORBIT.toString()),
        ROBOT_BASE_CONFIG ("club.towr5291.Autonomous.RobotConfigBase", robotConfigSettings.robotConfigChoice.TileRunnerMecanum.toString()),
        DEBUG ("club.towr5291.Autonomous.Debug", "1");

        private final String SharedPrefString;
        private final String SharedPrefDefault;

        SharedPreferencesValues(String SharedPrefString, String SharedPrefDefault){
            this.SharedPrefString = SharedPrefString;
            this.SharedPrefDefault = SharedPrefDefault;
        }

        public String getSharedPrefString(){ return SharedPrefString; }
        public String getSharedPrefDefault(){ return SharedPrefDefault; }
    }

}