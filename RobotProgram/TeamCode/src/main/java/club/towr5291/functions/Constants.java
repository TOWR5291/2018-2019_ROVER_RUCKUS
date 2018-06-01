package club.towr5291.functions;

/**
 * Created by LZTDD0 on 11/7/2016.
 */

public abstract class Constants {
    //BEACON
    public static final double BEACON_WIDTH = 21.8;     //entire beacon width
    public static final double BEACON_HEIGHT = 14.5;    //entire beacon height
    public static final double BEACON_WH_RATIO = BEACON_WIDTH / BEACON_HEIGHT; //entire beacon ratio

    public enum stepState {
        STATE_INIT,
        STATE_START,
        STATE_RUNNING,
        STATE_PAUSE,
        STATE_COMPLETE,
        STATE_TIMEOUT,
        STATE_ERROR,
        STATE_FINISHED
    }

    public enum LEDColours {
        LED_WHITE,  //all on
        LED_RED, //red on
        LED_BLUE, //blue on
        LED_CYAN, //blue and green on
        LED_GREEN,  //green on
        LED_MAGENTA,  //red and blue on
        LED_YELLOW,  //green and red on
        LED_OFF

    }

    public enum LEDState {
        STATE_NULL,
        STATE_ERROR,
        STATE_TEAM,
        STATE_MOVING,
        STATE_OBJECT,
        STATE_SUCCESS,
        STATE_COLOUR,
        STATE_FLASHC_COLOUR,
        STATE_FLASHV_COLOUR,
        STATE_FINISHED
    }

    public enum RobotSide {
        LEFT,
        RIGHT,
        BOTH
    }

    public enum ObjectColours {
        OBJECT_RED,
        OBJECT_BLUE,
        OBJECT_RED_BLUE,
        OBJECT_BLUE_RED,
        OBJECT_RED_LEFT,
        OBJECT_RED_RIGHT,
        OBJECT_BLUE_LEFT,
        OBJECT_BLUE_RIGHT,
        UNKNOWN;

        public String toString() {
            switch (this) {
                case OBJECT_RED:
                    return "RED";
                case OBJECT_BLUE:
                    return "BLUE";
                case OBJECT_RED_BLUE:
                    return "RED_BLUE";
                case OBJECT_RED_LEFT:
                    return "RED_LEFT";
                case OBJECT_RED_RIGHT:
                    return "RED_RIGHT";
                case OBJECT_BLUE_RED:
                    return "BLUE_RED";
                case OBJECT_BLUE_LEFT:
                    return "BLUE_LEFT";
                case OBJECT_BLUE_RIGHT:
                    return "BLUE_RIGHT";
                case UNKNOWN:
                default:
                    return "???";
            }
        }

    }
}
