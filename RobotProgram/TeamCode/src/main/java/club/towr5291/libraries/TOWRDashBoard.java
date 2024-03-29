package club.towr5291.libraries;

import android.graphics.Paint;
import android.widget.TextView;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.NoSuchElementException;

/**
 * This class is a wrapper for the Telemetry class. In addition to providing a way to send named data to the Driver
 * Station to be displayed, it also simulates an LCD display similar to the NXT Mindstorms. The Mindstorms has only
 * 8 lines but this dashboard can support as many lines as the Driver Station can support. By default, we set the
 * number of lines to 16. By providing the numLines parameter when calling createInstance, you can have as many lines
 * as you want. This dashboard display is very useful for displaying debug information. In particular, the TrcMenu
 * class uses the dashboard to display a choice menu and interact with the user for choosing autonomous strategies
 * and options.
 */
public class TOWRDashBoard {
    private static final String moduleName = "HalDashboard";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;

    public static final int DEF_NUM_TEXTLINES = 16;

    private static final String displayKeyFormat = "%02d";
    private static TOWRDashBoard instance = null;
    private Telemetry telemetry = null;
    private int numLines = DEF_NUM_TEXTLINES;
    private Paint paint = null;
    private Telemetry.Item[] display;

    /**
     * This static methods creates an instance of the object if none already exist. If the object exists previously,
     * that instance is returned.
     *
     * @param telemetry specifies the Telemetry object.
     * @param numLines specifies the number of display lines.
     * @return existing instance or newly created instance of the object.
     */
    public static TOWRDashBoard createInstance(Telemetry telemetry, int numLines)
    {
        if (instance == null)
        {
            instance = new TOWRDashBoard(telemetry, numLines);
        }

        return instance;
    }   //createInstance

    /**
     * This static methods creates an instance of the object if none already exist. If the object exists previously,
     * that instance is returned.
     *
     * @param telemetry specifies the Telemetry object.
     * @return existing instance or newly created instance of the object.
     */
    public static TOWRDashBoard createInstance(Telemetry telemetry)
    {
        return createInstance(telemetry, DEF_NUM_TEXTLINES);
    }   //createInstance

    /**
     * This method returns the instance of this object if one already exist, returns null if none existed.
     *
     * @return instance of the object, null if none existed.
     */
    public static TOWRDashBoard getInstance()
    {
        return instance;
    }   //getInstance

    /**
     * Constructor: Creates an instance of the object. There should only be one global instance of this object.
     * Typically, only the FtcOpMode object should construct an instance of this object via getInstance(telemetry)
     * and nobody else.
     *
     * @param telemetry specifies the Telemetry object.
     * @param numLines specifies the number of display lines.
     */
private TOWRDashBoard(Telemetry telemetry, int numLines)
    {
        this.telemetry = telemetry;
        this.numLines = numLines;
        telemetry.clearAll();
        telemetry.setAutoClear(false);
        display = new Telemetry.Item[numLines];

        for (int i = 0; i < display.length; i++)
        {
            //display[i] = telemetry.addData(String.format(Locale.US, displayKeyFormat, i), "");
            display[i] = telemetry.addData("","" );
        }

        telemetry.update();
    }   //HalDashboard

    /**
     * This method returns the number of text lines on the display.
     *
     * @return number of display lines.
     */
    public int getNumTextLines()
    {
        return numLines;
    }   //getNumTextLines

    /**
     * This method sets the number of text lines on the display.
     *
     */
    public void setNumTextLines(int numLines)
    {
        this.numLines = numLines;
        telemetry.clearAll();
        telemetry.setAutoClear(false);
        display = new Telemetry.Item[numLines];

        for (int i = 0; i < display.length; i++)
        {
            display[i] = telemetry.addData("","" );
        }

        telemetry.update();

    }   //setNumTextLines

    /**
     * This method sets the TextView object from which to query the typeface measurement for centering/right justifying
     * messages. You don't need to call this method if you never centered or right justified messages on the dashboard.
     *
     * @param textView specifies the TextView object.
     */
    public void setTextView(TextView textView)
    {
        this.paint = textView.getPaint();
    }   //setTextView

    /**
     * This method displays a text message in the specified display line on the Driver Station.
     *
     * @param lineNum specifies the line number on the display.
     * @param text specifies the text message.
     * @param fieldWidth specified the field width in pixel units that the message will be centered or right justified
     *                   in. If specified 0, the message will be left justified.
     * @param rightJustified specifies true if text message is right justified, false if text is centered. fieldWidth
     *                       must be greater than 0. If not, this parameter is ignored.
     */
    public void displayText(int lineNum, String text, int fieldWidth, boolean rightJustified)
    {
        final String funcName = "displayText";

        if (lineNum >= 0 && lineNum < numLines)
        {
            if (fieldWidth > 0)
            {
                text = rightJustified? rightJustifiedText(fieldWidth, text): centeredText(fieldWidth, text);
            }
            display[lineNum].setValue(text);
            telemetry.update();
        }
    }   //displayText

    /**
     * This method displays a text message in the specified display line on the Driver Station.
     *
     * @param lineNum specifies the line number on the display.
     * @param text specifies the text message.
     */
    public void displayText(int lineNum, String text)
    {
        displayText(lineNum, text, 0, false);
    }   //displayText

    /**
     * This method centers a text message in the specified display line on the Driver Station.
     *
     * @param lineNum specifies the line number on the display.
     * @param text specifies the text message.
     * @param fieldWidth specified the field width in pixel units that the message will be centered in.
     */
    public void displayCenterText(int lineNum, String text, int fieldWidth)
    {
        displayText(lineNum, text, fieldWidth, false);
    }   //displayCenterText

    /**
     * This method right justifies a text message in the specified display line on the Driver Station.
     *
     * @param lineNum specifies the line number on the display.
     * @param text specifies the text message.
     * @param fieldWidth specified the field width in pixel units that the message will be right justified in.
     */
    public void displayRightText(int lineNum, String text, int fieldWidth)
    {
        displayText(lineNum, text, fieldWidth, true);
    }   //displayRightText

    /**
     * This method displays a formatted message in the specified display line on the Driver Station.
     *
     * @param lineNum specifies the line number on the display.
     * @param format specifies the format string.
     * @param args specifies variable number of substitution arguments.
     */
    public void displayPrintf(int lineNum, String format, Object... args)
    {
        String text = String.format(format, args);
        displayText(lineNum, text, 0, false);
    }   //displayPrintf

    /**
     * This method displays a formatted message in the specified display line on the Driver Station.
     *
     * @param lineNum specifies the line number on the display.
     * @param labelWidth specified the label width in pixel units that the label will be right justified in.
     * @param labelText specifies the label text that will be right justified.
     * @param format specifies the format string.
     * @param args specifies variable number of substitution arguments.
     */
    public void displayPrintf(int lineNum, int labelWidth, String labelText, String format, Object... args)
    {
        String text = rightJustifiedText(labelWidth, labelText) + String.format(format, args);
        displayText(lineNum, text, 0, false);
    }   //displayPrintf

    /**
     * This method centers a formatted message in the specified display line on the Driver Station.
     *
     * @param lineNum specifies the line number on the display.
     * @param fieldWidth specified the field width in pixel units that the message will be centered in.
     * @param format specifies the format string.
     * @param args specifies variable number of substitution arguments.
     */
    public void displayCenterPrintf(int lineNum, int fieldWidth, String format, Object... args)
    {
        String text = String.format(format, args);
        displayText(lineNum, text, fieldWidth, false);
    }   //displayCenterPrintf

    /**
     * This method right justified a formatted message in the specified display line on the Driver Station.
     *
     * @param lineNum specifies the line number on the display.
     * @param fieldWidth specified the field width in pixel units that the message will be right justified in.
     * @param format specifies the format string.
     * @param args specifies variable number of substitution arguments.
     */
    public void displayRightPrintf(int lineNum, int fieldWidth, String format, Object... args)
    {
        String text = String.format(format, args);
        displayText(lineNum, text, fieldWidth, true);
    }   //displayRightPrintf

    /**
     * This method clears all the display lines.
     */
    public void clearDisplay()
    {
        final String funcName = "clearDisplay";

        for (int i = 0; i < numLines; i++)
        {
            display[i].setValue("");
        }
        telemetry.update();
    }   //clearDisplay

    /**
     * This method refresh the display lines to the Driver Station.
     */
    public void refreshDisplay()
    {
        final String funcName = "refreshDisplay";
        telemetry.update();
    }   //refreshDisplay

    /**
     * This method returns the value of the named boolean data read from the Telemetry class.
     *
     * @param key specifies the name associated with the boolean data.
     * @return boolean data value.
     */
    public boolean getBoolean(String key)
    {
        final String funcName = "getBoolean";
        boolean value;

        String strValue = getValue(key);
        if (strValue.equals("true"))
        {
            value = true;
        }
        else if (strValue.equals("false"))
        {
            value = false;
        }
        else
        {
            throw new IllegalArgumentException("object is not boolean");
        }

        return value;
    }   //getBoolean

    /**
     * This method returns the value of the named boolean data read from the Telemetry class. If the named data does
     * not exist, it is created and assigned the given default value. Then it is sent to the Driver Station.
     *
     * @param key specifies the name associated with the boolean data.
     * @param defaultValue specifies the default value if it does not exist.
     * @return boolean data value.
     */
    public boolean getBoolean(String key, boolean defaultValue)
    {
        final String funcName = "getBoolean";
        boolean value;

        try
        {
            value = getBoolean(key);
        }
        catch (NoSuchElementException e)
        {
            putBoolean(key, defaultValue);
            value = defaultValue;
        }

        return value;
    }   //getBoolean

    /**
     * This method sets the named boolean data with the given value and also sends it to the Driver Station.
     *
     * @param key specifies the name associated with the boolean data.
     * @param value specifies the data value.
     */
    public void putBoolean(String key, boolean value)
    {
        final String funcName = "putBoolean";

        telemetry.addData(key, Boolean.toString(value));
    }   //putBoolean

    /**
     * This method returns the value of the named double data read from the Telemetry class.
     *
     * @param key specifies the name associated with the double data.
     * @return double data value.
     */
    public double getNumber(String key)
    {
        final String funcName = "getNumber";
        double value;

        try
        {
            value = Double.parseDouble(getValue(key));
        }
        catch (NumberFormatException e)
        {
            throw new IllegalArgumentException("object is not a number");
        }

        return value;
    }   //getNumber

    /**
     * This method returns the value of the named double data read from the Telemetry class. If the named data does
     * not exist, it is created and assigned the given default value. Then it is sent to the Driver Station.
     *
     * @param key specifies the name associated with the double data.
     * @param defaultValue specifies the default value if it does not exist.
     * @return double data value.
     */
    public double getNumber(String key, double defaultValue)
    {
        final String funcName = "getNumber";
        double value;

        try
        {
            value = getNumber(key);
        }
        catch (NoSuchElementException e)
        {
            putNumber(key, defaultValue);
            value = defaultValue;
        }

        return value;
    }   //getNumber

    /**
     * This method sets the named double data with the given value and also sends it to the Driver Station.
     *
     * @param key specifies the name associated with the double data.
     * @param value specifies the data value.
     */
    public void putNumber(String key, double value)
    {
        final String funcName = "putNumber";

        telemetry.addData(key, Double.toString(value));
    }   //putNumber

    /**
     * This method returns the value of the named string data read from the Telemetry class.
     *
     * @param key specifies the name associated with the string data.
     * @return string data value.
     */
    public String getString(String key)
    {
        final String funcName = "getString";
        String value = getValue(key);

        return value;
    }   //getString

    /**
     * This method returns the value of the named string data read from the Telemetry class. If the named data does
     * not exist, it is created and assigned the given default value. Then it is sent to the Driver Station.
     *
     * @param key specifies the name associated with the string data.
     * @param defaultValue specifies the default value if it does not exist.
     * @return string data value.
     */
    public String getString(String key, String defaultValue)
    {
        final String funcName = "getString";
        String value;

        try
        {
            value = getString(key);
        }
        catch (NoSuchElementException e)
        {
            putString(key, defaultValue);
            value = defaultValue;
        }

        return value;
    }   //getString

    /**
     * This method sets the named string data with the given value and also sends it to the Driver Station.
     *
     * @param key specifies the name associated with the string data.
     * @param value specifies the data value.
     */
    public void putString(String key, String value)
    {
        final String funcName = "putString";

        telemetry.addData(key, value);
    }   //putString

    /**
     * This method calls Telemetry class to retrieve the named data item.
     *
     * @param key specifies the name associated with the string data.
     * @return string data associated with the given name.
     */
    private String getValue(String key)
    {
        throw new UnsupportedOperationException("Not support in FTC.");
    }   //getValue

    /**
     * This method calculates the number of padding spaces to be inserted in front of the original text so that the
     * text will be right justified within the given pixel width. The resulted string will be returned.
     *
     * @param totalWidth specifies the total pixel width.
     * @param text specifies the text string.
     * @return new string that has prepended padding spaces.
     */
    private String rightJustifiedText(int totalWidth, String text)
    {
        int paddingSpaces =
                paint == null? 0: Math.round((totalWidth - paint.measureText(text))/paint.measureText(" "));
        return String.format("%" + (paddingSpaces + text.length()) + "s", text);
    }   //rightJustifiedText

    /**
     * This method calculates the number of padding spaces to be inserted in front of the original text so that the
     * text will be centered within the given pixel width. The resulted string will be returned.
     *
     * @param totalWidth specifies the total pixel width.
     * @param text specifies the text string.
     * @return new string that has prepended padding spaces.
     */
    private String centeredText(int totalWidth, String text)
    {
        int paddingSpaces =
                paint == null? 0: Math.round((totalWidth - paint.measureText(text))/paint.measureText(" ")/2);
        return String.format("%" + (paddingSpaces + text.length()) + "s", text);
    }   //centeredText
}
