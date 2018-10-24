package club.towr5291.functions;


import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;

import club.towr5291.libraries.LibraryOCVHSVFilter;
import club.towr5291.libraries.TOWRDashBoard;

import static org.opencv.core.Core.flip;

/**
 * Created by LZTDD0 on 11/7/2016.
 */

public class RoverRuckusOCV {

    private final static double MIN_COLOR_ZONE_AREA = 0.2;// fraction of total image area

    private Mat hsvImg;
    private Mat finalImg;
    private Mat jewelMaskImg;
    private Mat zonedImg;
    private Mat tmpHsvImg;
    private Mat tmp1Img;
    private Mat tmp2Img;
    private Mat maskImg;
    private Mat showImg;
    private Mat cvImage;
    private Mat colorDiff;
    private Mat onesImg;
    private Mat zeroImg;
    private Mat white;
    private Mat out;
    private Mat original;
    private Mat crap1;
    private Mat crap2;
    private Mat btnTmpImg;
    private Mat blue;
    private Mat red;

    private Rect white_box = new Rect();
    private Rect blue_box = new Rect();
    private Rect red_box = new Rect();

    private List<Mat>        hsv_channels    = new ArrayList<>();
    private List<Mat>        rgb_channels    = new ArrayList<>();
    private List<MatOfPoint> red_blobs       = new ArrayList<>();
    private List<MatOfPoint> blue_blobs      = new ArrayList<>();
    private List<MatOfPoint> white_blobs     = new ArrayList<>();
    private List<MatOfPoint> black_blobs     = new ArrayList<>();

    private ArrayList<Rect>  red_matches     = new ArrayList<>();
    private ArrayList<Rect>  blue_matches    = new ArrayList<>();
    private ArrayList<Rect>  white_matches   = new ArrayList<>();
    private List<Rect>       buttons         = new ArrayList<>();
    private ArrayList<Point> centroidRed     = new ArrayList<>();
    private ArrayList<Point> centroidBlue    = new ArrayList<>();
    private ArrayList<Point> centroidWhite   = new ArrayList<>();

    private double lumAvg = 0;

    private Constants.ObjectColours ObjectColourResult;

    private int debug = 1;

    private int imageCounter;

    //set up the variables for the logger
    private FileLogger fileLogger;
    private static final String TAG = "JewelAnalysisOCV";

    LibraryOCVHSVFilter processingHSV = new LibraryOCVHSVFilter(0,0,0,0,0,0);
    private HashMap<String,LibraryOCVHSVFilter> HSVRedFilters = new HashMap<String,LibraryOCVHSVFilter>();
    private HashMap<String,LibraryOCVHSVFilter> HSVCrapFilters = new HashMap<String,LibraryOCVHSVFilter>();
    private int loadHSVRedindex = 0;
    private int loadHSVBlueindex = 0;
    private int loadHSVCrapindex = 0;
    private double imageTimeStamp;

    //default resolution
    int desiredWidth = 1280;
    int desiredHeight = 720;

    public RoverRuckusOCV() {

    }

    public Constants.ObjectColours JewelAnalysisOCV(FileLogger fileLoggerFromMaster, TOWRDashBoard dashBoard, Mat img, int count, boolean flipit, int quadrant, boolean calibrate) {

        this.fileLogger = fileLoggerFromMaster;

        debug = fileLogger.getDebugLevel();

        //clear out old information
        finalImg = new Mat();
        original = new Mat();
        hsvImg = new Mat();
        crap1 = new Mat();
        crap2 = new Mat();
        btnTmpImg = new Mat();
        showImg = new Mat();

        hsv_channels.clear();
        rgb_channels.clear();
        white_blobs.clear();
        red_blobs.clear();
        red_matches.clear();
        white_matches.clear();
        buttons.clear();
        centroidRed.clear();
        centroidWhite.clear();
        white_box = new Rect( 0, 0, 1, 1 );

        imageTimeStamp = System.currentTimeMillis();

        //start loading HSV Paramters from file, if this takes too long abandon this process
        readHSVFiltersFromFile("HSVFilters.csv");

        fileLogger.writeEvent(3, TAG, "HSV Filters loaded");

        imageCounter = count;
        Rect roi = new Rect(img.width() / 2, img.height() / 2, img.width() / 2, img.height() / 2);

        double offset = img.height() / 6;

        switch (quadrant) {
            case 0:
                //All The image
                roi = new Rect(0, 0, img.width(), img.height());
                break;
            case 1:
                // top right
                // Just get where the Jewel is so we don't analyse the wrong items in the image
                roi = new Rect(0, 0, img.width() / 2, img.height() / 2);
                break;
            case 2:
                // top left
                // Just get where the Jewel is so we don't analyse the wrong items in the image
                roi = new Rect(img.width() / 2, 0, img.width() / 2, img.height() / 2);
                break;
            case 3:
                //bottom right
                // Just get where the Jewel is so we don't analyse the wrong items in the image
                roi = new Rect(img.width() / 2, img.height() / 2 - (int)offset, img.width() / 2, img.height() / 2);
                break;
            case 4:
                // bottom left
                // Just get where the Jewel is so we don't analyse the wrong items in the image
                roi = new Rect(0, img.height() / 2 - (int)offset, img.width() / 2, img.height() / 2);
                break;
            default:
                // Just get where the Jewel is so we don't analyse the wrong items in the image
                roi = new Rect(img.width() / 2, img.height() / 2, img.width() / 2, img.height() / 2);
        }

        if (flipit) {
            flip(img,img,-1);
            // camera image size by default is 1280x720
        }

        SaveImage(3,img, imageTimeStamp + " 01 Initial " + imageCounter );

        Mat cropped = new Mat(img, roi);
        original = cropped.clone();

        fileLogger.writeEvent(3, TAG, "Cropped");

        SaveImage(3, original, imageTimeStamp + " 02 Cropped" + imageCounter );

        fileLogger.writeEvent(3, TAG, "HSV cvt.Color");

        //convert to HSV Colour space
        Imgproc.cvtColor( original, hsvImg, Imgproc.COLOR_RGB2HSV, 4 );

        SaveImage(9, hsvImg, imageTimeStamp + "-03 HSV Image " + imageCounter );

        fileLogger.writeEvent(3, TAG, "HSV copy to show");

        hsvImg.copyTo(showImg);

        SaveImage(9, showImg, imageTimeStamp + "-04 showImg " + count );

        for(int c = 0; c < img.channels(); c++)
        {
            rgb_channels.add(new Mat());
        }

        for(int c = 0; c < hsvImg.channels(); c++)
        {
            hsv_channels.add(new Mat());
        }

        fileLogger.writeEvent(3, TAG, "splitting RGD Channels to RED and Blue");

        //Core.split( original, rgb_channels );
        red = new Mat();
        blue = new Mat();
        Core.extractChannel(original,red,2);
        Core.extractChannel(original,blue,0);

        fileLogger.writeEvent(3, TAG, "Saving RED and Blue to 5 and 6");

        SaveImage(9,red, imageTimeStamp + "-05 red " + imageCounter);
        SaveImage(9,blue, imageTimeStamp + "-06 blue " + imageCounter);

        fileLogger.writeEvent(3,TAG, "Creating a new MAT colorDiff");
        fileLogger.writeEvent(3,TAG, "colorDiff Size " + red.rows() + ", " + red.cols());
        fileLogger.writeEvent(3,TAG, "colorDiff Type " + red.type());

        if (calibrate) {
            if (imageCounter < 1 )
                findAreas(dashBoard);
            ObjectColourResult = Constants.ObjectColours.OBJECT_RED_BLUE;
        } else {
            fileLogger.writeEvent(1, TAG, "findRed() Start");
            findRed2();

            fileLogger.writeEvent(1, TAG, "draw() Start");
            finalImg = draw();

            SaveImage(2, finalImg, imageTimeStamp + "-99 final " + imageCounter);

            calcPosition();
        }
        return ObjectColourResult;
    }

    private void findAreas(TOWRDashBoard dashBoard) {
        Mat imageAreas = new Mat();
        Mat mergeBlueSubtracted = new Mat();
        Mat mergeRedSubtracted = new Mat();
        Mat imageBlueRGB = new Mat();
        Mat imageRedRGB = new Mat();

        fileLogger.writeEvent(3,TAG, "Finding HSV Values");

        SaveImage(1,blue, "97 FA Blue " + imageCounter );
        SaveImage(1,red, "97 FA Red " + imageCounter );

        Imgproc.cvtColor( blue, imageBlueRGB, Imgproc.COLOR_GRAY2RGB, 4);
        SaveImage(1,imageBlueRGB, "98 FA BlueRGB " + imageCounter );
        fileLogger.writeEvent(3,TAG, "Blue Channels " + blue.channels());
        fileLogger.writeEvent(3,TAG, "BlueRGB Channels " + imageBlueRGB.channels());
        fileLogger.writeEvent(3,TAG, "original Channels " + original.channels());

        Core.subtract(original, imageBlueRGB, mergeBlueSubtracted);
        SaveImage(1,mergeBlueSubtracted, "98 FA BlueSub " + imageCounter );

        Imgproc.cvtColor( red, imageRedRGB, Imgproc.COLOR_GRAY2RGB, 4);
        SaveImage(1,imageRedRGB, "98 FA RedRGB " + imageCounter );
        Core.subtract(original, imageRedRGB, mergeRedSubtracted);
        SaveImage(1,mergeRedSubtracted, "98 FA RedSub " + imageCounter );

        //lets try find a value or values that work
        for (int x = 0; x < 265; x = x + 10) {
            for (int y = 0; y < 265; y = y + 10) {
                for (int z = 0; z < 265; z = z + 10) {
                    Core.inRange(mergeRedSubtracted, new Scalar(x, y, z), new Scalar(x + 20, y + 20, z + 20), imageAreas);
                    if (Core.countNonZero(imageAreas) > 0) {
                        SaveImage(1, imageAreas, "99 FA R Seeking " + imageCounter + "- x " + x + " y " + y + " z " + z);
                    }
                    dashBoard.displayPrintf(9, "X = " + String.valueOf(x));
                    dashBoard.displayPrintf(10, "Y = " + String.valueOf(y));
                    dashBoard.displayPrintf(11, "Z = " + String.valueOf(z));
                }
            }
        }

    }

    private void findRed2()
    {
        // Same game, just a different hue
        Mat red1 = new Mat();
        Mat red_areas = new Mat();
        Mat redSubtracted = new Mat();
        Mat redRGB = new Mat();
        Mat tmpHsv2Img = new Mat();

        fileLogger.writeEvent(3,TAG, "Start Red Filter");
        Imgproc.cvtColor( red, redRGB, Imgproc.COLOR_GRAY2RGB,4);

        Core.subtract(original,redRGB,redSubtracted);

        // this process is 200ms
        for (int x = 0; x < loadHSVRedindex; x++) {
            if (x == 0) {
                if (HSVRedFilters.containsKey(String.valueOf(x))) {
                    processingHSV = HSVRedFilters.get(String.valueOf(String.valueOf(x)));
                    Core.inRange(redSubtracted, new Scalar(processingHSV.getmHLowerLimit(), processingHSV.getmSLowerLimit(), processingHSV.getmVLowerLimit()), new Scalar(processingHSV.getmHUpperLimit(), processingHSV.getmSUpperLimit(), processingHSV.getmVUpperLimit()), red_areas);
                }
            } else {
                if (HSVRedFilters.containsKey(String.valueOf(x))) {
                    processingHSV = HSVRedFilters.get(String.valueOf(String.valueOf(x)));
                    Core.inRange(redSubtracted, new Scalar(processingHSV.getmHLowerLimit(), processingHSV.getmSLowerLimit(), processingHSV.getmVLowerLimit()), new Scalar(processingHSV.getmHUpperLimit(), processingHSV.getmSUpperLimit(), processingHSV.getmVUpperLimit()), red1);
                    Core.bitwise_or(red_areas, red1, red_areas);
                }
            }
        }
        fileLogger.writeEvent(3,TAG, "Finish Red Filter");

        //filter out the known crap
        if (!crap1.empty())
            Core.subtract(red_areas, crap1, red_areas);
        SaveImage(9,red_areas, imageTimeStamp + "-28.1 findRed subtract " + imageCounter);

        red_areas.copyTo(tmpHsv2Img);

        SaveImage(9,red_areas, imageTimeStamp + "-29 findRed multiply " + imageCounter );
        red_areas.copyTo(tmpHsv2Img);
        Imgproc.dilate( tmpHsv2Img, red_areas, new Mat() );
        SaveImage(9,red_areas, imageTimeStamp + "-30 findRed dilate " + imageCounter );
        findWeightedPos( red_areas, red_blobs, red_matches, centroidRed );
    }

    public Mat draw() {
        out = new Mat();
        fileLogger.writeEvent(1, "OUT", "Defined");
        red_box = new Rect(0,0,0,0);
        fileLogger.writeEvent(1, "RED_BOX", "Defined");
        //if (out == null)
        //    out = original.clone();
        original.copyTo(out);
        fileLogger.writeEvent(1, "ORIGINAL", "COPYTo OUT");

        for ( Rect rb : red_matches ) {
            fileLogger.writeEvent(1, "for red_matches");
            Imgproc.rectangle( out, rb.tl(), rb.br(), new Scalar(255,150,150), 3 );
            if (red_box.area() < rb.area())
                red_box = rb;
        }

        fileLogger.writeEvent(1, "DONE", "DONE 1");

        for ( Point rc : centroidRed) {
            fileLogger.writeEvent(1, "cetroidRED", "RC");
            Imgproc.circle(out, rc, 50, new Scalar(255, 0, 0), 5);
        }

        fileLogger.writeEvent(1, "HElP", "HELP");
        fileLogger.writeEvent(3,TAG, "Red Box Area " +red_box.area());
        fileLogger.writeEvent(1, "HElP 1", "HELP");

        SaveImage(1, out, "Test Red Blobs");
        Imgproc.drawContours( out, red_blobs, -1, new Scalar(255,0,0), 2 );

        fileLogger.writeEvent(1, "HElP 2", "HELP");

        return out;
    }

    private void calcPosition()
    {

        Point centroidRedPosition;
        Point centroidBluePosition;

        ObjectColourResult = Constants.ObjectColours.UNKNOWN;
        fileLogger.writeEvent(3,TAG, "Jewel colour - colour unknown");
        fileLogger.writeEvent(3,TAG, "Jewel centroidBlue " + centroidBlue.size());
        fileLogger.writeEvent(3,TAG, "Jewel centroidRed " + centroidRed.size());
        fileLogger.writeEvent(3,TAG, "Jewel red_box area " + red_box.area());
        fileLogger.writeEvent(3,TAG, "Jewel blue_box area " + blue_box.area());

        try {
            centroidBluePosition = centroidBlue.get(0);
            centroidRedPosition = centroidRed.get(0);

            //can see 2 jewels, need to work out which one is left or right
            if ((red_box.area() > 10000) && (blue_box.area() > 10000)) {
                if (centroidBluePosition.x < centroidRedPosition.x) {
                    ObjectColourResult = Constants.ObjectColours.OBJECT_BLUE_RED;
                    return;
                } else if (centroidBluePosition.x > centroidRedPosition.x) {
                    ObjectColourResult = Constants.ObjectColours.OBJECT_RED_BLUE;
                    return;
                }
            }
        } catch (Exception e) {
            //throw new RuntimeException("Error in reading CSV file:" + ex);
            fileLogger.writeEvent(2,TAG, "Error getting centroid:" + e);
        }


        if ( red_box.area()  > blue_box.area() ) {
            ObjectColourResult = Constants.ObjectColours.OBJECT_RED;
            return;
        } else if ( red_box.area() < blue_box.area() ) {
            ObjectColourResult = Constants.ObjectColours.OBJECT_BLUE;
            return;
        }

        ObjectColourResult = Constants.ObjectColours.UNKNOWN;
        return;

    }

    public void findWeightedPos( Mat img, List<MatOfPoint> calcCtr, ArrayList<Rect> boxMatches, ArrayList<Point> centroid ) {

        Mat hchy = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();

        Imgproc.findContours(img, contours, hchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find max contour area
        double maxArea = 0;
        Iterator<MatOfPoint> each = contours.iterator();
        while (each.hasNext()) {
            MatOfPoint wrapper = each.next();
            double area = Imgproc.contourArea(wrapper);
            if (area > maxArea)
                maxArea = area;
        }

        // Filter contours by area and resize to fit the original image size

        double contour_area, box_area;
        Rect bounded_box;

        calcCtr.clear();
        boxMatches.clear();
        centroid.clear();

        each = contours.iterator();
        while (each.hasNext()) {

            MatOfPoint contour = each.next();

            contour_area = Imgproc.contourArea( contour );
            bounded_box = Imgproc.boundingRect( contour );

            if ( contour_area > MIN_COLOR_ZONE_AREA * maxArea ) {
                int largestContour = contoursLargestIndex(contours);
                centroid.add(massCenterMatOfPoint2f(contours.get(largestContour)));
                calcCtr.add(contour);
                boxMatches.add( bounded_box );

            }
        }
    }

    private static int contoursLargestIndex(List<MatOfPoint> contours) {
        double maxArea = -1;
        int maxAreaIdx = 0;
        for (int idx = 0; idx < contours.size(); idx++) {
            Mat contour = contours.get(idx);
            double contourarea = Imgproc.contourArea(contour);
            if (contourarea > maxArea) {
                maxArea = contourarea;
                maxAreaIdx = idx;
            }
        }
        return maxAreaIdx;
    }

    private Point massCenterMatOfPoint2f(MatOfPoint map)
    {
        Moments moments = Imgproc.moments(map, true);
        Point centroid = new Point();
        centroid.x = moments.get_m10() / moments.get_m00();
        centroid.y = moments.get_m01() / moments.get_m00();
        return centroid;
    }

    private double scoreFit( Rect wb, Rect rb, Rect bb )
    {
        double beac_w = 9.0;
        double beac_h = 6.5;

        double actl_beac_rt = beac_w / beac_h;
        double sens_bcn_rt = wb.width / wb.height;

        double beac_rt = wb.area() / ( hsvImg.cols() * hsvImg.rows() );
        double red_rt = rb.area() / wb.area();
        double blue_rt = bb.area() / wb.area();

        double beac_aspect_factor =
                Math.pow( Range.clip( sens_bcn_rt - actl_beac_rt, -1.0, 1.0 ) / actl_beac_rt, 2 );
        double wb_ratio_factor =
                Math.pow( Range.clip( 0.6 - beac_rt, -0.6, 0.6 ) * 1.67, 2 );
        double rb_ratio_factor =
                Math.pow( Range.clip( 0.4 - ( red_rt + blue_rt ) / 2, -0.4, 0.4 ) * 2.5, 2 );

        return Range.clip( 1 - Math.sqrt(
                ( beac_aspect_factor +
                        3 * wb_ratio_factor +
                        2 * rb_ratio_factor
                ) / 6.0 ), 0.0, 1.0 );
    }

    private double minOf( double w, double r, double b )
    {
        return ( w + 2 * Math.min( r, b ) ) / 3;
    }

    private double maxOf( double w, double r, double b )
    {
        return ( w + 2 * Math.max( r, b ) ) / 3;
    }

    private Rect bestFit( Rect wb, ArrayList<Rect> matches )
    {
        int w_ctr_x = wb.x + wb.width / 2;
        Double mp1, mp2, mp3, mpf;
        Double minFit = Double.POSITIVE_INFINITY;
        Rect best = new Rect( wb.x, wb.y, 1, 1 );

        for ( Rect cb : matches )
        {
            if ( wb.contains( new Point( cb.x + cb.width / 2, cb.y + cb.height / 2 ) ) )
            {
                mp1 = Math.pow( wb.tl().y - cb.tl().y, 2 );
                mp2 = Math.pow( wb.br().y - cb.br().y, 2 );
                mp3 = Math.min( Math.pow( w_ctr_x - cb.tl().x, 2 ), Math.pow( w_ctr_x - cb.br().x, 2 ) );

                mpf = Math.sqrt( mp1 + mp2 + mp3 / 3.0 );
                if ( mpf < minFit )
                {
                    minFit = mpf;
                    best = cb;
                }
            }
        }
        return best;
    }

    public void SaveImage (int debug, Mat mat, String info) {
        if (fileLogger.getDebugLevel() >= debug )
            SaveImage(mat, info );
    }

    public void SaveImage (Mat mat, String info) {
        Mat mIntermediateMat = new Mat();
        Mat mIntermediateMat2 = new Mat();

        mat.convertTo(mIntermediateMat2, CvType.CV_8UC4);
        if (mIntermediateMat2.channels() > 2)
            Imgproc.cvtColor(mIntermediateMat2, mIntermediateMat, Imgproc.COLOR_RGBA2BGR, 3);
        else
            mIntermediateMat = mIntermediateMat2;


        File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES);
        String filename = info + ".png";
        File file = new File(path, filename);

        Boolean bool = null;
        filename = file.toString();
        bool = Imgcodecs.imwrite(filename, mIntermediateMat);

        fileLogger.writeEvent(1,TAG, "Writing image " + filename + " = " + bool);
    }

    public Mat loadImageFromFile(String fileName) {

        Mat rgbLoadedImage = null;

        File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES);
        File file = new File(path, fileName);

        // this should be in BGR format according to the
        // documentation.
        Mat image = Imgcodecs.imread(file.getAbsolutePath());

        if (image.width() > 0) {

            rgbLoadedImage = new Mat(image.size(), image.type());

            Imgproc.cvtColor(image, rgbLoadedImage, Imgproc.COLOR_BGR2RGB);

            Log.d("OpenCVLoadImage", "loadedImage: " + "chans: " + image.channels() + ", (" + image.width() + ", " + image.height() + ")");

            image.release();
            image = null;
        }

        return rgbLoadedImage;
        //return image;

    }

    private void loadHSVFilter(String Filter, int parm1, int parm2, int parm3, int parm4, int parm5, int parm6)
    {
        switch (Filter) {
            case "red":
                HSVRedFilters.put(String.valueOf(loadHSVRedindex), new LibraryOCVHSVFilter(parm1, parm2, parm3, parm4, parm5, parm6));
                loadHSVRedindex++;
                break;
            case "crap":
                HSVCrapFilters.put(String.valueOf(loadHSVCrapindex), new LibraryOCVHSVFilter(parm1, parm2, parm3, parm4, parm5, parm6));
                loadHSVCrapindex++;
                break;
            default:
                fileLogger.writeEvent(1,TAG, "CSV Value UNKNOWN " + Filter);
                break;

        }
    }

    private void readHSVFiltersFromFile(String Filename) {
        try {
            File f = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS + "/Sequences"), Filename);
            BufferedReader reader = new BufferedReader(new FileReader(f));

            String csvLine;
            while((csvLine = reader.readLine()) != null) {
                //check if line is a comment and ignore it
                if (csvLine.substring(0, 2).equals("//")) {

                } else {
                    String[] row = csvLine.split(",");
                    fileLogger.writeEvent(2,TAG, "CSV Value " + row[0].trim() + "," + row[1].trim() + "," + row[2].trim() + "," + row[3].trim() + "," + row[4].trim() + "," + row[5].trim() + "," + row[6].trim());
                    loadHSVFilter(row[0].trim(),Integer.parseInt(row[1].trim()),Integer.parseInt(row[2].trim()),Integer.parseInt(row[3].trim()),Integer.parseInt(row[4].trim()),Integer.parseInt(row[5].trim()),Integer.parseInt(row[6].trim()));
                }
            }
        } catch(IOException ex) {
            //throw new RuntimeException("Error in reading CSV file:" + ex);
            fileLogger.writeEvent(2,TAG, "Error in reading HSV CSV file:" + ex);
        }
    }
}