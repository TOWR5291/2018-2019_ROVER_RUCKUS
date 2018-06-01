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

/**
 * Created by LZTDD0 on 11/7/2016.
 */

public class BeaconAnalysisOCVPlayground {

    private final static double MIN_COLOR_ZONE_AREA = 0.2;// fraction of total image area

    private Mat hsvImg;
    private Mat finalImg;
    private Mat beaconMaskImg;
    private Mat zonedImg;
    private Mat tmpHsvImg;
    private Mat tmp1Img;
    private Mat tmp2Img;
    private Mat maskImg;
    private Mat showImg = new Mat();
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
    private ArrayList<Point> centroidButtons = new ArrayList<>();
    private ArrayList<Point> centroidRed     = new ArrayList<>();
    private ArrayList<Point> centroidBlue    = new ArrayList<>();
    private ArrayList<Point> centroidWhite   = new ArrayList<>();

    private Rect red_box;
    private Rect blue_box;
    private Rect white_box;
    private Rect beacon_box;

    private double lumAvg = 0;

    private Constants.ObjectColours ObjectColourResult;

    private int debug = 4;

    private int imageCounter;

    //set up the variables for the logger
    private String startDate;
    private ElapsedTime runtime = new ElapsedTime();
    private FileLogger fileLogger;
    private static final String TAG = "BeaconAnalysisOCV2";

    LibraryOCVHSVFilter processingHSV = new LibraryOCVHSVFilter(0,0,0,0,0,0);
    private HashMap<String,LibraryOCVHSVFilter> HSVRedFilters = new HashMap<String,LibraryOCVHSVFilter>();
    private HashMap<String,LibraryOCVHSVFilter> HSVBlueFilters = new HashMap<String,LibraryOCVHSVFilter>();
    private HashMap<String,LibraryOCVHSVFilter> HSVCrapFilters = new HashMap<String,LibraryOCVHSVFilter>();
    private int loadHSVRedindex = 0;
    private int loadHSVBlueindex = 0;
    private int loadHSVCrapindex = 0;
    private double imageTimeStamp;

    public BeaconAnalysisOCVPlayground() {


    }

    public Constants.ObjectColours BeaconAnalysisOCVPlayground(int debuglevel, Mat img, int count, Point beacTopL, Point beacBotR, Point beacMiddle) {

        //set debug level based on menu system
        debug = debuglevel;
        debug = 10;

        if ((count >= 1) && (debug >= 9))
            return Constants.ObjectColours.UNKNOWN;
        
        //clear out old information
        finalImg = new Mat();
        original = new Mat();
        hsvImg = new Mat();
        crap1 = new Mat();
        crap2 = new Mat();
        btnTmpImg = new Mat();

        hsv_channels.clear();
        rgb_channels.clear();
        white_blobs.clear();
        red_blobs.clear();
        blue_blobs.clear();
        red_matches.clear();
        blue_matches.clear();
        white_matches.clear();
        buttons.clear();
        centroidRed.clear();
        centroidBlue.clear();
        centroidWhite.clear();
        white_box = new Rect( 0, 0, 1, 1 );
        blue_box = new Rect( 0, 0, 1, 1 );
        red_box = new Rect( 0, 0, 1, 1 );
        beacon_box = new Rect( 0, 0, 1, 1 );

        imageTimeStamp = System.currentTimeMillis();


        if (debug >= 1)
        {
            startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());
            fileLogger = new FileLogger(runtime);
            fileLogger.open();
            fileLogger.write("Time,SysMS,Thread,Event,Desc");
            fileLogger.writeEvent(TAG, "Log Started");
            fileLogger.writeEvent(TAG, "Read and Load HSV Filters");
        }

        //start loading HSV Paramters from file, if this takes too long abandon this process
        readHSVFiltersFromFile("HSVFilters.csv");

        if (debug >= 3)
        {
            fileLogger.writeEvent(TAG, "HSV Filters loaded");
        }

        imageCounter = count;

        //image from Lake Orion School
        //img = loadImageFromFile("test1.png");

        //image from FORC
        //img = loadImageFromFile("test2.png");

        //image from outside in sun
        //img = loadImageFromFile("test3.png");

        //image from home
        //img = loadImageFromFile("test4.png");

        if (debug >= 2)
            SaveImage(img, imageTimeStamp + "-01 initial image " + imageCounter );

        Rect roi = new Rect(0, 0, 0, 0);

        //crop the image to half the height, bottom half is just the wall as camera is mounted 12 inches
        //beaconMaskImg = new Mat(360,1280,CvType.CV_8UC4);
        //if (debug >= 9) {
            //takes 2.5 seconds to process at this size
            //image is 1280 x720
            //roi = new Rect(0, 0, 1280, 360);
        //} else

        // added march 20 ian to see if we get Beacon by itself

        //roi = new Rect((int)beacTopL.x, (int)beacTopL.y, (int)beacBotR.x - (int)beacTopL.x, (int)beacBotR.y - (int)beacTopL.y);
        Log.d("Vuforia", "beacTopL.x "  + beacTopL.x);
        Log.d("Vuforia", "beacTopL.y "  + beacTopL.y);
        Log.d("Vuforia", "beacBotR.x "  + beacBotR.x);
        Log.d("Vuforia", "beacBotR.y "  + beacBotR.y);

        //commented March 20 Ian
        //see if reducing image size make its faster
        //takes 600ms to process at this size
        Size size = new Size(0, 0);
        if (img.height() == 360) {
            size = new Size(640, 180);//the dst image size,e.g.100x100
        } else {
            size = new Size(640, 360);//the dst image size,e.g.100x100
        }
        Imgproc.resize(img, img, size);
        roi = new Rect(0, 0, 640, 180);

        Mat cropped = new Mat(img, roi);
        original = cropped.clone();

        //if (debug >= 9)
            SaveImage(original, imageTimeStamp + "-02 cropped " + imageCounter );

        //convert to HSV Colour space
        Imgproc.cvtColor( original, hsvImg, Imgproc.COLOR_RGB2HSV, 4 );

        if (debug >= 9)
            SaveImage(hsvImg, imageTimeStamp + "-03 HSV Image " + imageCounter );

        hsvImg.copyTo(showImg);

        if (debug >= 9)
            SaveImage(showImg, imageTimeStamp + "-04 showImg " + count );

        for(int c = 0; c < img.channels(); c++)
        {
            rgb_channels.add(new Mat());
        }

        for(int c = 0; c < hsvImg.channels(); c++)
        {
            hsv_channels.add(new Mat());
        }

        Core.split( original, rgb_channels );
        Mat red = rgb_channels.get( 0 );
        Mat blue = rgb_channels.get( 2 );

        if (debug >= 9)
            SaveImage(red, imageTimeStamp + "-05 red rgb_channels 0  " + imageCounter );
        if (debug >= 9)
            SaveImage(blue, imageTimeStamp + "-06 blue rgb_channels 2  " + imageCounter );

        if (colorDiff == null)
            colorDiff = new Mat(red.rows(), red.cols(), red.type());

        if (debug >= 9)
            SaveImage(red, imageTimeStamp + "-07 colorDiff " + imageCounter );

        Core.absdiff( red, blue, colorDiff );

        if (debug >= 9)
            SaveImage(colorDiff, imageTimeStamp + "-08 absdiff " + imageCounter );

        if(tmp1Img == null)
            tmp1Img = colorDiff.clone();
        else
            colorDiff.copyTo(tmp1Img);

        Imgproc.threshold( tmp1Img,  colorDiff, 20, 255, Imgproc.THRESH_BINARY );

        if (debug >= 9)
            SaveImage(colorDiff, imageTimeStamp + "-09 threshold " + imageCounter );

        if (debug >= 1) fileLogger.writeEvent(TAG, "findLum()");
        findLum();
        if (debug >= 1) fileLogger.writeEvent(TAG, "findBlue()");
        findBlue();
        if (debug >= 1) fileLogger.writeEvent(TAG, "findRed()");
        findRed();
        if (debug >= 1) fileLogger.writeEvent(TAG, "findBeaconBox()");
        findBeaconBox();
        if (debug >= 1) fileLogger.writeEvent(TAG, "findButtons()");
        findButtons(1);
        //createBeaconMask();
        if (debug >= 1) fileLogger.writeEvent(TAG, "draw()");
        finalImg = draw();
        if (debug >= 2)
            SaveImage(finalImg, imageTimeStamp + "-99 final " + imageCounter );
        calcPosition(beacMiddle);

        if (debug >= 1)
        {
            if (fileLogger != null)
            {
                fileLogger.writeEvent(TAG, "Stopped");
                fileLogger.close();
                fileLogger = null;
            }
        }

        return ObjectColourResult;
    }

    private void findLum()
    {
        Core.split( hsvImg, hsv_channels );

        Mat sat = hsv_channels.get( 1 );
        Mat lum = hsv_channels.get( 2 );

        if (debug >= 9)
            SaveImage(sat, imageTimeStamp + "-10 findLum sat " + imageCounter );
        if (debug >= 9)
            SaveImage(lum, imageTimeStamp + "-11 findLum lum " + imageCounter );

        lumAvg = Core.mean( lum ).val[0];

        //if(tmp1Img == null)
            tmp1Img = sat.clone();
        //else
        //    sat.copyTo(tmp1Img);

        Core.normalize( tmp1Img, sat, 120, 255, Core.NORM_MINMAX );
        lum.copyTo(tmp1Img);
        Core.normalize( tmp1Img, lum,   0, 180, Core.NORM_MINMAX );

        if (debug >= 9)
            SaveImage(sat, imageTimeStamp + "-12 findLum sat normalize " + imageCounter );
        if (debug >= 9)
            SaveImage(lum, imageTimeStamp + "-13 findLum lum normalize " + imageCounter );

        //if (white == null)
            white = lum.clone();

        Imgproc.GaussianBlur( lum, tmp1Img, new Size(25,25), 25);
        if (debug >= 9)
            SaveImage(tmp1Img, imageTimeStamp + "-14 findLum GaussianBlur " + imageCounter );

        tmp1Img.copyTo(btnTmpImg);

        Imgproc.threshold( tmp1Img, white, 255 - lumAvg, 255, Imgproc.THRESH_BINARY );
        if (debug >= 9)
            SaveImage(white, imageTimeStamp + "-15 findLum threshold " + imageCounter );
        //+ or Imgproc.THRESH_OTSU
        white.copyTo(tmp1Img);
        Imgproc.erode( tmp1Img, white, Imgproc.getGaussianKernel( 5, 2 ) );
        if (debug >= 9)
            SaveImage(white, imageTimeStamp + "-16 findLum erode " + imageCounter );

        findWeightedPos( white, white_blobs, white_matches, centroidWhite );

        hsv_channels.set( 1, sat );
        hsv_channels.set( 2, lum );

        //if (zonedImg == null)
            zonedImg = new Mat(hsvImg.rows(), hsvImg.cols(), hsvImg.type());

        Core.merge( hsv_channels, zonedImg );
        if (debug >= 9)
            SaveImage(zonedImg, imageTimeStamp + "-17 findLum merge " + imageCounter );

        if(tmpHsvImg == null)
            tmpHsvImg = zonedImg.clone();
        else
            zonedImg.copyTo(tmpHsvImg);

        List<Mat> tmp = new ArrayList<>();
        Core.split( hsvImg, tmp );

        //if(maskImg == null)
          maskImg = new Mat(hsvImg.rows(), hsvImg.cols(), hsvImg.type());

        //if(onesImg == null)
            onesImg = Mat.ones( hsvImg.rows(), hsvImg.cols(), white.type() );

        //if(zeroImg == null)
            zeroImg = Mat.zeros( hsvImg.rows(), hsvImg.cols(), white.type() );

        white.copyTo(tmp1Img);
        Imgproc.dilate( tmp1Img, white, Imgproc.getGaussianKernel( 5, 2 ) );
        if (debug >= 9)
            SaveImage(white, imageTimeStamp + "-18 findLum dilate " + imageCounter );

        tmp.set( 0, onesImg );
        tmp.set( 1, onesImg );
        tmp.set( 2, white );
        Core.merge( tmp, maskImg );
        if (debug >= 9)
            SaveImage(maskImg, imageTimeStamp + "-19 findLum merge " + imageCounter );

        Core.multiply( tmpHsvImg, maskImg, zonedImg );
        if (debug >= 9)
            SaveImage(zonedImg, imageTimeStamp + "-20 findLum multiply " + imageCounter );

        //get known rubbish areas from hashmap and process
        if (debug >= 3)
        {
            fileLogger.writeEvent(TAG, "Start Crap Filter");
        }
        // this process is 350ms
        for (int x = 0; x < loadHSVCrapindex; x++) {
            if (x == 0) {
                if (HSVCrapFilters.containsKey(String.valueOf(x))) {
                    processingHSV = HSVCrapFilters.get(String.valueOf(String.valueOf(x)));
                    Core.inRange(zonedImg, new Scalar(processingHSV.getmHLowerLimit(), processingHSV.getmSLowerLimit(), processingHSV.getmVLowerLimit()), new Scalar(processingHSV.getmHUpperLimit(), processingHSV.getmSUpperLimit(), processingHSV.getmVUpperLimit()), crap1);
                }
            } else {
                if (HSVCrapFilters.containsKey(String.valueOf(x))) {
                    processingHSV = HSVCrapFilters.get(String.valueOf(String.valueOf(x)));
                    Core.inRange(zonedImg, new Scalar(processingHSV.getmHLowerLimit(), processingHSV.getmSLowerLimit(), processingHSV.getmVLowerLimit()), new Scalar(processingHSV.getmHUpperLimit(), processingHSV.getmSUpperLimit(), processingHSV.getmVUpperLimit()), crap2);
                    Core.bitwise_or(crap1, crap2, crap1);
                }
            }
        }
        if (debug >= 3)
        {
            fileLogger.writeEvent(TAG, "Finish Crap Filter");
        }
        Imgproc.dilate(crap1, crap1, new Mat());
        Imgproc.dilate(crap1, crap1, new Mat());
        Imgproc.erode(crap1, crap1, new Mat());
        Imgproc.erode(crap1, crap1, new Mat());

        if (debug >= 9)
            SaveImage(crap1, imageTimeStamp + "-20.1 findLum crap inRange " + imageCounter);

        zonedImg.copyTo(showImg);
    }

    private void findBlue()
    {
        Mat blue_areas = new Mat();
        Mat blue1 = new Mat();

        if (debug < 10) {
            // Threshold based on color.  White regions match the desired color.  Black do not.
            // We now have a binary image to work with.  Contour detection looks for white blobs
            // from shelby robotics
//            Core.inRange(zonedImg, new Scalar( 105, 100, 100 ), new Scalar( 125, 255, 255 ), blue_areas );
//            Core.inRange(zonedImg, new Scalar( 40, 120, 160 ), new Scalar( 80, 190, 255 ), blue2);
//            if (debug >= 9)
//                SaveImage(blue1, imageTimeStamp + "-21 findBlue inRange1 " + imageCounter);
//            if (debug >= 9)
//                SaveImage(blue2, imageTimeStamp + "-22 findBlue inRange2 " + imageCounter);
//            Core.bitwise_or(blue_areas, blue2, blue_areas);

            if (debug >= 3)
            {
                fileLogger.writeEvent(TAG, "Start Blue Filter");
            }

            // this process is 300ms
            for (int x = 0; x < loadHSVBlueindex; x++) {
                if (x == 0) {
                    if (HSVBlueFilters.containsKey(String.valueOf(x))) {
                        processingHSV = HSVBlueFilters.get(String.valueOf(String.valueOf(x)));
                        Core.inRange(zonedImg, new Scalar(processingHSV.getmHLowerLimit(), processingHSV.getmSLowerLimit(), processingHSV.getmVLowerLimit()), new Scalar(processingHSV.getmHUpperLimit(), processingHSV.getmSUpperLimit(), processingHSV.getmVUpperLimit()), blue_areas);
                    }
                } else {
                    if (HSVBlueFilters.containsKey(String.valueOf(x))) {
                        processingHSV = HSVBlueFilters.get(String.valueOf(String.valueOf(x)));
                        Core.inRange(zonedImg, new Scalar(processingHSV.getmHLowerLimit(), processingHSV.getmSLowerLimit(), processingHSV.getmVLowerLimit()), new Scalar(processingHSV.getmHUpperLimit(), processingHSV.getmSUpperLimit(), processingHSV.getmVUpperLimit()), blue1);
                        Core.bitwise_or(blue_areas, blue1, blue_areas);
                    }
                }
            }
            if (debug >= 3)
            {
                fileLogger.writeEvent(TAG, "Finish Blue Filter");
            }

            if (debug >= 9)
                SaveImage(blue_areas, imageTimeStamp + "-23 findBlue bitwise_or " + imageCounter);

            //filter out the known crap
            Core.subtract(blue_areas, crap1, blue_areas);
            if (debug >= 9)
                SaveImage(blue_areas, imageTimeStamp + "-23.1 findBlue subtract " + imageCounter);

            blue_areas.copyTo(tmpHsvImg);

            //Core.multiply(tmpHsvImg, colorDiff, blue_areas);
            if (debug >= 9)
                SaveImage(blue_areas, imageTimeStamp + "-24 findBlue multiply " + imageCounter);
            blue_areas.copyTo(tmpHsvImg);
            Imgproc.dilate(tmpHsvImg, blue_areas, new Mat());
            //Imgproc.dilate(blue_areas, blue_areas, new Mat());
            if (debug >= 9)
                SaveImage(blue_areas, imageTimeStamp + "-25 findBlue dilate " + imageCounter);
            //Imgproc.erode(blue_areas, blue_areas, new Mat());
            //Imgproc.erode(blue_areas, blue_areas, new Mat());
            Imgproc.erode(blue_areas, blue_areas, new Mat());
            if (debug >= 9)
                SaveImage(blue_areas, imageTimeStamp + "-25.1 findBlue erode " + imageCounter);
        } else {
            //lets try find a value or values that work
            for (int x = 0; x < 265; x = x + 20) {
                for (int y = 0; y < 265; y = y + 20) {
                    for (int z = 0; z < 265; z = z + 20) {
                        Core.inRange(zonedImg, new Scalar(x, y, z), new Scalar(x + 20, y + 20, z + 20), blue_areas);
                        int n = Core.countNonZero(blue_areas);
                        if (n > 100)
                            SaveImage(blue_areas, imageTimeStamp + "-21 Seeking White Pixels = " + n + " - x " + x + " y " + y + " z " + z);
                        blue_areas.copyTo(tmpHsvImg);
                    }
                }
            }
        }

        // There can be several blobs.  Find the largest that fills a certain amount
        // of the image.  These are crude heuristics but should be fine if we control
        // the conditions of when we start searching (ie, appx size of beacon in image
        // frame, etc).
        findWeightedPos(blue_areas, blue_blobs, blue_matches, centroidBlue);
    }

    private void findRed()
    {
        // Same game, just a different hue
        Mat red1 = new Mat();
        Mat red_areas = new Mat();

        //Core.inRange( zonedImg, new Scalar( 0,100,150 ), new Scalar( 10,255,255 ), red1);
//        Core.inRange( zonedImg, new Scalar( 0,100,150 ), new Scalar( 20,200,200 ), red_areas);
//        if (debug >= 9)
//            SaveImage(red1, imageTimeStamp + "-26 findRed inRange1 " + imageCounter );
//        Core.inRange( zonedImg, new Scalar( 140,100,150 ), new Scalar( 179,255,255 ), red2);
//        if (debug >= 9)
//            SaveImage(red2, imageTimeStamp + "-27 findRed inRange2 " + imageCounter );
//        Core.bitwise_or(red_areas, red1, red_areas);
//        if (debug >= 9)
//            SaveImage(red_areas, imageTimeStamp + "-28 findRed bitwise_or " + imageCounter );

        if (debug >= 3)
        {
            fileLogger.writeEvent(TAG, "Start Red Filter");
        }

        // this process is 200ms
        for (int x = 0; x < loadHSVRedindex; x++) {
            if (x == 0) {
                if (HSVRedFilters.containsKey(String.valueOf(x))) {
                    processingHSV = HSVRedFilters.get(String.valueOf(String.valueOf(x)));
                    Core.inRange(zonedImg, new Scalar(processingHSV.getmHLowerLimit(), processingHSV.getmSLowerLimit(), processingHSV.getmVLowerLimit()), new Scalar(processingHSV.getmHUpperLimit(), processingHSV.getmSUpperLimit(), processingHSV.getmVUpperLimit()), red_areas);
                }
            } else {
                if (HSVRedFilters.containsKey(String.valueOf(x))) {
                    processingHSV = HSVRedFilters.get(String.valueOf(String.valueOf(x)));
                    Core.inRange(zonedImg, new Scalar(processingHSV.getmHLowerLimit(), processingHSV.getmSLowerLimit(), processingHSV.getmVLowerLimit()), new Scalar(processingHSV.getmHUpperLimit(), processingHSV.getmSUpperLimit(), processingHSV.getmVUpperLimit()), red1);
                    Core.bitwise_or(red_areas, red1, red_areas);
                }
            }
        }
        if (debug >= 3)
        {
            fileLogger.writeEvent(TAG, "Finish Red Filter");
        }

        //filter out the known crap
        Core.subtract(red_areas, crap1, red_areas);
        if (debug >= 9)
            SaveImage(red_areas, imageTimeStamp + "-28.1 findRed subtract " + imageCounter);

        red_areas.copyTo(tmpHsvImg);

        //Core.multiply( tmpHsvImg, colorDiff, red_areas );
        if (debug >= 9)
            SaveImage(red_areas, imageTimeStamp + "-29 findRed multiply " + imageCounter );
        red_areas.copyTo(tmpHsvImg);
        Imgproc.dilate( tmpHsvImg, red_areas, new Mat() );
        if (debug >= 9)
            SaveImage(red_areas, imageTimeStamp + "-30 findRed dilate " + imageCounter );
        findWeightedPos( red_areas, red_blobs, red_matches, centroidRed );
    }

    private void findBeaconBox()
    {
        if ( white_matches.size() == 0 )
        {
            white_box = new Rect( 0, 0, 1, 1 );
            blue_box = new Rect( 0, 0, 1, 1 );
            red_box = new Rect( 0, 0, 1, 1 );
            beacon_box = new Rect( 0, 0, 1, 1 );
        }
        else
        {
            double fs, maxFit = 0.0;
            Rect rb, bb;

            for ( Rect wb : white_matches )
            {
                rb = bestFit( wb, red_matches );
                bb = bestFit( wb, blue_matches );

                fs = scoreFit( wb, rb, bb );
                if ( fs > maxFit )
                {
                    maxFit = fs;

                    white_box = wb;
                    red_box = rb;
                    blue_box = bb;
                }
            }

            double tx = minOf(white_box.x, red_box.x, blue_box.x);
            double bx = maxOf(white_box.br().x, red_box.br().x, blue_box.br().x);
            double ty = minOf(white_box.y, red_box.y, blue_box.y);
            double by = maxOf(white_box.br().y, red_box.br().y, blue_box.br().y);

            beacon_box = new Rect( (int) tx, (int) ty, (int)(bx - tx), (int)(by - ty) );
        }
    }

    private void createBeaconMask(){
        //beaconMaskImg
        Point buttonsCenterLeft;
        Point buttonsCenterRight;
        Point point1 = new Point();
        Point point2 = new Point();
        Point center = new Point();
        int loop = 0;
        double pixelsperinchx;
        double pixelsperinchy;

        if (debug >= 2)
        {
            fileLogger.writeEvent(TAG, "createBeaconMask ");
            Log.d(TAG, "createBeaconMask ");
        }

        for ( Rect butn : buttons )
        {
            center = new Point();
            center.x = butn.tl().x + (butn.br().x - butn.tl().x) / 2;
            center.y = butn.br().y + (butn.tl().y - butn.br().y) / 2;

            if (loop == 0)
                point1 = center;
            else
                point2 = center;
            loop++;
            if (debug >= 2)
            {
                fileLogger.writeEvent(TAG, "Button Center x,y " + center.x + "," + center.y);
                Log.d(TAG, "Button Center x,y " + center.x + "," + center.y);
            }
        }

        if ((point1.x - point2.x) < 0) {
            buttonsCenterLeft = point1;
            buttonsCenterRight = point2;
        } else {
            buttonsCenterLeft = point2;
            buttonsCenterRight = point1;
        }
        if (debug >= 2)
        {
            fileLogger.writeEvent(TAG, "Button Left Center x,y " + buttonsCenterLeft.x + "," + buttonsCenterLeft.y);
            Log.d(TAG, "Button Left Center x,y " + buttonsCenterLeft.x + "," + buttonsCenterLeft.y);
            fileLogger.writeEvent(TAG, "Button Right Center x,y " + buttonsCenterRight.x + "," + buttonsCenterRight.y);
            Log.d(TAG, "Button Right Center x,y " + buttonsCenterRight.x + "," + buttonsCenterRight.y);
        }

        //calculate pixels per inch
        // distance between button is 5.3 inches
        pixelsperinchx = Math.abs(buttonsCenterRight.x - buttonsCenterLeft.x) / 5.3;  //5.3
        pixelsperinchy = pixelsperinchx * 1;
        int tx = (int)(buttonsCenterLeft.x - pixelsperinchx * 1.6);
        int ty = (int)(buttonsCenterLeft.y - pixelsperinchy * 3.7);
        int bx = (int)(buttonsCenterRight.x + pixelsperinchx * 1.6);
        int by = (int)(buttonsCenterRight.y + pixelsperinchy * 2.0);
        int ctx = (int)(buttonsCenterLeft.x + pixelsperinchx * 1.6);
        int cty = (int)(ty);
        int cbx = (int)(buttonsCenterRight.x - pixelsperinchx * 1.6);
        int cby = (int)(by);

        if (debug >= 2)
        {
            fileLogger.writeEvent(TAG, "Beacon tx,ty " + tx + "," + ty);
            Log.d(TAG, "Beacon x,y " + tx + "," + ty);
            fileLogger.writeEvent(TAG, "Beacon bx,by " + bx + "," + by);
            Log.d(TAG, "Beacon x,y " + bx + "," + by);
        }

        //mask everything out 1280x360
        //left rectangle
        Imgproc.rectangle( beaconMaskImg, new Point (0,0), new Point (tx,360), new Scalar(255,255,255), -1);
        //top rectangle
        Imgproc.rectangle( beaconMaskImg, new Point (0,0), new Point (1280,ty), new Scalar(255,255,255), -1);
        //bottom rectangle
        Imgproc.rectangle( beaconMaskImg, new Point (0,by), new Point (1280,360), new Scalar(255,255,255), -1);
        //right rectangle
        Imgproc.rectangle( beaconMaskImg, new Point (bx,0), new Point (1280,360), new Scalar(255,255,255), -1);
        //center rectangle
        Imgproc.rectangle( beaconMaskImg, new Point (ctx,cty), new Point (cbx,cby), new Scalar(255,255,255), -1);
        if (debug >= 9)
            SaveImage(beaconMaskImg, imageTimeStamp + "-50 BeaconMask");
    }

    private void findButtons(int depth)
    {
        buttons.clear();
        black_blobs.clear();

        //Core.split( zonedImg, hsv_channels );

        //Mat d_value = hsv_channels.get( 2 );
        //d_value.copyTo(tmp1Img);
        Mat d_value = new Mat();

        //Imgproc.threshold( tmp1Img, d_value, 40, 255, Imgproc.THRESH_BINARY_INV ); // + Imgproc.THRESH_OTSU );
        double multiplier = 1;
        switch (depth) {
            case 9: multiplier = 0.7;
                break;
            case 8: multiplier = 1.30;
                break;
            case 7: multiplier = 0.77;
                break;
            case 6: multiplier = 1.23;
                break;
            case 5: multiplier = 0.83;
                break;
            case 4: multiplier = 1.17;
                break;
            case 3: multiplier = 0.92;
                break;
            case 2: multiplier = 1.08;
                break;
            case 1: multiplier = 1;
                break;

        }

        if (debug >= 9)
            SaveImage(btnTmpImg, imageTimeStamp + "-32.0 findButtons tmp1Img " + imageCounter );

        Imgproc.threshold( btnTmpImg, d_value, 255 - (lumAvg * multiplier), 255, Imgproc.THRESH_BINARY ); // + Imgproc.THRESH_OTSU );

        if (debug >= 9)
            SaveImage(d_value, imageTimeStamp + "-32.1 findButtons threshold depth " + depth + ", value " + (lumAvg * multiplier) + ","  + imageCounter );

        Mat hchy = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();

        //Imgproc.findContours(d_value, contours, hchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(d_value, contours, hchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find max contour area
        Rect bounded_box;
        double maxWidth = (double) beacon_box.width * 0.2;
        double minWidth = (double) beacon_box.width * 0.015;
        Iterator<MatOfPoint> each = contours.iterator();
        while (each.hasNext()) {
            MatOfPoint wrapper = each.next();
            bounded_box = Imgproc.boundingRect(wrapper);

            if (debug >= 9) {
                //draw the bounded box on the temp image to see output?
                Imgproc.rectangle( d_value, bounded_box.tl(), bounded_box.br(), new Scalar(200,200,200), 3 );
            }

            // Reasonably sized
            if (bounded_box.width > minWidth &&
                    bounded_box.width < maxWidth &&
                    // Inside beacon box
                    bounded_box.x  > beacon_box.x &&
                    bounded_box.x + bounded_box.width < beacon_box.x + beacon_box.width &&
                    bounded_box.y > beacon_box.y &&
                    bounded_box.y + bounded_box.height < beacon_box.y + beacon_box.height ) {

                black_blobs.add( wrapper );
                buttons.add( bounded_box );

                Point center = massCenterMatOfPoint2f(wrapper);
                int numberCenteroidButtons = centroidButtons.size();
                Point bc = new Point();
                boolean addPoint = true;
                if (centroidButtons.size() > 0) {
                    for (int x = 0; x < numberCenteroidButtons; x++) {
                        //for (Point bc : centroidButtons) {
                        bc = centroidButtons.get(x);
                        if (((bc.x - 5) < center.x) && ((bc.x + 5) > center.x) && (((bc.y - 5) < center.y) && ((bc.y + 5) > center.y))) {
                            //point already exists don't add it
                            addPoint = false;
                        }
                        Imgproc.circle(d_value, bc, 15, new Scalar(200, 200, 200), 3);
                    }
                }
                if (addPoint) {
                    centroidButtons.add(center);
                    if (debug >= 2)
                    {
                        fileLogger.writeEvent(TAG, "Adding Button = " + center );
                        Log.d(TAG, "Adding Button = " + center );
                    }
                    Imgproc.circle(d_value, center, 15, new Scalar(200, 200, 200), 3);
                }
            }

            if (debug >= 9) {
                SaveImage(d_value, imageTimeStamp + "-32.5 draw bounding box " + depth  + ","  + imageCounter );
            }
        }
        if (debug >= 2)
        {
            fileLogger.writeEvent(TAG, "depth = " + depth + ", Buttons = " + buttons.size());
            Log.d(TAG, "depth = " + depth + ", Buttons = " + buttons.size());
        }
        if (( buttons.size() == 2 ) || (depth == 9)) {
            return;
        } else {
            findButtons(depth + 1);
        }
    }

    public Mat draw() {

        out = new Mat();

        //if (out == null)
        //    out = original.clone();
        original.copyTo(out);

        //Imgproc.cvtColor( showImg, out, Imgproc.COLOR_HSV2RGB, 4 );

        for ( Rect bb : blue_matches )
        {
            Imgproc.rectangle( out, bb.tl(), bb.br(), new Scalar(150,150,255), 3 );
        }

        for ( Rect rb : red_matches )
        {
            Imgproc.rectangle( out, rb.tl(), rb.br(), new Scalar(255,150,150), 3 );
        }

        for ( Point bc : centroidBlue )
        {
            Imgproc.circle(out, bc, 50, new Scalar(0, 0, 255), 5);
        }

        for ( Point rc : centroidRed )
        {
            Imgproc.circle(out, rc, 50, new Scalar(255, 0, 0), 5);
        }

        Imgproc.rectangle( out, beacon_box.tl(), beacon_box.br(), new Scalar(200,200,200), 3 );
        Imgproc.rectangle( out, blue_box.tl(), blue_box.br(), new Scalar(50,50,255), 3 );
        Imgproc.rectangle( out, red_box.tl(), red_box.br(), new Scalar(255,50,50), 3 );

        for ( Rect butn : buttons )
        {
            Imgproc.rectangle( out, butn.tl(), butn.br(), new Scalar(40,40,40), -1 );
        }

        Imgproc.drawContours( out, blue_blobs, -1, new Scalar(0,0,255), 2 );
        Imgproc.drawContours( out, red_blobs, -1, new Scalar(255,0,0), 2 );
        Imgproc.drawContours( out, white_blobs, -1, new Scalar(255,255,255), 2 );
        Imgproc.drawContours( out, black_blobs, -1, new Scalar(0,0,0), 2 );

        return out;
    }

    private void calcPosition(Point beacMiddle)
    {
        double beac_ctr;
        Point centroidRedPosition;
        Point centroidBluePosition;
        int loop = 0;

        if ( beacon_box.height == 0 || beacon_box.width == 0)
        {
            ObjectColourResult = Constants.ObjectColours.UNKNOWN;
            Log.d("beacon colour", "( beacon_box.height == 0 || beacon_box.width == 0) = colour unknow");
            return;
        }

        double scrn_ctr = hsvImg.cols() / 2;
        if ( buttons.size() != 2 )
        {
            beac_ctr = beacon_box.x + beacon_box.width / 2;
        }
        else
        {
            beac_ctr = Math.min(buttons.get(0).tl().x, buttons.get(1).tl().x) + ((buttons.get(0).tl().x + Math.abs((buttons.get(0).tl().x -  buttons.get(0).br().x) / 2)) - ((buttons.get(1).tl().x + Math.abs(buttons.get(1).tl().x -  buttons.get(1).br().x) / 2))) / 2;
        }
        Log.d("beacon center", "beac_ctr " + beac_ctr);

        if (( centroidBlue.size() == 0 ) || ( centroidRed.size() == 0 ))  {
            ObjectColourResult = Constants.ObjectColours.UNKNOWN;
            return;
        }
        centroidBluePosition = centroidBlue.get(0);
        centroidRedPosition = centroidRed.get(0);

        if (( red_box.width < 5 || red_box.height < 5 ) || ( blue_box.width < 5 || blue_box.height < 5 )) {
            //beaconColourResult = Constants.BeaconColours.UNKNOWN;
            //return;
        } else if ( centroidBluePosition.x  < centroidRedPosition.x ) {
            ObjectColourResult = Constants.ObjectColours.OBJECT_BLUE_RED;
            return;
        } else if ( centroidRedPosition.x < centroidBluePosition.x ) {
            ObjectColourResult = Constants.ObjectColours.OBJECT_RED_BLUE;
            return;
        }

        if (( red_box.width > 5 && red_box.height > 5 )) {
            if (centroidRedPosition.x < beacMiddle.x) {
                ObjectColourResult = Constants.ObjectColours.OBJECT_RED_LEFT;
                return;
            } else if (centroidRedPosition.x > beacMiddle.x) {
                ObjectColourResult = Constants.ObjectColours.OBJECT_RED_RIGHT;
                return;
            }
        } else if (( blue_box.width > 5 && blue_box.height > 5 )) {
            if (centroidBluePosition.x < beacMiddle.x) {
                ObjectColourResult = Constants.ObjectColours.OBJECT_BLUE_LEFT;
                return;
            } else if (centroidBluePosition.x > beacMiddle.x) {
                ObjectColourResult = Constants.ObjectColours.OBJECT_BLUE_RIGHT;
                return;
            }
        }

        if (( red_box.width < 5 || red_box.height < 5 ) || ( blue_box.width < 5 || blue_box.height < 5 )) {
            ObjectColourResult = Constants.ObjectColours.UNKNOWN;
            return;
        } else if (( beac_ctr > centroidBluePosition.x ) && ( beac_ctr < centroidRedPosition.x )) {
            ObjectColourResult = Constants.ObjectColours.OBJECT_BLUE_RED;
            return;
        } else if (( beac_ctr > centroidRedPosition.x ) && ( beac_ctr < centroidBluePosition.x )) {
            ObjectColourResult = Constants.ObjectColours.OBJECT_RED_BLUE;
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
            //Log.d("OPENCV","contoursLargestIndex Area  " + contourarea);
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

    public void SaveImage (Mat mat, String info) {
        Mat mIntermediateMat = new Mat();
        Mat mIntermediateMat2 = new Mat();

        mat.convertTo(mIntermediateMat2, CvType.CV_8UC4);
        if (mIntermediateMat2.channels() > 2)
            Imgproc.cvtColor(mIntermediateMat2, mIntermediateMat, Imgproc.COLOR_RGBA2BGR, 3);
        else
            mIntermediateMat = mIntermediateMat2;


        File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES);
        String filename = "ian" + info + ".png";
        File file = new File(path, filename);

        Boolean bool = null;
        filename = file.toString();
        bool = Imgcodecs.imwrite(filename, mIntermediateMat);

        if (bool == true)
            Log.d("filesave", "SUCCESS writing image to external storage");
        else
            Log.d("filesave", "Fail writing image to external storage");
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
            case "blue":
                HSVBlueFilters.put(String.valueOf(loadHSVBlueindex), new LibraryOCVHSVFilter(parm1, parm2, parm3, parm4, parm5, parm6));
                loadHSVBlueindex++;
                break;
            case "crap":
                HSVCrapFilters.put(String.valueOf(loadHSVCrapindex), new LibraryOCVHSVFilter(parm1, parm2, parm3, parm4, parm5, parm6));
                loadHSVCrapindex++;
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
                    if (debug >= 2)
                    {
                        fileLogger.writeEvent(TAG, "CSV Value " + row[0].trim() + "," + row[1].trim() + "," + row[2].trim() + "," + row[3].trim() + "," + row[4].trim() + "," + row[5].trim() + "," + row[6].trim());
                        Log.d(TAG, "CSV Value " + row[0].trim() + "," + row[1].trim() + "," + row[2].trim() + "," + row[3].trim() + "," + row[4].trim() + "," + row[5].trim() + "," + row[6].trim());
                    }
                    loadHSVFilter(row[0].trim(),Integer.parseInt(row[1].trim()),Integer.parseInt(row[2].trim()),Integer.parseInt(row[3].trim()),Integer.parseInt(row[4].trim()),Integer.parseInt(row[5].trim()),Integer.parseInt(row[6].trim()));
                }
            }
        } catch(IOException ex) {
            //throw new RuntimeException("Error in reading CSV file:" + ex);
            if (debug >= 1)
            {
                Log.d(TAG, "Error in reading HSV CSV file:" + ex);
            }
        }
    }


}
