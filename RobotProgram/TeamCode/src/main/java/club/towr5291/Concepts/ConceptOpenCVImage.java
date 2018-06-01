package club.towr5291.Concepts;

import android.graphics.Bitmap;
import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;

import club.towr5291.functions.FileLogger;

import static org.opencv.imgproc.Imgproc.resize;


/**
 * Created by ianhaden on 4/10/2016.
 */

@Autonomous(name="OpenCV Images", group="5291Concept")
@Disabled
public class ConceptOpenCVImage extends LinearOpMode{

    //set up the variables for the logger
    private String startDate;
    private ElapsedTime runtime = new ElapsedTime();
    private FileLogger fileLogger;

    //set up openCV stuff

    private double contourarea;
    private double ContourAreaLast;
    private double redlength;
    private double bluelength;
    private double directionOfBeacon;
    private boolean beaconLeft;
    private double beaconLeftXPos;
/*

    static {
        if (!OpenCVLoader.initDebug()) {
            Log.d("OpenCV", "OpenCV Not Loaded");
        } else {
            Log.d("OpenCV", "OpenCV Loaded");

        }
    }

*/


    private Point redpoint = new Point(0,0);
    private Point bluepoint = new Point(0,0);

    @Override
    public void runOpMode() throws InterruptedException {

        //start the log
        startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());
        fileLogger = new FileLogger(runtime);
        fileLogger.open();
        fileLogger.write("Time,SysMS,Thread,Event,Desc");
        fileLogger.writeEvent("init()","Log Started");




        //set up openCV stuff
        Scalar RED_LOWER_BOUNDS_HSV = new Scalar(0,100,150);
        Scalar RED_UPPER_BOUNDS_HSV = new Scalar(22,255,255);  //was 30,255,255

        Scalar BLUE_LOWER_BOUNDS_HSV = new Scalar(150,100,100);
        Scalar BLUE_UPPER_BOUNDS_HSV = new Scalar(270,255,255);


        Mat mat1 = new Mat(720,1280, CvType.CV_8UC4);
        Mat mat2 = new Mat(720,1280, CvType.CV_8UC4);
        Mat mat3 = new Mat(720,1280, CvType.CV_8UC4);
        Mat mat4 = new Mat(720,1280, CvType.CV_8UC4);
        Mat mat5 = new Mat(720,1280, CvType.CV_8UC4);
        Mat mat6 = new Mat(720,1280, CvType.CV_8UC4);
        Mat mat7 = new Mat(720,1280, CvType.CV_8UC4);

        List<MatOfPoint> contoursRed  = new ArrayList<MatOfPoint>();
        List<MatOfPoint> contoursBlue = new ArrayList<MatOfPoint>();

        Mat houghlines = new Mat();
        Mat lines = new Mat();
        Mat mHierarchy = new Mat();

        MatOfPoint2f approxCurve = new MatOfPoint2f();

        waitForStart();

        //Mat tmp = new Mat();

         while (opModeIsActive()) {



            /*rgb is now the Image object that we’ve used in the video*/
            /*Log.d("OPENCV","Height " + rgb.getHeight() + " Width " + rgb.getWidth());

            Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(rgb.getPixels());
            //Mat tmp = OCVUtils.bitmapToMat(bm, CvType.CV_8UC4);
            Mat tmp = new Mat(rgb.getWidth(), rgb.getHeight(), CvType.CV_8UC4);
            Utils.bitmapToMat(bm, tmp);*/
             Mat tmp = loadImageFromFile("ian-master.jpg");

            SaveImage(tmp, "-raw");
            fileLogger.writeEvent("process()","Saved original file ");
            Log.d("OPENCV","CV_8UC4 Height " + tmp.height() + " Width " + tmp.width());
            Log.d("OPENCV","Channels " + tmp.channels());

            tmp.convertTo(mat1, CvType.CV_8UC4);
            Size size = new Size(640,480);

             resize(mat1,mat1,size);//resize image
             SaveImage(mat1, "-convertcv_8uc4");
             Log.d("OPENCV","CV_8UC4 Height " + mat1.height() + " Width " + mat1.width());
             fileLogger.writeEvent("process()","converted to cv_8uc4");
             Log.d("OPENCV","Channels " + mat1.channels());

             Imgproc.cvtColor(mat1, mat2, Imgproc.COLOR_RGB2HSV_FULL);
             SaveImage(mat2, "-COLOR_RGB2HSV_FULL");
             Log.d("OPENCV","COLOR_RGB2HSV Height " + mat2.height() + " Width " + mat2.width());
             Log.d("OPENCV","Channels " + mat2.channels());

             Core.inRange(mat2, RED_LOWER_BOUNDS_HSV, RED_UPPER_BOUNDS_HSV, mat3);
             Log.d("OPENCV","mat2 Channels " + mat2.channels() + " empty " + mat2.empty());
             Log.d("OPENCV","mat3 Channels " + mat3.channels() + " empty " + mat3.empty());
             //Core.inRange(mat2, new Scalar(0,100,150), new Scalar(22,255,255), mat3);
             fileLogger.writeEvent("process()","Set Red window Limits: ");
             SaveImage(mat3, "-red limits");


             Core.inRange(mat2, BLUE_LOWER_BOUNDS_HSV, BLUE_UPPER_BOUNDS_HSV, mat4);
            fileLogger.writeEvent("process()","Set Blue window Limits: ");
            SaveImage(mat4, "-blue limits");

            Core.bitwise_or(mat3, mat4, mat5);
            SaveImage(mat5, "-bitwise red and blue images");

            telemetry.update();
        }

        //stop the log
        if (fileLogger != null)
        {
            fileLogger.writeEvent("stop()","Stopped");
            fileLogger.close();
            fileLogger = null;
        }
    }


    public void SaveImage (Mat mat, String info) {
        Mat mIntermediateMat = new Mat();

        mat.convertTo(mIntermediateMat, CvType.CV_8UC3);
        //Imgproc.cvtColor(mat, mIntermediateMat, Imgproc.COLOR_RGBA2BGR, 3);

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

    }
}
