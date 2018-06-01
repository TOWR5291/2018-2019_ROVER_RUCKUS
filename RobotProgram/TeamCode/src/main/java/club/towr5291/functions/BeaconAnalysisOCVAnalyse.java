package club.towr5291.functions;

import android.os.Environment;
import android.util.Log;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import static org.opencv.imgproc.Imgproc.contourArea;

/**
 * Created by LZTDD0 on 11/7/2016.
 */

public class BeaconAnalysisOCVAnalyse {

    public BeaconAnalysisOCVAnalyse() {


    }

    public Constants.ObjectColours ObjectAnalysisOCV(Mat tmp, int count) {
        List<MatOfPoint> contoursRed = new ArrayList<MatOfPoint>();
        List<MatOfPoint> contoursBlue = new ArrayList<MatOfPoint>();
        List<MatOfInt> hullRed = new ArrayList<MatOfInt>();
        List<MatOfInt> hullBlue = new ArrayList<MatOfInt>();

        //Scalar RED_LOWER_BOUNDS_HSV = new Scalar((int) (300.0 / 360.0 * 255.0), (int) (0.090 * 255.0), (int) (0.500 * 255.0));
        //Scalar RED_UPPER_BOUNDS_HSV = new Scalar((int) (400.0 / 360.0 * 255.0), 255, 255);

        //Scalar RED_LOWER_BOUNDS_HSV = new Scalar((int) (220.0 / 360.0 * 255.0), (int) (0.090 * 255.0), (int) (0.500 * 255.0));
        //Scalar RED_UPPER_BOUNDS_HSV = new Scalar((int) (500.0 / 360.0 * 255.0), 255, 255);


        //on bgr
       // Scalar RED_LOWER_BOUNDS_HSV = new Scalar((int) (010.0 / 360.0 * 255.0), (int) (0.090 * 255.0), (int) (0.500 * 255.0));
       // Scalar RED_UPPER_BOUNDS_HSV = new Scalar((int) (300.0 / 360.0 * 255.0), 255, 255);

        //rgb to HSV
        Scalar RED_LOWER_BOUNDS_HSV = new Scalar(130,60,130);
        Scalar RED_UPPER_BOUNDS_HSV = new Scalar(180,150,255);

        //rgb to BGR
        //Scalar RED_LOWER_BOUNDS_HSV = new Scalar(105,150,120);
        //Scalar RED_UPPER_BOUNDS_HSV = new Scalar(180,225,255);


        //Scalar BLUE_LOWER_BOUNDS_HSV = new Scalar((int) (170.0 / 360.0 * 255.0), (int) (0.090 * 255.0), (int) (0.500 * 255.0));
        //Scalar BLUE_UPPER_BOUNDS_HSV = new Scalar((int) (270.0 / 360.0 * 255.0), 255, 255);
        //Scalar BLUE_LOWER_BOUNDS_HSV = new Scalar((int) (170.0 / 360.0 * 255.0), (int) (0.090 * 255.0), (int) (0.500 * 255.0));
        //Scalar BLUE_UPPER_BOUNDS_HSV = new Scalar((int) (250.0 / 360.0 * 255.0), 255, 255);

        Scalar BLUE_LOWER_BOUNDS_HSV = new Scalar(80,140,130);
        Scalar BLUE_UPPER_BOUNDS_HSV = new Scalar(150,255,255);

        //Mat mat1 = new Mat(720, 1280, CvType.CV_8UC4);
        //Mat mat2 = new Mat(720, 1280, CvType.CV_8UC4);
        //Mat mat3 = new Mat(720, 1280, CvType.CV_8UC4);
        //Mat mat4 = new Mat(720, 1280, CvType.CV_8UC4);
        //Mat mat5 = new Mat(720, 1280, CvType.CV_8UC4);
        //Mat mat6 = new Mat(720, 1280, CvType.CV_8UC4);
        //Mat mat7 = new Mat(720, 1280, CvType.CV_8UC4);
        //Mat mat8 = new Mat(720, 1280, CvType.CV_8UC4);
        //Mat mat9 = new Mat(720, 1280, CvType.CV_8UC4);

        Mat original = new Mat();
        Mat mat1 = new Mat();
        Mat mat2 = new Mat();
        Mat mat3 = new Mat();
        Mat mat4 = new Mat();
        Mat mat5 = new Mat();
        Mat mat6 = new Mat();
        Mat mat7 = new Mat();
        Mat mat8 = new Mat();
        Mat mat9 = new Mat();


        Mat mHierarchy = new Mat();

        Point centroidRed = new Point();
        Point centroidBlue = new Point();

        boolean noRed = false;
        boolean noBlue = false;

        double areaRed = 0;
        double areaBlue = 0;


        //cropping bounding box out of camera image
        //Mat cropped = new Mat(tmp, new Rect((int) x, (int) y, (int) width, (int) height));
        //at 24 inches crop is
        // image is 720, 1280
        //x
        //y
        //top
        //bottom is always half 640 the image height as camera is 12 inches abover ground (base of beacon)


        //Rect roi = new Rect(460, 110, 360, 250);  //perfect for just eacon if head on
        //Rect roi = new Rect(400, 110, 500, 250);  //good for 24 inches
        Rect roi = new Rect(0, 0, 1280, 360);
        Mat cropped = new Mat(tmp, roi);
        original = cropped.clone();

        //roi = new Rect(0, 0, tmp.width(), tmp.height()/2);

        SaveImage(cropped, "-raw" + count);
        //Log.d("OPENCV","tmp CV_8UC4 Height " + tmp.height() + " Width " + tmp.width());
        //Log.d("OPENCV","Channels " + tmp.channels());

        cropped.convertTo(mat1, CvType.CV_8UC4);
        //SaveImage(mat1, "-convertcv_8uc4");
        //Log.d("OPENCV","mat1 CV_8UC4 Height " + mat1.height() + " Width " + mat1.width());
        //Log.d("OPENCV","mat1 convertcv_8uc4 Channels " + mat1.channels());

        Imgproc.cvtColor(cropped, mat2, Imgproc.COLOR_RGB2HSV);
        //Imgproc.cvtColor(cropped, mat2, Imgproc.COLOR_RGBA2BGR);
        //SaveImage(mat2, "-COLOR_RGB2HSV_FULL");
        //Log.d("OPENCV","mat2 COLOR_RGB2HSV Height " + mat2.height() + " Width " + mat2.width());
        //Log.d("OPENCV","mat2 Channels " + mat2.channels());

        //Imgproc.cvtColor(tmp, mat6, Imgproc.COLOR_RGB2YCrCb);
        //SaveImage(mat6, "-COLOR_RGB2YCrCb");
        //Log.d("OPENCV","mat6 COLOR_RGB2HSV Height " + mat6.height() + " Width " + mat6.width());
        //Log.d("OPENCV","mat6 Channels " + mat6.channels());

        Mat erode = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Mat dilate = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(10, 10));

        int x = 0;
        int y = 0;

        for (x = 120; x < 250; x = x + 10) {
        for (y = 0; y < 300; y = y + 10) {

        Mat mat10 = cropped.clone();
        //analyse RED
        //Core.inRange(mat2, RED_LOWER_BOUNDS_HSV, RED_UPPER_BOUNDS_HSV, mat3);
        Core.inRange(mat2, new Scalar(x, y, 130),  new Scalar(x + 30, y + 30, 255), mat3);
        //Log.d("OPENCV","mat3 Red Height " + mat3.height() + " Width " + mat3.width());
        //SaveImage(mat3, "-red limits" + count);
        SaveImage(mat3, "-red limits" + count + " x " + x + " y " + y);

        Imgproc.GaussianBlur( mat3, mat8, new Size(0,0) , 3);
        Core.addWeighted(mat3, 1.5, mat8, -0.5, 0, mat3);

        //Imgproc.blur( mat3, mat3, new Size(3,3) );
        //SaveImage(mat3, "-red limits blurred " + count );
        SaveImage(mat3, "-red limits blurred " + count + " x " + x + " y " + y);


        //fill in any holes
        //Imgproc.dilate(mat3, mat3, new Mat());
        //Imgproc.dilate(mat3, mat3, new Mat());

        //Canny edge detection
        //Imgproc.Canny(mat3, mat9, 30, 90);  //was 20,100


        //Imgproc.erode(mat8, mat8, erode);
        //Imgproc.erode(mat8, mat8, erode);
        Imgproc.dilate(mat3, mat9, dilate);
        //Imgproc.dilate(mat7, mat9, dilate);
        //Imgproc.erode(mat8, mat8, erode);
        //Imgproc.erode(mat8, mat8, erode);

        //find and draw the contours
        Imgproc.findContours(mat9, contoursRed, mHierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        if (!(contoursRed.isEmpty())) {
            int largestRed = contoursLargestIndex(contoursRed);
            areaRed = contourArea(contoursRed.get(largestRed));
            if (areaRed > 10000) {

                Imgproc.drawContours(mat9, contoursRed, -1, new Scalar(255, 0, 0), 4);

                //Draw square around contour
                MatOfPoint2f approxCurveRed = new MatOfPoint2f();
                MatOfPoint2f contour2fRed = new MatOfPoint2f(contoursRed.get(largestRed).toArray());

                double approxDistanceRed = Imgproc.arcLength(contour2fRed, true) * 0.02;
                Imgproc.approxPolyDP(contour2fRed, approxCurveRed, approxDistanceRed, true);

                //Convert back to MatOfPoint
                MatOfPoint pointsRed = new MatOfPoint(approxCurveRed.toArray());

                // Get bounding rect of contour
                Rect rectRed = Imgproc.boundingRect(pointsRed);
                Imgproc.rectangle(original, new Point(rectRed.x, rectRed.y), new Point(rectRed.x + rectRed.width, rectRed.y + rectRed.height), new Scalar(255, 0, 0), 3);
                Imgproc.rectangle(mat10, new Point(rectRed.x, rectRed.y), new Point(rectRed.x + rectRed.width, rectRed.y + rectRed.height), new Scalar(255, 0, 0), 3);

                centroidRed = massCenterMatOfPoint2f(contoursRed.get(largestRed));
                Log.d("OPENCV", "Red Centroid " + centroidRed);
                Imgproc.circle(original, centroidRed, 50, new Scalar(255, 0, 0), 5);
                //Imgproc.putText(original, "RED Area " + areaRed, centroidRed, 3, 0.5, new Scalar(255, 0, 0), 1);
                Imgproc.circle(mat10, centroidRed, 50, new Scalar(255, 0, 0), 5);
                //Imgproc.putText(mat10, "RED ", centroidRed, 3, 0.5, new Scalar(255, 0, 0), 1);
                Imgproc.putText(mat10, "RED x " + x + " y " + y, centroidRed, 3, 0.5, new Scalar(255, 0, 0), 1);
                Log.d("OPENCV", "Area Red " + areaRed);

                SaveImage(mat9, "-red contours " + count + " x " + x + " y " + y );
                SaveImage(mat10, "-final RED " + count + " x " + x + " y " + y );
                //SaveImage(mat9, "-red contours " + count );
                //SaveImage(original, "-final RED " + count );
                mat9.release();

            }
        }

        }
        }

        for (x = 120; x < 250; x = x + 10) {
            for (y = 0; y < 300; y = y + 10) {



                    //Imgproc.cvtColor(cropped, mat2, Imgproc.COLOR_RGB2HSV);
        Mat mat10 = cropped.clone();

        //analyse blue
        //Core.inRange(mat2, BLUE_LOWER_BOUNDS_HSV, BLUE_UPPER_BOUNDS_HSV, mat4);
        Core.inRange(mat2, new Scalar(x, y, 130),  new Scalar(x + 30, y + 30, 255), mat4);
        //Log.d("OPENCV","mat4 Blue Height " + mat4.height() + " Width " + mat4.width());
        SaveImage(mat4, "-blue limits" + count + " x " + x + " y " + y );

        //empty out the current mats
        mat7.release();
        mat8.release();
        mat9.release();

        Imgproc.GaussianBlur( mat4, mat9, new Size(0,0) , 3);
        Core.addWeighted(mat4, 1.5, mat9, -0.5, 0, mat4);

        SaveImage(mat4, "-blue limits blurred " + count + " x " + x + " y " + y );

        //Imgproc.dilate(mat4, mat4, dilate);

        //Canny edge detection
        //Imgproc.Canny(mat4, mat9, 50, 100);

        //Imgproc.erode(mat4, mat8, erode);
        //Imgproc.erode(mat8, mat8, erode);
        //Imgproc.dilate(mat9, mat9, dilate);
        Imgproc.dilate(mat4, mat9, dilate);
        //Imgproc.erode(mat8, mat8, erode);
        //Imgproc.erode(mat8, mat8, erode);

        //find and draw the contours
        Imgproc.findContours(mat9, contoursBlue, mHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        if (!(contoursBlue.isEmpty())) {

            int largestBlue = contoursLargestIndex(contoursBlue);
            areaBlue = contourArea(contoursBlue.get(largestBlue));
            //only look for large areas
            if (areaBlue > 10000) {
                Imgproc.drawContours(mat9, contoursBlue, -1, new Scalar(0, 0, 255), 4);

                //Draw square around contour
                MatOfPoint2f approxCurveBlue = new MatOfPoint2f();
                MatOfPoint2f contour2fBlue = new MatOfPoint2f(contoursBlue.get(largestBlue).toArray());

                double approxDistanceBlue = Imgproc.arcLength(contour2fBlue, true) * 0.02;
                Imgproc.approxPolyDP(contour2fBlue, approxCurveBlue, approxDistanceBlue, true);

                //Convert back to MatOfPoint
                MatOfPoint pointsBlue = new MatOfPoint(approxCurveBlue.toArray());

                // Get bounding rect of contour
                Rect rectBlue = Imgproc.boundingRect(pointsBlue);
                Imgproc.rectangle(original, new Point(rectBlue.x, rectBlue.y), new Point(rectBlue.x + rectBlue.width, rectBlue.y + rectBlue.height), new Scalar(0, 0, 255), 3);
                Imgproc.rectangle(mat10, new Point(rectBlue.x, rectBlue.y), new Point(rectBlue.x + rectBlue.width, rectBlue.y + rectBlue.height), new Scalar(0, 0, 255), 3);

                centroidBlue = massCenterMatOfPoint2f(contoursBlue.get(largestBlue));
                Log.d("OPENCV", "Blue Centroid " + centroidBlue);
                Imgproc.circle(original, centroidBlue, 50, new Scalar(0, 0, 255), 5);
                Imgproc.circle(mat10, centroidBlue, 50, new Scalar(0, 0, 255), 5);
                Imgproc.putText(original, "BLUE Area " + areaBlue, centroidBlue, 3, 0.5, new Scalar(0, 0, 255), 1);
                Imgproc.putText(mat10, "BLUE x " + x + " y " + y, centroidBlue, 3, 0.5, new Scalar(0, 0, 255), 1);
                MatOfPoint lineBlue = contoursBlue.get(0);
                Log.d("OPENCV", "Area Blue " + areaBlue);
                SaveImage(mat9, "-blue contours" + count + " x " + x + " y " + y );
                SaveImage(mat10, "-final BLUE " + count + " x " + x + " y " + y );


            }
        } else {
            noBlue = true;
        }

        }
        }

        SaveImage(original, "-final" + count + " x " + x + " y " + y );

        //merge the two images together
        Core.bitwise_or(mat3, mat4, mat5);
        SaveImage(mat5, "-bitwise red and blue images" + count);

        // convert to bitmap:
        //Bitmap bmDisplay = Bitmap.createBitmap(mat5.cols(), mat5.rows(),Bitmap.Config.ARGB_8888);
        //Utils.matToBitmap(mat5, bmDisplay);

        //if both areas are similar, then we have both lights one
        if (((areaBlue > (areaRed * 0.3)) && (areaBlue < (areaRed * 2))) || ((areaRed > (areaBlue * .3)) && (areaRed < (areaBlue * 2)))) {
            if ((centroidRed.x) < (centroidBlue.x))
            {
                Log.d("OPENCV", "RED_BLUE");
                return Constants.ObjectColours.OBJECT_RED_BLUE;
            } else
            {
                Log.d("OPENCV", "BLUE_RED");
                return Constants.ObjectColours.OBJECT_BLUE_RED;
            }
        }

        if ((areaBlue) > (areaRed))
        {
            Log.d("OPENCV", "BLUE_BLUE");
            return Constants.ObjectColours.OBJECT_BLUE;
        }
        if ((areaRed) > (areaBlue))
        {
            Log.d("OPENCV", "RED_RED");
            return Constants.ObjectColours.OBJECT_RED;
        }
        Log.d("OPENCV", "UNKNOWN");
        return Constants.ObjectColours.UNKNOWN;
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
}
