package club.towr5291.libraries;

import android.graphics.Bitmap;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.Function;
import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import club.towr5291.functions.Constants;
import club.towr5291.functions.FileLogger;
import club.towr5291.libraries.LibraryVuforiaRoverRuckus;
import club.towr5291.libraries.TOWRDashBoard;

public class ImageCaptureOCV {

    private LibraryVuforiaRoverRuckus libraryVuforiaRoverRuckus;
    private Mat currentMat;
    private TOWRDashBoard dash;
    private FileLogger logger;
    private Image rgbImage;

    public interface OnImageCapture{
        void OnImageCaptureVoid(Mat mat);
    }

    public ImageCaptureOCV(){
        //Nothing in here yet
    }

    public void initImageCaptureOCV(LibraryVuforiaRoverRuckus libraryVuforiaRoverRuckus, TOWRDashBoard dashBoard, FileLogger fileLogger){
        this.libraryVuforiaRoverRuckus = libraryVuforiaRoverRuckus;
        this.currentMat = new Mat();
        this.dash = dashBoard;
        this.logger = fileLogger;
        this.rgbImage = null;
    }

    public void takeImage(OnImageCapture onImageCapture){

        try {
            VuforiaLocalizer.CloseableFrame frame = this.libraryVuforiaRoverRuckus.getVuforia().getFrameQueue().take(); //takes the frame at the head of the queue
            long numImages = frame.getNumImages();
            this.logger.writeEvent(3, "VISION", "Number of Images " + numImages);

            for (int i = 0; i < numImages; i++) {
                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                    this.rgbImage = frame.getImage(i);
                    break;
                }
            }

            /*rgb is now the Image object that we’ve used in the video*/
            Bitmap bm = Bitmap.createBitmap(this.rgbImage.getWidth(), this.rgbImage.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(this.rgbImage.getPixels());

            //put the image into a MAT for OpenCV
            this.currentMat = new Mat(this.rgbImage.getWidth(), this.rgbImage.getHeight(), CvType.CV_8UC4);
            Utils.bitmapToMat(bm, this.currentMat);
            //close the frame, prevents memory leaks and crashing
            frame.close();

        } catch (InterruptedException e) {
            dash.displayPrintf(1, "VUFORIA --- ERROR ERROR ERROR");
            dash.displayPrintf(2, "VUFORIA --- ERROR ERROR ERROR");
        }

        onImageCapture.OnImageCaptureVoid(currentMat);
    }
}