package club.towr5291.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;


/**
 * Created by ianhaden on 8/02/2017.
 */

public abstract class OpModeMasterLinear extends LinearOpMode{

    boolean initialized = false;
    private static OpModeMasterLinear instance = null;
    private String TAG = "OpMasterLinear";

    boolean isInitialized() {
        return initialized;
    }


    protected void initOpenCv()
    {
        BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(hardwareMap.appContext)
        {
            @Override
            public void onManagerConnected(int status)
            {
                switch (status) {
                    case LoaderCallbackInterface.SUCCESS:
                    {
                        initialized = true;
                        RobotLog.i(TAG, "OpenCV loaded successfully");
                    }
                    break;
                    default: {
                        super.onManagerConnected(status);
                    }
                    break;
                }
            }
        };


        if (!OpenCVLoader.initDebug())
        {
            RobotLog.d(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION, hardwareMap.appContext,  mLoaderCallback);
            initialized = false;
        } else
        {
            initialized = true;
            RobotLog.d(TAG, "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }

    public OpModeMasterLinear()
    {

        super();
        instance = this;
    }
}
