package club.towr5291.training;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.HINT;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


@TeleOp(name = "WyattOpenCVTraining")
@Disabled
public class WyattOpenCVTraining extends OpMode {
    VuforiaLocalizer vuforia;

    @Override
    public void init() {

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AW9Mt0r/////AAAAGZTQpkbS80ZAmcBkWIDLcy6CS9VhLDFoyKS2MgMoVfFc4dJZnfKCp8KkOAJoW9SBWFImTgCniDMEbhB4Pk1R+q1R5iWbeE9m6JhgwNT1kZOmpJohh59A8H8yEqkl0v7gL4LgRLVWH/GrOw+RVxCNrP2kYNyr9mAoCGxoU8pKZQ2XUPDclGl5xzt0y4yTElsZL+92X6tJ7uEOoqhyvoviORCWU3oPDUQX9ki7ZedBC0IWXZWu38Uw/XuIJNGvDw3YfX7zs1z6rfTvAh85jns5l2PZ4QQtknCCBd4ynRFkG+d0PIYKAdPDZ47gJ4jm5scg9cp3Am2lElbPQOuv0SnYLyqD2DPsnclhBQz3PVt3smr0";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);


    }

    @Override
    public void loop() {

    }

    private void hslThreshold(Mat input, double[] hue, double[] sat, double[] lum, Mat out) {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HLS);
        Core.inRange(out, new Scalar(hue[0], lum[0], sat[0]),
                new Scalar(hue[1], lum[1], sat[1]), out);
    }

}
