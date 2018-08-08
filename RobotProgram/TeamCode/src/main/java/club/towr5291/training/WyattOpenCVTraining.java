package club.towr5291.training;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Scalar;

@TeleOp(name = "WyattOpenCVTraining")
@Disabled
public class WyattOpenCVTraining extends LinearOpMode{
    Scalar RED_LOW_BOUND_HSV = new Scalar(0, 100, 150);
    Scalar RED_HIGH_BOUND_HSV = new Scalar(22,255,255);

    Scalar BLUE_LOW_BOUND_HSV = new Scalar(150,100,100);
    Scalar BLUE_HIGH_BOUND_HSV = new Scalar(270,255,255);

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
