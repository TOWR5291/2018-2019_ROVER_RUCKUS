package club.towr5291.libraries;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.NonConst;

public class DeadReckoning extends Thread{
    //r is the rotation of the robot so it is 0-360
    //x is the line that runs between the field
    //y is the line that runs through the driver station
    //The boxes to the left and right are the driver station
    //          The line going down is the x
    // |--| |---|---| |--|
    // |  | |       | |  |
    // | -| |- -|- -| |- | This is the y axis
    // |  | |       | |  |
    // |--| |---|---| |--|

    private double RobotX, RobotY, RobotAngle;

    private DcMotor LeftMotor, RightMotor, sidewaysMotor;
    private LinearOpMode RobotLinearOpMode;

    private double dblLeftStartOffset, dblRightStartOffset, dblSidewaysStartOffset;
    private double dblLeftCurrentPosition, dblRightCurrentPosition, dblSidewaysCurrentPosition;
    private double dblCosForXMovement, dblSinForYMovement;
    private double dblLeftRightAverage;
    private double dblWheelDiameter = 2;
    private double dblWheelBaseIN = 16;

    private int intCountsPerREV = 600;
    private int intCountsPerInch = (int) (this.intCountsPerREV / (this.dblWheelDiameter * Math.PI));
    private int intCountsPerDegree = (int) (((Math.PI * this.dblWheelBaseIN) * this.intCountsPerInch) / 360);

    private String strRunningIn = "";

    public DeadReckoning(DcMotor leftMotor, DcMotor rightMotor, DcMotor sidewaysMotor, LinearOpMode linearOpMode){
        this.LeftMotor = leftMotor;
        this.RightMotor = rightMotor;
        this.sidewaysMotor = sidewaysMotor;

        this.RobotLinearOpMode = linearOpMode;

        this.dblLeftStartOffset = this.LeftMotor.getCurrentPosition();
        this.dblRightStartOffset = this.RightMotor.getCurrentPosition();
        this.dblSidewaysStartOffset = this.sidewaysMotor.getCurrentPosition();

        this.strRunningIn = this.RobotLinearOpMode.getClass().getCanonicalName();

        Log.i(this.strRunningIn, "End of Constructor for DeadReckoning");
    }

    public void run(){
        while (this.RobotLinearOpMode.opModeIsActive()) {
            this.dblLeftCurrentPosition = this.LeftMotor.getCurrentPosition() - this.dblLeftStartOffset;
            this.dblRightCurrentPosition = this.RightMotor.getCurrentPosition() - this.dblRightStartOffset;
            this.dblSidewaysCurrentPosition = this.sidewaysMotor.getCurrentPosition() - this.dblSidewaysStartOffset;
            this.dblLeftRightAverage = ((this.dblLeftCurrentPosition + this.dblRightCurrentPosition) / 2);
            
            //this.RobotAngle = 25; //However you get your robot's angle
            this.RobotAngle = (this.dblLeftCurrentPosition - this.dblRightCurrentPosition) / this.intCountsPerDegree;

            this.dblCosForXMovement = Math.cos(this.RobotAngle);
            this.dblSinForYMovement = Math.sin(this.RobotAngle);
            
            this.RobotX += (this.dblSidewaysCurrentPosition * this.dblCosForXMovement) - (this.dblLeftRightAverage * this.dblSinForYMovement);
            this.RobotY += (this.dblLeftRightAverage * this.dblCosForXMovement) + (this.dblSidewaysCurrentPosition * this.dblSinForYMovement);


        }
        Log.i(this.strRunningIn, "Ended run() for DeadReckoning");
    }

    public double getRobotX(){
        return this.RobotX;
    }

    public double getRobotY(){
        return this.RobotY;
    }

    public double getRobotAngle(){
        return this.RobotAngle;
    }
}