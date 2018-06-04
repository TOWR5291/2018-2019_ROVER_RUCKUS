package club.towr5291.libraries;

/**
 * Created by ianhaden on 2/09/16.
 * Define a "PathSegment" object, used for building a path for the robot to follow.
 */

public class LibraryStateSegAutoRoverRuckus {

    private double mRobotTimeOut;        //how much time is allowed for the step to complete
    private String mRobotCommand;        //Command
    private double mRobotDistanceX;      //distance to move
    private double mRobotDistanceY;      //distance to move
    private double mRobotDirection;      //direction to move
    private boolean mGyroDrive;          //if enable use the fyro to maintain the direction
    private double mRobotSpeed;          //what angle to move in
    private boolean mRobotParallel;      //run in parallel with next step
    private boolean mRobotLastPos;       //run using last known position or current encoder position
    private double mRobotParm1;          //1st Parameter
    private double mRobotParm2;          //2nd Parameter
    private double mRobotParm3;          //3rd Parameter
    private double mRobotParm4;          //4th Parameter
    private double mRobotParm5;          //5rd Parameter
    private double mRobotParm6;          //6th Parameter

    // Constructor
    public LibraryStateSegAutoRoverRuckus(double timeout, String RobotCommand, double RobotDistanceX, double RobotDistanceY, double RobotDirection, double robotSpeed,boolean GyroDrive, boolean RobotParallel, boolean RobotLastPos, double RobotParm1, double RobotParm2, double RobotParm3, double RobotParm4, double RobotParm5, double RobotParm6)
    {
        mRobotTimeOut    = timeout;
        mRobotCommand    = RobotCommand;
        mRobotDistanceX  = RobotDistanceX;
        mRobotDistanceY  = RobotDistanceY;
        mRobotDirection  = RobotDirection;
        mRobotSpeed      = robotSpeed;
        mGyroDrive       = GyroDrive;
        mRobotParallel   = RobotParallel;
        mRobotLastPos    = RobotLastPos;
        mRobotParm1      = RobotParm1;
        mRobotParm2      = RobotParm2;
        mRobotParm3      = RobotParm3;
        mRobotParm4      = RobotParm4;
        mRobotParm5      = RobotParm5;
        mRobotParm6      = RobotParm6;
    }

    public void setmRobotTimeOut(double mRobotTimeOut)
    {
        this.mRobotTimeOut = mRobotTimeOut;
    }

    public void setmRobotCommand(String mRobotCommand)
    {
        this.mRobotCommand = mRobotCommand;
    }

    public void setmRobotDistanceX(double RobotDistanceX) {
        this.mRobotDistanceX = RobotDistanceX;
    }

    public void setmRobotDistanceY(double RobotDistanceY) {
        this.mRobotDistanceY = RobotDistanceY;
    }

    public void setmRobotDirection(double RobotDirection) {
        this.mRobotDirection = RobotDirection;
    }

    public void setmRobotParallel(boolean mRobotParallel)
    {
        this.mRobotParallel = mRobotParallel;
    }

    public void setmRobotLastPos(boolean mRobotLastPos)
    {
        this.mRobotLastPos = mRobotLastPos;
    }

    public void setmRobotParm1(double mRobotParm1)
    {
        this.mRobotParm1 = mRobotParm1;
    }

    public void setmRobotParm2(double mRobotParm2)
    {
        this.mRobotParm2 = mRobotParm2;
    }

    public void setmRobotParm3(double mRobotParm3)
    {
        this.mRobotParm3 = mRobotParm3;
    }

    public void setmRobotParm4(double mRobotParm4)
    {
        this.mRobotParm3 = mRobotParm4;
    }

    public void setmRobotParm5(double mRobotParm3)
    {
        this.mRobotParm3 = mRobotParm5;
    }

    public void setmRobotParm6(double mRobotParm4)
    {
        this.mRobotParm3 = mRobotParm6;
    }

    public void setmRobotSpeed(double mRobotSpeed)
    {
        this.mRobotSpeed = mRobotSpeed;
    }

    public void setmGyroDrive(boolean mGyroDrive) {
        this.mGyroDrive = mGyroDrive;
    }

    public boolean ismGyroDrive() {
        return mGyroDrive;
    }

    public double getmRobotTimeOut()
    {
        return mRobotTimeOut;
    }

    public String getmRobotCommand()
    {
        return mRobotCommand;
    }

    public double getmRobotDistanceX() {
        return mRobotDistanceX;
    }
    public double getmRobotDistanceY() {
        return mRobotDistanceY;
    }

    public double getmRobotDirection() {
        return mRobotDirection;
    }

    public boolean getmRobotParallel()
    {
        return mRobotParallel;
    }

    public boolean getmRobotLastPos()
    {
        return mRobotLastPos;
    }

    public double mRobotParm1()
    {
        return mRobotParm1;
    }

    public double mRobotParm2()
    {
        return mRobotParm2;
    }

    public double mRobotParm3()
    {
        return mRobotParm3;
    }

    public double mRobotParm4()
    {
        return mRobotParm4;
    }

    public double mRobotParm5()
    {
        return mRobotParm5;
    }

    public double mRobotParm6()
    {
        return mRobotParm6;
    }

    public double mRobotSpeed()
    {
        return mRobotSpeed;
    }

}
