package club.towr5291.libraries;

/**
 * Created by ianhaden on 2/09/16.
 * Define a "PathSegment" object, used for building a path for the robot to follow.
 */

public class LibraryStateSegAutoRoverRuckus {

    private int mStep;                   //step number
    private double mRobotTimeOut;        //how much time is allowed for the step to complete
    private String mRobotCommand;        //Command
    private double mRobotDistance;       //how far to move
    private boolean mRobotParallel;      //run in parallel with next step
    private boolean mRobotLastPos;       //run using last known position or current encoder position
    private double mRobotParm1;          //1st Parameter
    private double mRobotParm2;          //2nd Parameter
    private double mRobotParm3;          //3rd Parameter
    private double mRobotParm4;          //4th Parameter
    private double mRobotParm5;          //5rd Parameter
    private double mRobotParm6;          //6th Parameter
    private double mRobotSpeed;          //what angle to move in

    // Constructor
    // Constructor
    public LibraryStateSegAutoRoverRuckus(int step, double timeout, String RobotCommand, double RobotDistance, double robotSpeed, boolean RobotParallel, boolean RobotLastPos, double RobotParm1, double RobotParm2, double RobotParm3, double RobotParm4, double RobotParm5, double RobotParm6)
    {
        mStep = step;
        mRobotTimeOut = timeout;
        mRobotCommand = RobotCommand;
        mRobotDistance = RobotDistance;
        mRobotSpeed = robotSpeed;
        mRobotParallel = RobotParallel;
        mRobotLastPos = RobotLastPos;
        mRobotParm1 = RobotParm1;
        mRobotParm2 = RobotParm2;
        mRobotParm3 = RobotParm3;
        mRobotParm4 = RobotParm4;
        mRobotParm5 = RobotParm5;
        mRobotParm6 = RobotParm6;
    }

    public void setmRobotTimeOut(double mRobotTimeOut)
    {
        this.mRobotTimeOut = mRobotTimeOut;
    }

    public void setmRobotCommand(String mRobotCommand)
    {
        this.mRobotCommand = mRobotCommand;
    }

    public void setmRobotDistance(double mRobotDistance)
    {
        this.mRobotDistance = mRobotDistance;
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
        this.mRobotParm4 = mRobotParm4;
    }

    public void setmRobotParm5(double mRobotParm5)
    {
        this.mRobotParm5 = mRobotParm5;
    }

    public void setmRobotParm6(double mRobotParm6)
    {
        this.mRobotParm6 = mRobotParm6;
    }

    public void setmRobotSpeed(double mRobotSpeed)
    {
        this.mRobotSpeed = mRobotSpeed;
    }

    public double getmStep()
    {
        return mStep;
    }

    public double getmRobotTimeOut()
    {
        return mRobotTimeOut;
    }

    public String getmRobotCommand()
    {
        return mRobotCommand;
    }

    public double getmRobotDistance()
    {
        return this.mRobotDistance;
    }

    public boolean getmRobotParallel()
    {
        return mRobotParallel;
    }

    public boolean getmRobotLastPos()
    {
        return mRobotLastPos;
    }

    public double getmRobotParm1()
    {
        return mRobotParm1;
    }

    public double getmRobotParm2()
    {
        return mRobotParm2;
    }

    public double getmRobotParm3()
    {
        return mRobotParm3;
    }

    public double getmRobotParm4()
    {
        return mRobotParm4;
    }

    public double getmRobotParm5()
    {
        return mRobotParm5;
    }

    public double getmRobotParm6()
    {
        return mRobotParm6;
    }

    public double getmRobotSpeed()
    {
        return mRobotSpeed;
    }


}
