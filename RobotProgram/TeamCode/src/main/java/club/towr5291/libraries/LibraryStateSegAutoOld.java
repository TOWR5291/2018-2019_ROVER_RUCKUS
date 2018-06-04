package club.towr5291.libraries;

/**
 * Created by ianhaden on 2/09/16.
 * Define a "PathSegment" object, used for building a path for the robot to follow.
 */

public class LibraryStateSegAutoOld {

    public double mRobotTimeOut;        //how much time is allowed for the step to complete
    public double mRobotDistance;       //how far to move in inches
    public double mRobotSpeed;          //how fast to move -1 to 1
    public String mRobotDirection;      //what angle to move in

    // Constructor
    public LibraryStateSegAutoOld(double timeout, String robotDirection, double robotDistance, double robotSpeed)
    {
        mRobotTimeOut = timeout;
        mRobotDirection = robotDirection;
        mRobotDistance = robotDistance;
        mRobotSpeed = robotSpeed;
    }

    public double getmRobotTimeOut() {
        return mRobotTimeOut;
    }

    public double getmRobotDistance() {
        return mRobotDistance;
    }

    public double getmRobotSpeed() {
        return mRobotSpeed;
    }

    public String getmRobotDirection() {
        return mRobotDirection;
    }
}