package club.towr5291.libraries;

/**
 * Created by ianhaden on 12/02/2017.
 */




public class LibraryOCVHSVFilter {


    private double mHLowerLimit;          //1st Parameter
    private double mSLowerLimit;          //2nd Parameter
    private double mVLowerLimit;          //3rd Parameter
    private double mHUpperLimit;          //4th Parameter
    private double mSUpperLimit;          //5rd Parameter
    private double mVUpperLimit;          //6th Parameter

    // Constructor
    public LibraryOCVHSVFilter(double HLowerLimit, double SLowerLimit, double VLowerLimit, double HUpperLimit, double SUpperLimit, double VUpperLimit)
    {

        mHLowerLimit = HLowerLimit;
        mSLowerLimit = SLowerLimit;
        mVLowerLimit = VLowerLimit;
        mHUpperLimit = HUpperLimit;
        mSUpperLimit = SUpperLimit;
        mVUpperLimit = VUpperLimit;
    }

    public double getmHLowerLimit() {
        return mHLowerLimit;
    }

    public void setmHLowerLimit(double mHLowerLimit) {
        this.mHLowerLimit = mHLowerLimit;
    }

    public double getmSLowerLimit() {
        return mSLowerLimit;
    }

    public void setmSLowerLimit(double mSLowerLimit) {
        this.mSLowerLimit = mSLowerLimit;
    }

    public double getmVLowerLimit() {
        return mVLowerLimit;
    }

    public void setmVLowerLimit(double mVLowerLimit) {
        this.mVLowerLimit = mVLowerLimit;
    }

    public double getmHUpperLimit() {
        return mHUpperLimit;
    }

    public void setmHUpperLimit(double mHUpperLimit) {
        this.mHUpperLimit = mHUpperLimit;
    }

    public double getmSUpperLimit() {
        return mSUpperLimit;
    }

    public void setmSUpperLimit(double mSUpperLimit) {
        this.mSUpperLimit = mSUpperLimit;
    }

    public double getmVUpperLimit() {
        return mVUpperLimit;
    }

    public void setmVUpperLimit(double mVUpperLimit) {
        this.mVUpperLimit = mVUpperLimit;
    }
}

