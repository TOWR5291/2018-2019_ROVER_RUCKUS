package club.towr5291.libraries;

/**
 * Created by LZTDD0 on 2/16/2017.
 */



public class LibraryStateTrack {


    private boolean mTrackRunning;      //is the current step running  False is no, True is Yes
    private boolean mTrackComplete;     //is the current step complete, False is no, True is Yes

    // Constructor
    public LibraryStateTrack(boolean Running, boolean Complete)
    {
        mTrackRunning = Running;
        mTrackComplete = Complete;
    }

    public boolean ismTrackRunning() {
        return mTrackRunning;
    }

    public void setmTrackRunning(boolean mTrackRunning) {
        this.mTrackRunning = mTrackRunning;
    }

    public boolean ismTrackComplete() {
        return mTrackComplete;
    }

    public void setmTrackComplete(boolean mTrackComplete) {
        this.mTrackComplete = mTrackComplete;
    }
}
