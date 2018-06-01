package club.towr5291.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.List;

import club.towr5291.astarpathfinder.A0Star;
import club.towr5291.astarpathfinder.AStarValue;
import club.towr5291.astarpathfinder.sixValues;
import club.towr5291.functions.FileLogger;
import club.towr5291.libraries.LibraryStateSegAutoOld;
import club.towr5291.robotconfig.HardwareDriveMotorsBaseConfig;
import club.towr5291.robotconfig.HardwareSensors;


/**
 * Created by ianhaden on 2/09/16.
 */

@Autonomous(name="Pushbot: Auto Drive AStar Path Finder", group="5291Test")
@Disabled
public class AutoDriveAStarPathFinder extends OpMode {
    /* Declare OpMode members. */
    HardwareDriveMotorsBaseConfig robotDrive   = new HardwareDriveMotorsBaseConfig();   // Use base drive hardware configuration

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder  Neverrest 20=140, Neverrest 40=280, Neverrest 60=420
    static final double     DRIVE_GEAR_REDUCTION    = 1.333 ;   // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     WHEEL_ACTUAL_FUDGE      = 1;        // Fine tuning amount
    static final double     COUNTS_PER_INCH         = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)) * WHEEL_ACTUAL_FUDGE ;
    static final double     ROBOT_TRACK             = 16.5;     //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
    static final double     COUNTS_PER_DEGREE       =  ((2 * 3.1415 * ROBOT_TRACK) * COUNTS_PER_INCH) / 360;

    HardwareSensors robotSensors   = new HardwareSensors();     // Use base drive hardware configuration

    //set up the variables for the logger
    private String startDate;
    private ElapsedTime runtime = new ElapsedTime();
    private FileLogger fileLogger;

    //define each state for the step.  Each step should go through some of the states below
    private enum stepState {
        STATE_INIT,
        STATE_START,
        STATE_RUNNING,
        STATE_PAUSE,
        STATE_COMPLETE,
        STATE_TIMEOUT,
        STATE_ERROR,
        STATE_FINISHED
    }

    private int mCurrentStep = 0;                               // Current State Machine State.
    private stepState  mCurrentStepState;                       // Current State Machine State.
    private stepState  mCurrentDriveState;                      // Current State Machine State.
    private stepState  mCurrentTurnState;                       // Current State Machine State.
    private LibraryStateSegAutoOld[] mStateSegAuto;
    double mStepTimeout;
    double mStepDistance;
    double mStepSpeed;
    String mRobotDirection;
    double mStepTurnL;
    double mStepTurnR;
    int mStepLeftTarget;
    int mStepRightTarget;
    boolean baseStepComplete = false;
    boolean armStepComplete = true;
    public boolean activated = false;
    public boolean startPointLoaded = false;

    // Loop cycle time stats variables
    public ElapsedTime  mRuntime = new ElapsedTime();           // Time into round.

    private ElapsedTime mStateTime = new ElapsedTime();         // Time into current state

    //this is the sequence the state machine will follow
    private LibraryStateSegAutoOld[] mRobotAutonomous = {
            //                        time, head, dist, powe
            //                        out   ing   ance  r
            //                         s    deg   inch   %
            new LibraryStateSegAutoOld ( 10,  "L90",  12,  0.5 ),
            new LibraryStateSegAutoOld ( 10,  "R90",  12,  0.5 )

    };

    public static final int FIELDWIDTH = 144;
    public static final int FIELDLENGTH = 144;
    boolean[][] fieldOpen = new boolean[FIELDWIDTH][FIELDLENGTH];
    boolean[][] fieldObstacles = new boolean[FIELDWIDTH][FIELDLENGTH];
    boolean[][] gValues = new boolean[FIELDWIDTH][FIELDLENGTH];    // Movement Cost
    boolean[][] hValues = new boolean[FIELDWIDTH][FIELDLENGTH];    // Heuristic
    boolean[][] fValues = new boolean[FIELDWIDTH][FIELDLENGTH];    // this is hValues + gValues
    boolean[][] parentValues = new boolean[FIELDWIDTH][FIELDLENGTH];    // parents
    boolean[] closedList = new boolean[FIELDWIDTH * FIELDLENGTH];  // closed list.  List of nodes already checked
    boolean[] openList = new boolean[FIELDWIDTH * FIELDLENGTH];    // open list.  List of nodes to be checked

    AStarValue AStarValues = new AStarValue();
    AStarValue AStarClosed = new AStarValue();

    public HashMap<String,AStarValue> AStarValueMap = new HashMap<String,AStarValue>();

    public sixValues[] pathValues = new sixValues[1000];
    public int pathIndex = 0;

    public HashMap<String,AStarValue> AStarClosedMap = new HashMap<String,AStarValue>();

    A0Star a0Star = new A0Star();

    public String fieldOutput;

        //set up Vuforia
    public static final String TAG = "Vison OPMODE";
    OpenGLMatrix lastLocation = null;
    public VuforiaLocalizer vuforia;                                   // {@link #vuforia} is the variable we will use to store our instance of the Vuforia localization engine.
    public List<VuforiaTrackable> allTrackables;

    public float mmPerInch        = 25.4f;
    public float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
    public float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

    /*
    * Code to run ONCE when the driver hits INIT
    */
    @Override
    public void init() {
        int loopColumn;
        int loopRow;


        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());
        runtime.reset();
        telemetry.addData("FileLogger: ", runtime.toString());
        fileLogger = new FileLogger(runtime);
        fileLogger.open();
        telemetry.addData("FileLogger Op Out File: ", fileLogger.getFilename());
        fileLogger.write("Time,SysMS,Thread,Event,Desc");
        fileLogger.writeEvent("init()","Log Started");

        robotDrive.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.update();
        robotDrive.leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotDrive.rightMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mCurrentStepState = stepState.STATE_INIT;
        mCurrentTurnState = stepState.STATE_INIT;
        mCurrentDriveState = stepState.STATE_INIT;

        robotDrive.leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotDrive.rightMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        for ( int i = 0; i < pathValues.length; i++) {
            pathValues[i] = new sixValues();
        }

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());
        runtime.reset();
        telemetry.addData("FileLogger: ", runtime.toString());
        fileLogger = new FileLogger(runtime);
        fileLogger.open();
        telemetry.addData("FileLogger Op Out File: ", fileLogger.getFilename());
        fileLogger.write("Time,SysMS,Thread,Event,Desc");
        fileLogger.writeEvent("init()","Log Started");

        // Send telemetry message to signify robot waiting;
        telemetry.update();

        mCurrentStepState = stepState.STATE_INIT;
        mCurrentTurnState = stepState.STATE_INIT;
        mCurrentDriveState = stepState.STATE_INIT;



        fileLogger.writeEvent("init()","Init Complete");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        fileLogger.writeEvent("start()","START PRESSED: ");


    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if (activated == false)
        {
            //init the Veforia
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
            parameters.vuforiaLicenseKey = "AVATY7T/////AAAAGQJxfNYzLUgGjSx0aOEU0Q0rpcfZO2h2sY1MhUZUr+Bu6RgoUMUP/nERGmD87ybv1/lM2LBFDxcBGRHkXvxtkHel4XEUCsNHFTGWYcVkMIZqctQsIrTe13MnUvSOfQj8ig7xw3iULcwDpY+xAftW61dKTJ0IAOCxx2F0QjJWqRJBxrEUR/DfQi4LyrgnciNMXCiZ8KFyBdC63XMYkQj2joTN579+2u5f8aSCe8jkAFnBLcB1slyaU9lhnlTEMcFjwrLBcWoYIFAZluvFT0LpqZRlS1/XYf45QBSJztFKHIsj1rbCgotAE36novnAQBs74ewnWsJifokJGOYWdFJveWzn3GE9OEH23Y5l7kFDu4wc";
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
            this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

            VuforiaTrackables velocityVortex = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");
            VuforiaTrackable wheels = velocityVortex.get(0);
            VuforiaTrackable tools = velocityVortex.get(1);
            VuforiaTrackable legos = velocityVortex.get(2);
            VuforiaTrackable gears = velocityVortex.get(3);

            wheels.setName("wheels");  // wheels target
            tools.setName("tools");  // tools target
            legos.setName("legos");  // legos target
            gears.setName("gears");  // gears target

            /** For convenience, gather together all the trackable objects in one easily-iterable collection */
            allTrackables = new ArrayList<VuforiaTrackable>();
            allTrackables.addAll(velocityVortex);

            // RED Targets
            // To Place GEARS Target
            // position is approximately - (-6feet, -1feet)

            OpenGLMatrix gearsTargetLocationOnField = OpenGLMatrix
                    /* Then we translate the target off to the RED WALL. Our translation here
                    is a negative translation in X.*/
                    .translation(-mmFTCFieldWidth / 2, -1 * 12 * mmPerInch, 0)
                    .multiplied(Orientation.getRotationMatrix(
                            /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                            AxesReference.EXTRINSIC, AxesOrder.XZX,
                            AngleUnit.DEGREES, 90, 90, 0));
            gears.setLocation(gearsTargetLocationOnField);
            RobotLog.ii(TAG, "Gears Target=%s", format(gearsTargetLocationOnField));
            fileLogger.writeEvent("init()", "Tools Target=%s" + format(gearsTargetLocationOnField));

            // To Place GEARS Target
            // position is approximately - (-6feet, 3feet)
            OpenGLMatrix toolsTargetLocationOnField = OpenGLMatrix
                    /* Then we translate the target off to the Blue Audience wall.
                    Our translation here is a positive translation in Y.*/
                    .translation(-mmFTCFieldWidth / 2, 3 * 12 * mmPerInch, 0)
                    //.translation(0, mmFTCFieldWidth/2, 0)
                    .multiplied(Orientation.getRotationMatrix(
                            /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                            AxesReference.EXTRINSIC, AxesOrder.XZX,
                            AngleUnit.DEGREES, 90, 90, 0));
            tools.setLocation(toolsTargetLocationOnField);
            RobotLog.ii(TAG, "Tools Target=%s", format(toolsTargetLocationOnField));
            fileLogger.writeEvent("init()", "Tools Target=%s" + format(toolsTargetLocationOnField));

            //Finsih RED Targets

            // BLUE Targets
            // To Place LEGOS Target
            // position is approximately - (-6feet, -1feet)

            OpenGLMatrix legosTargetLocationOnField = OpenGLMatrix
                    /* Then we translate the target off to the RED WALL. Our translation here
                    is a negative translation in X.*/
                    .translation(-3 * 12 * mmPerInch, mmFTCFieldWidth / 2, 0)
                    .multiplied(Orientation.getRotationMatrix(
                             /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                            AxesReference.EXTRINSIC, AxesOrder.XZX,
                            AngleUnit.DEGREES, 90, 0, 0));
            legos.setLocation(legosTargetLocationOnField);
            RobotLog.ii(TAG, "Gears Target=%s", format(legosTargetLocationOnField));
            fileLogger.writeEvent("init()", "Tools Target=%s" + format(legosTargetLocationOnField));

            // To Place WHEELS Target
            // position is approximately - (-6feet, 3feet)
            OpenGLMatrix wheelsTargetLocationOnField = OpenGLMatrix
                    /* Then we translate the target off to the Blue Audience wall.
                    Our translation here is a positive translation in Y.*/
                    .translation(1 * 12 * mmPerInch, mmFTCFieldWidth / 2, 0)
                    .multiplied(Orientation.getRotationMatrix(
                            /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                            AxesReference.EXTRINSIC, AxesOrder.XZX,
                            AngleUnit.DEGREES, 90, 0, 0));
            wheels.setLocation(wheelsTargetLocationOnField);
            RobotLog.ii(TAG, "Tools Target=%s", format(wheelsTargetLocationOnField));
            fileLogger.writeEvent("init()", "Tools Target=%s" + format(wheelsTargetLocationOnField));

            //Finsih BLUE Targets

            //set up phone location
            OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                    .translation(mmBotWidth / 2, 0, 0)
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.YZY,
                            AngleUnit.DEGREES, -90, 0, 0));
            RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));
            fileLogger.writeEvent("init()", "Phone Location=%s" + format(phoneLocationOnRobot));
            //finish phone location

            //trackable listeners
            ((VuforiaTrackableDefaultListener) gears.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
            ((VuforiaTrackableDefaultListener) tools.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
            ((VuforiaTrackableDefaultListener) legos.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
            ((VuforiaTrackableDefaultListener) wheels.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
            /** Start tracking the data sets we care about. */
            velocityVortex.activate();
            activated = true;
        }

        if (lastLocation != null) {
            if (startPointLoaded) {
                //load start point
                int startX = 122;
                int startY = 122;
                int endX = 12;
                int endY = 80;
                AStarValues.xvalue = startX;
                AStarValues.yvalue = startY;
                AStarValues.zvalue = 0;
                AStarValues.GValue = 0;
                AStarValues.HValue = 0;
                AStarValues.FValue = 0;
                AStarValues.Parent = 0;
                AStarValues.ID = getKey(AStarValues.xvalue, AStarValues.yvalue, a0Star.fieldWidth, a0Star.fieldLength);
                AStarValueMap.put(String.valueOf(AStarValues.ID), new AStarValue(AStarValues.ID, AStarValues.FValue, AStarValues.GValue, AStarValues.HValue, AStarValues.Parent, AStarValues.xvalue, AStarValues.yvalue,  AStarValues.zvalue));

                fileLogger.writeEvent("loop()", "loaded astargvalue into hashmap");
                startPointLoaded = true;
            }
        }


        for (VuforiaTrackable trackable : allTrackables) {
            /**
             * getUpdatedRobotLocation() will return null if no new information is available since
             * the last time that call was made, or if the trackable is not currently visible.
             * getRobotLocation() will return null if the trackable is not currently visible.
             */
            telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
            }
        }
        if (lastLocation != null) {
            //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
            telemetry.addData("Pos", format(lastLocation));
            fileLogger.writeEvent("loop()", "Last Location :- " + format(lastLocation));
        } else {
            telemetry.addData("Pos", "Unknown");
            fileLogger.writeEvent("loop()", "Unknown");
        }

        fileLogger.writeEvent("loop()", "STATE: " + String.format("%4.1f ", mStateTime.time()) + " Current Step:- " + mCurrentStep + " Current Step State:- " + mCurrentStepState.toString() + " Current Turn State:- " + mCurrentTurnState.toString() + " Current Drive State:- " + mCurrentDriveState.toString());
        // Send the current state info (state and time) back to first line of driver station telemetry.
        telemetry.addData("STATE", String.format("%4.1f ", mStateTime.time()) + " Current Step:- " + mCurrentStep + " Current Step State:- " + mCurrentStepState.toString());
        // Execute the current state.  Each STATE's case code does the following:

        switch (mCurrentStepState) {
            case STATE_INIT: {
                initStep(mRobotAutonomous);
            }
            break;
            case STATE_START: {

            }
            break;
            case STATE_RUNNING: {
                runningTurnStep();
                runningDriveStep();
                if ((mCurrentDriveState == stepState.STATE_COMPLETE) && (mCurrentTurnState == stepState.STATE_COMPLETE) && (armStepComplete)) {
                    //  Transition to a new state.
                    mCurrentStepState = stepState.STATE_COMPLETE;
                }
            }
            break;
            case STATE_PAUSE: {

            }
            break;
            case STATE_COMPLETE: {
                fileLogger.writeEvent("loop()", "Current Step:- " + mCurrentStep + ", Array Size: " + mRobotAutonomous.length);
                if ((mCurrentStep) < (mRobotAutonomous.length - 1)) {
                    fileLogger.writeEvent("loop()", "Current Step:- " + mCurrentStep + ", Array Size: " + mRobotAutonomous.length);
                    //  Transition to a new state and next step.
                    mCurrentStep++;
                    mCurrentStepState = stepState.STATE_INIT;

                } else {
                    fileLogger.writeEvent("loop()", "STATE_COMPLETE - Setting FINISHED ");
                    //  Transition to a new state.
                    mCurrentStepState = stepState.STATE_FINISHED;
                }
            }
            break;
            case STATE_TIMEOUT: {
                setDriveMotorPower(Math.abs(0));
                //  Transition to a new state.
                mCurrentStepState = stepState.STATE_FINISHED;

            }
            break;
            case STATE_ERROR: {
                telemetry.addData("STATE", "ERROR WAITING TO FINISH " + mCurrentStep);
            }
            break;
            case STATE_FINISHED: {
                telemetry.addData("STATE", "FINISHED " + mCurrentStep);
            }
            break;

        }

        //check timeout vale
        if ((mStateTime.seconds() > mStepTimeout) && ((mCurrentStepState != stepState.STATE_ERROR) && (mCurrentStepState != stepState.STATE_FINISHED))) {
            //  Transition to a new state.
            mCurrentStepState = stepState.STATE_TIMEOUT;
        }

        telemetry.update();
    }


     /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop()
    {
        telemetry.addData("FileLogger Op Stop: ", runtime.toString());
        if (fileLogger != null) {
            fileLogger.writeEvent("stop()","Stopped");
            fileLogger.close();
            fileLogger = null;
        }
    }


    //--------------------------------------------------------------------------
    // User Defined Utility functions here....
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    //  Initialise the state.
    //--------------------------------------------------------------------------
    public void initStep (LibraryStateSegAutoOld[] step)
    {

        // Reset the state time, and then change to next state.
        baseStepComplete = false;
        mStateTime.reset();
        mStateSegAuto = step;
        mStepTimeout = mStateSegAuto[mCurrentStep].mRobotTimeOut;
        mStepDistance =  mStateSegAuto[mCurrentStep].mRobotDistance;
        mStepSpeed = mStateSegAuto[mCurrentStep].mRobotSpeed;
        mRobotDirection = mStateSegAuto[mCurrentStep].mRobotDirection;

        mCurrentTurnState = stepState.STATE_INIT;
        mCurrentDriveState = stepState.STATE_INIT;

//        fileLogger.writeEvent("initStep()","mRobotDirection.substring(0, 0)    :- " + mRobotDirection.substring(0, 0)  );
//        fileLogger.writeEvent("initStep()","mRobotDirection.substring(0, 1)    :- " + mRobotDirection.substring(0, 1)  );
//        fileLogger.writeEvent("initStep()","mRobotDirection.substring(0, 2)    :- " + mRobotDirection.substring(0, 2)  );
//        fileLogger.writeEvent("initStep()","mRobotDirection.substring(0, 3)    :- " + mRobotDirection.substring(0, 3)  );
//        fileLogger.writeEvent("initStep()","mRobotDirection.substring(1)       :- " + mRobotDirection.substring(1)  );

        if (mRobotDirection.substring(0, 1).equals("L")) {
            mStepTurnL = Double.parseDouble(mRobotDirection.substring(1));
            mStepTurnR = 0;
        } else if (mRobotDirection.substring(0, 1).equals("R" )) {
            mStepTurnL = 0;
            mStepTurnR = Double.parseDouble(mRobotDirection.substring(1));
        } else {
            mStepTurnL = 0;
            mStepTurnR = 0;
            mCurrentTurnState = stepState.STATE_COMPLETE;
        }

        mCurrentStepState = stepState.STATE_RUNNING;

//        fileLogger.writeEvent("initStep()","Current Step    :- " + mCurrentStep  );
//        fileLogger.writeEvent("initStep()","mStepTimeout    :- " + mStepTimeout  );
//        fileLogger.writeEvent("initStep()","mStepDistance   :- " + mStepDistance  );
//        fileLogger.writeEvent("initStep()","mStepSpeed      :- " + mStepSpeed  );
//        fileLogger.writeEvent("initStep()","mRobotDirection :- " + mRobotDirection  );
//        fileLogger.writeEvent("initStep()","mStepTurnL      :- " + mStepTurnL  );
//        fileLogger.writeEvent("initStep()","mStepTurnR      :- " + mStepTurnR  );

    }

    //--------------------------------------------------------------------------
    //  Execute the state.
    //--------------------------------------------------------------------------
    public void runningTurnStepPivot ()
    {

        switch (mCurrentTurnState) {
            case STATE_INIT: {
                fileLogger.writeEvent("runningTurnStep()","mStepTurnL      :- " + mStepTurnL  );
                fileLogger.writeEvent("runningTurnStep()","mStepTurnR      :- " + mStepTurnR  );

                // Turn On RUN_TO_POSITION
                if(mStepTurnR == 0) {
                    // Determine new target position
                    fileLogger.writeEvent("runningTurnStep()","Current LPosition:-" + robotDrive.leftMotor1.getCurrentPosition());
                    mStepLeftTarget = robotDrive.leftMotor1.getCurrentPosition() + (int) (mStepTurnL * COUNTS_PER_DEGREE);
                    mStepRightTarget = robotDrive.rightMotor1.getCurrentPosition() + (int) (mStepTurnR * COUNTS_PER_DEGREE);
                    fileLogger.writeEvent("runningTurnStep()","mStepLeftTarget:-  " + mStepLeftTarget);
                    // pass target position to motor controller
                    robotDrive.leftMotor1.setTargetPosition(mStepLeftTarget);
                    // set motor controller to mode
                    robotDrive.leftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    // set power on motor controller to start moving
                    setDriveLeftMotorPower(Math.abs(.5));
                }
                else {
                    // Determine new target position
                    fileLogger.writeEvent("runningTurnStep()","Current RPosition:-" + robotDrive.rightMotor1.getCurrentPosition());
                    mStepLeftTarget = robotDrive.leftMotor1.getCurrentPosition() + (int) (mStepTurnL * COUNTS_PER_DEGREE);
                    mStepRightTarget = robotDrive.rightMotor1.getCurrentPosition() + (int) (mStepTurnR * COUNTS_PER_DEGREE);
                    fileLogger.writeEvent("runningTurnStep()","mStepRightTarget:- " + mStepRightTarget);
                    // pass target position to motor controller
                    robotDrive.rightMotor1.setTargetPosition(mStepRightTarget);
                    // set motor controller to mode, Turn On RUN_TO_POSITION
                    robotDrive.rightMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    // set power on motor controller to start moving
                    setDriveRightMotorPower(Math.abs(.5));
                }

                fileLogger.writeEvent("runningTurnStep()","mStepLeftTarget :- " + mStepLeftTarget  );
                fileLogger.writeEvent("runningTurnStep()","mStepRightTarget:- " + mStepRightTarget  );

                mCurrentTurnState = stepState.STATE_RUNNING;
            }
            break;
            case STATE_RUNNING: {
                //if (robotDrive.leftMotor.isBusy() || robotDrive.rightMotor.isBusy()) {
                fileLogger.writeEvent("runningTurnStep()","Running         " );
                fileLogger.writeEvent("runningTurnStep()","Current LPosition:-" + robotDrive.leftMotor1.getCurrentPosition() + " LTarget:- " + mStepLeftTarget);
                fileLogger.writeEvent("runningTurnStep()","Current RPosition:-" + robotDrive.rightMotor1.getCurrentPosition() + " RTarget:- " + mStepRightTarget);
                if (mStepTurnR == 0) {
                    fileLogger.writeEvent("runningTurnStep()","Running         " );
                    telemetry.addData("Path1", "Running to %7d :%7d", mStepLeftTarget, mStepRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d", robotDrive.leftMotor1.getCurrentPosition(), robotDrive.rightMotor1.getCurrentPosition());
                    if (!robotDrive.leftMotor1.isBusy()) {
                        fileLogger.writeEvent("runningTurnStep()","Complete         " );
                        mCurrentTurnState = stepState.STATE_COMPLETE;
                    }
                } else if (mStepTurnL == 0) {
                    fileLogger.writeEvent("runningTurnStep()","Running         " );
                    telemetry.addData("Path1", "Running to %7d :%7d", mStepLeftTarget, mStepRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d", robotDrive.leftMotor1.getCurrentPosition(), robotDrive.rightMotor1.getCurrentPosition());
                    if (!robotDrive.rightMotor1.isBusy()) {
                        fileLogger.writeEvent("runningTurnStep()","Complete         " );
                        mCurrentTurnState = stepState.STATE_COMPLETE;
                    }
                } else {
                    // Stop all motion by setting power to 0
                    setDriveMotorPower(Math.abs(0));
                    fileLogger.writeEvent("runningTurnStep()","Complete         " );
                    mCurrentTurnState = stepState.STATE_COMPLETE;
                }
            }
            break;
        }
    }

    public void runningTurnStep ()
    {
        switch (mCurrentTurnState) {
            case STATE_INIT: {
                fileLogger.writeEvent("runningTurnStep()","mStepTurnL      :- " + mStepTurnL  );
                fileLogger.writeEvent("runningTurnStep()","mStepTurnR      :- " + mStepTurnR  );

                // Turn On RUN_TO_POSITION
                if(mStepTurnR == 0) {
                    // Determine new target position
                    fileLogger.writeEvent("runningTurnStep()","Current LPosition:-" + robotDrive.leftMotor1.getCurrentPosition());
                    mStepLeftTarget = robotDrive.leftMotor1.getCurrentPosition() + (int)(0.5 * mStepTurnL * COUNTS_PER_DEGREE);
                    mStepRightTarget = robotDrive.rightMotor1.getCurrentPosition() - (int)(0.5 * mStepTurnL * COUNTS_PER_DEGREE);
                    fileLogger.writeEvent("runningTurnStep()","mStepLeftTarget:-  " + mStepLeftTarget);
                }
                else {
                    // Determine new target position
                    fileLogger.writeEvent("runningTurnStep()","Current RPosition:-" + robotDrive.rightMotor1.getCurrentPosition());
                    mStepLeftTarget = robotDrive.leftMotor1.getCurrentPosition() - (int)(0.5 * mStepTurnR * COUNTS_PER_DEGREE);
                    mStepRightTarget = robotDrive.rightMotor1.getCurrentPosition() + (int)(0.5 * mStepTurnR * COUNTS_PER_DEGREE);
                    fileLogger.writeEvent("runningTurnStep()","mStepRightTarget:- " + mStepRightTarget);
                }

                // pass target position to motor controller
                robotDrive.leftMotor1.setTargetPosition(mStepLeftTarget);
                robotDrive.rightMotor1.setTargetPosition(mStepRightTarget);
                // set motor controller to mode
                robotDrive.leftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robotDrive.rightMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // set power on motor controller to start moving
                setDriveMotorPower(Math.abs(.5));

                fileLogger.writeEvent("runningTurnStep()","mStepLeftTarget :- " + mStepLeftTarget  );
                fileLogger.writeEvent("runningTurnStep()","mStepRightTarget:- " + mStepRightTarget  );

                mCurrentTurnState = stepState.STATE_RUNNING;
            }
            break;
            case STATE_RUNNING: {
                //if (robotDrive.leftMotor.isBusy() || robotDrive.rightMotor.isBusy()) {
                fileLogger.writeEvent("runningTurnStep()","Running         " );
                fileLogger.writeEvent("runningTurnStep()","Current LPosition:-" + robotDrive.leftMotor1.getCurrentPosition() + " LTarget:- " + mStepLeftTarget);
                fileLogger.writeEvent("runningTurnStep()","Current RPosition:-" + robotDrive.rightMotor1.getCurrentPosition() + " RTarget:- " + mStepRightTarget);
                if (mStepTurnR == 0) {
                    fileLogger.writeEvent("runningTurnStep()","Running         " );
                    telemetry.addData("Path1", "Running to %7d :%7d", mStepLeftTarget, mStepRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d", robotDrive.leftMotor1.getCurrentPosition(), robotDrive.rightMotor1.getCurrentPosition());
                    if (!robotDrive.leftMotor1.isBusy()) {
                        fileLogger.writeEvent("runningTurnStep()","Complete         " );
                        mCurrentTurnState = stepState.STATE_COMPLETE;
                    }
                } else if (mStepTurnL == 0) {
                    fileLogger.writeEvent("runningTurnStep()","Running         " );
                    telemetry.addData("Path1", "Running to %7d :%7d", mStepLeftTarget, mStepRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d", robotDrive.leftMotor1.getCurrentPosition(), robotDrive.rightMotor1.getCurrentPosition());
                    if (!robotDrive.rightMotor1.isBusy()) {
                        fileLogger.writeEvent("runningTurnStep()","Complete         " );
                        mCurrentTurnState = stepState.STATE_COMPLETE;
                    }
                } else {
                    // Stop all motion by setting power to 0
                    setDriveMotorPower(0);
                    fileLogger.writeEvent("runningTurnStep()","Complete         " );
                    mCurrentTurnState = stepState.STATE_COMPLETE;
                }
            }
            break;
        }
    }

    public void runningDriveStep()
    {
        if (mCurrentTurnState == stepState.STATE_COMPLETE) {
            switch (mCurrentDriveState) {
                case STATE_INIT: {
                    fileLogger.writeEvent("runningDriveStep()","mStepDistance   :- " + mStepDistance  );
                    fileLogger.writeEvent("runningDriveStep()","mStepDistance   :- " + mStepDistance  );

                    // Determine new target position
                    mStepLeftTarget = robotDrive.leftMotor1.getCurrentPosition() + (int) (mStepDistance * COUNTS_PER_INCH);
                    mStepRightTarget = robotDrive.rightMotor1.getCurrentPosition() + (int) (mStepDistance * COUNTS_PER_INCH);
                    // pass target position to motor controller
                    robotDrive.leftMotor1.setTargetPosition(mStepLeftTarget);
                    robotDrive.rightMotor1.setTargetPosition(mStepRightTarget);

                    fileLogger.writeEvent("runningDriveStep()","mStepLeftTarget :- " + mStepLeftTarget  );
                    fileLogger.writeEvent("runningDriveStep()","mStepRightTarget:- " + mStepRightTarget  );

                    // set motor controller to mode, Turn On RUN_TO_POSITION
                    robotDrive.leftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robotDrive.rightMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // set power on motor controller to start moving
                    setDriveMotorPower(Math.abs(mStepSpeed));

                    mCurrentDriveState = stepState.STATE_RUNNING;
                }
                break;
                case STATE_RUNNING: {
                    if (robotDrive.leftMotor1.isBusy() && robotDrive.rightMotor1.isBusy()) {
                        telemetry.addData("Path1", "Running to %7d :%7d", mStepLeftTarget, mStepRightTarget);
                        telemetry.addData("Path2", "Running at %7d :%7d", robotDrive.leftMotor1.getCurrentPosition(), robotDrive.rightMotor1.getCurrentPosition());
                    } else {
                        // Stop all motion;
                        setDriveMotorPower(0);
                        baseStepComplete = true;
                        fileLogger.writeEvent("runningDriveStep()","Complete         " );
                        mCurrentDriveState = stepState.STATE_COMPLETE;
                    }
                }
                break;
            }
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

    public sixValues ProcessCurrentNode (int currentX, int currentY, int endX, int endY) {
        boolean closed = false;
        boolean diagonal = false;
        boolean canWalk = false;
        int searching = 1;
        int found = 0;
        boolean empty = false;
        double distFromCurrentToIJ = 0, distFromStartToCurrent = 0;
        double tempF, tempG, tempH;
        int minF;
        double lowestF = -1;
        int lowestFKey = 0;
        sixValues returnValue = new sixValues(0,0,0,0,0,0);

        AStarValue AStarValueCurrerntXY = new AStarValue();
        AStarValue AStarValueCurrerntIJ = new AStarValue();

        tempF = 99999;

        //add point to be process to the closed list
        AStarClosed.xvalue = currentX;
        AStarClosed.yvalue = currentY;
        AStarClosed.ID = getKey(AStarClosed.xvalue, AStarClosed.yvalue, a0Star.fieldWidth, a0Star.fieldLength);
        AStarClosedMap.put(String.valueOf(AStarClosed.ID), AStarClosed);
        //fileLogger.writeEvent("ProcessCurrentNode()","Added to closed list ");

        //analyze adjacent blocks/grid locations
        //key exists so get the values
        if (AStarValueMap.containsKey(getKey(currentX, currentY, a0Star.fieldWidth, a0Star.fieldLength)))
            AStarValueCurrerntXY = AStarValueMap.get(String.valueOf(getKey(currentX, currentY, a0Star.fieldWidth, a0Star.fieldLength)));
        else {
            AStarValueCurrerntXY.xvalue = currentX;
            AStarValueCurrerntXY.yvalue = currentY;
            AStarValueCurrerntXY.FValue = 0;
            AStarValueCurrerntXY.GValue = 0;
            AStarValueCurrerntXY.HValue = 0;
            AStarValueCurrerntXY.ID = 0;
        }

        AStarValues.xvalue = currentX;
        AStarValues.yvalue = currentY;
        AStarValues.ID = getKey(AStarValues.xvalue, AStarValues.yvalue, a0Star.fieldWidth, a0Star.fieldLength);

        for (int i = Math.max(0, (currentX - 1)); i <= Math.min(a0Star.fieldWidth - 1, currentX + 1); i++ ) {
            for (int j = Math.max(0, (currentY - 1)); j <= Math.min(a0Star.fieldLength - 1, currentY + 1); j++ ) {
                if ((i == currentX) && (j == currentY)) {
                    //fileLogger.writeEvent("ProcessCurrentNode()","i=x and j=y - nothing to do");
                } else {
                    //check if its on the closed list
                    //fileLogger.writeEvent("ProcessCurrentNode()","checking if on closed list " + i + " " + j);
                    if (AStarClosedMap.containsKey(String.valueOf(getKey(i, j, a0Star.fieldWidth, a0Star.fieldLength))))
                        closed = true;      //need to check if this returns null it doesn't error out
                    else
                        closed = false;                                                                 //not on closed list, must be on open list
                    //fileLogger.writeEvent("ProcessCurrentNode()","on closed list " + closed);

                    //fileLogger.writeEvent("ProcessCurrentNode()","checking if diagonal ");
                    if (( i + j ) % 2  == (currentX + currentY) % 2)
                        diagonal = true;
                    else
                        diagonal = false;

                    //fileLogger.writeEvent("ProcessCurrentNode()","Is diagonal " + diagonal);

                    if (diagonal) {
                        canWalk = a0Star.walkable[i][j] && a0Star.walkable[currentX][j] && a0Star.walkable[i][currentY];
                        distFromCurrentToIJ = 1420;                                                      //G Value
                    } else {
                        canWalk = a0Star.walkable[i][j];
                        distFromCurrentToIJ = 1000;                                                      //G Value
                    }
                }
                if (!closed && canWalk){
                    //calculated G,H,and F
                    tempG = AStarValueCurrerntXY.GValue + distFromCurrentToIJ;
                    tempH = Math.abs(i - endX) + Math.abs(j - endY);                              //insert heuristic of choice (we use manhattan)
                    //NOTE : you could also use point_distance(i,j,endX,endY);
                    tempF = tempG + tempH;
                    //update if necessary
                    if (AStarValueMap.containsKey(getKey(i, j, a0Star.fieldWidth, a0Star.fieldLength))){   //see if key is in G Map, means already processed
                        //var oldG=;
                        //show_debug_message(string(tempG)+" compare to "+string(oldG));
                        AStarValueCurrerntIJ = AStarValueMap.get(String.valueOf(getKey(i, j, a0Star.fieldWidth, a0Star.fieldLength)));
                        if (tempG < AStarValueCurrerntIJ.GValue) {
                            AStarValueMap.remove(String.valueOf(AStarValues.ID));
                            //fileLogger.writeEvent("ProcessCurrentNode()", "Removed OLD Key (" + i + "," + j + ") Key " + AStarValues.ID + "    G:" + tempG + "     H:" + tempH + "     F:" + tempF);

                            AStarValues.xvalue = i;
                            AStarValues.yvalue = j;
                            AStarValues.zvalue = 0;
                            AStarValues.GValue = tempG;
                            AStarValues.HValue = tempH;
                            AStarValues.FValue = tempF;
                            AStarValues.Parent = getKey(currentX, currentY, a0Star.fieldWidth, a0Star.fieldLength);
                            AStarValues.ID = getKey(AStarValues.xvalue, AStarValues.yvalue, a0Star.fieldWidth, a0Star.fieldLength);
                            AStarValueMap.put(String.valueOf(AStarValues.ID), new  AStarValue(AStarValues.ID, AStarValues.FValue, AStarValues.GValue, AStarValues.HValue, AStarValues.Parent, AStarValues.xvalue, AStarValues.yvalue, AStarValues.zvalue));
                            //fileLogger.writeEvent("ProcessCurrentNode()", "Updating (" + i + "," + j + ") Key " + AStarValues.ID + "    G:" + tempG + "     H:" + tempH + "     F:" + tempF);
                        }
                    } else {
                        AStarValues.xvalue = i;
                        AStarValues.yvalue = j;
                        AStarValues.zvalue = 0;
                        AStarValues.GValue = tempG;
                        AStarValues.HValue = tempH;
                        AStarValues.FValue = tempF;
                        AStarValues.Parent = getKey(currentX, currentY, a0Star.fieldWidth, a0Star.fieldLength);
                        AStarValues.ID = getKey(AStarValues.xvalue, AStarValues.yvalue, a0Star.fieldWidth, a0Star.fieldLength);
                        AStarValueMap.put(String.valueOf(AStarValues.ID), new  AStarValue(AStarValues.ID, AStarValues.FValue, AStarValues.GValue, AStarValues.HValue, AStarValues.Parent, AStarValues.xvalue, AStarValues.yvalue, AStarValues.zvalue));
                        //fileLogger.writeEvent("ProcessCurrentNode()", "Adding (" + i + "," + j + ") Key " + AStarValues.ID + "    G:" + tempG + "     H:" + tempH + "     F:" + tempF);
                    }
                }
            }
        }
        lowestF = 999999;
        //find best option
        for (String key: AStarValueMap.keySet()) {
            AStarValueCurrerntIJ = AStarValueMap.get(key);
            //fileLogger.writeEvent("ProcessCurrentNode()", "Valid key " + key + " ID " + AStarValueCurrerntIJ.ID + " FValue " + AStarValueCurrerntIJ.FValue + " LowestF " + lowestF);
            if (AStarValueCurrerntIJ.FValue < lowestF) {
                lowestF = AStarValueCurrerntIJ.FValue;
                lowestFKey = AStarValueCurrerntIJ.ID;
                //fileLogger.writeEvent("ProcessCurrentNode()", "Found LowerF " + lowestF);
            }
        }
        if (lowestF != 999999) {
            //found low values in map, remove it from map
            pathValues[pathIndex].val1 = (double)pathIndex;
            pathValues[pathIndex].val2 = currentX;
            pathValues[pathIndex].val3 = currentY;  //path_add_point (path, getXPos(curNode, a0Star.fieldWidth, a0Star.fieldLength), getYPos(curNode, a0Star.fieldWidth, a0Star.fieldLength);

            pathIndex++;

            AStarValueMap.remove(lowestFKey);
            currentX = getXPos(lowestFKey, a0Star.fieldWidth, a0Star.fieldLength);
            currentY = getYPos(lowestFKey, a0Star.fieldWidth, a0Star.fieldLength);
            fileLogger.writeEvent("ProcessCurrentNode()", "Trying Key " + getKey(currentX, currentY, a0Star.fieldWidth, a0Star.fieldLength) + " (" + currentX +  " " + currentY + ")");
            searching = 1;
            found = 0;
        } else {
            searching = 0;
            found = 0;
            fileLogger.writeEvent("ProcessCurrentNode()", "No More Nodes Left");
        }
        //check whether we're at the end
        if ((currentX == endX) && (currentY == endY)) {
            pathValues[pathIndex].val1 = (double)pathIndex;
            pathValues[pathIndex].val2 = currentX;
            pathValues[pathIndex].val3 = currentY;  //path_add_point (path, getXPos(curNode, a0Star.fieldWidth, a0Star.fieldLength), getYPos(curNode, a0Star.fieldWidth, a0Star.fieldLength);

            searching = 0;
            found = 1;
            fileLogger.writeEvent("ProcessCurrentNode()", "You found me I'm the final block");
        }
        returnValue.val1 = searching;
        returnValue.val2 = found;
        returnValue.val3 = currentX;
        returnValue.val4 = currentY;
        //fileLogger.writeEvent("ProcessCurrentNode()", "Returning 1- " + searching + " 2- " + found + " 3- "  + currentX + " 4- " + currentY);
        return returnValue;
    }

    public int getKey(int xPos, int yPos, int width, int length) {
        int value;
        value = yPos*length + xPos;
        return value;
    }
    public int getXPos(double key, int width, int length) {
        int value;
        value = (int)key % width;  //modulo
        return value;
    }
    public int getYPos(double key, int width, int length) {
        int value;
        value = (int)key / width;  //integer division
        return value;
    }

    public sixValues[] findPathAStar (int startX, int startY, int endX, int endY ) {
        boolean searching = true;

        for (String key: AStarValueMap.keySet())
            fileLogger.writeEvent("init()","Keys " + key + ": x:" + AStarValueMap.get(key).xvalue + " y:" + AStarValueMap.get(key).yvalue);

        //process map
        sixValues currentResult = new sixValues(1,0,AStarValues.xvalue,AStarValues.yvalue,0,0);

        while (searching) {
            fileLogger.writeEvent("init()","Searching ");
            currentResult = (ProcessCurrentNode((int)currentResult.val3, (int)currentResult.val4, endX, endY));
            searching = (currentResult.val1 == 1);
        }

        boolean found = (currentResult.val2 == 1);

        if (found) {

            for ( int i = 0; i < pathValues.length; i++) {
                fileLogger.writeEvent("init()","Path " + pathValues[i].val1 + " " + pathValues[i].val2 + " " + pathValues[i].val3 );
                if ((pathValues[i].val2 == endX) && (pathValues[i].val3 == endY)) {
                    break;
                }
            }

        }

        return pathValues;
    }

    void setDriveMotorPower (double power) {
        setDriveRightMotorPower(power);
        setDriveLeftMotorPower(power);
    }

    void setDriveRightMotorPower (double power) {
        robotDrive.rightMotor1.setPower(power);
        robotDrive.rightMotor2.setPower(power);

    }

    void setDriveLeftMotorPower (double power) {
        robotDrive.leftMotor1.setPower(power);
        robotDrive.leftMotor2.setPower(power);

    }

}
