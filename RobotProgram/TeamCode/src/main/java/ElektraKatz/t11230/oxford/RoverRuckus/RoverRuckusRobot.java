package ElektraKatz.t11230.oxford.RoverRuckus;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a K9 robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Servo channel:  Servo to raise/lower arm: "arm"
 * Servo channel:  Servo to open/close claw: "claw"
 *
 * Note: the configuration of the servos is such that:
 *   UPDATE - As the Lander arm servo approaches 0, the arm position moves up (away from the floor).
 *   UPDATE - As the claw servo approaches 0, the claw opens up (drops the game element).
 */
public class RoverRuckusRobot
{
    /* Public OpMode members. */
    public DcMotor leftMotor1 = null;
    public DcMotor rightMotor1 = null;
    public DcMotor leftMotor2 = null;
    public DcMotor rightMotor2 = null;
    // TODO: uncomment these when arms and servos are added
    public DcMotor liftMotor1 = null;
    //public DcMotor LanderArm2 = null;
    //public Servo landerClaw = null;
   /* public Servo ElementClaw = null;

    static double LANDER_CLAW_CLOSE = 1;


    public final static double LEFT_CLAW_OPEN = 0.5;
    public final static double LEFT_CLAW_CLOSE = 0.8;
    public final static double RIGHT_CLAW_OPEN = 0.2;
    public final static double RIGHT_CLAW_CLOSE = 0.0;
    public final static double ARM_MIN_RANGE  = 0.20;
    public final static double ARM_MAX_RANGE  = 0.90;
    public final static double CLAW_MIN_RANGE  = 0.20;
    public final static double CLAW_MAX_RANGE  = 0.7;
*/
    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public RoverRuckusRobot() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * st   ep (using the FTC Robot Controller app on the phone).
         */
        //Test
        //things like FLM means front left motor
        leftMotor1 = hwMap.get(DcMotor.class, "leftMotor1");
        rightMotor1 = hwMap.get(DcMotor.class, "rightMotor1");
        leftMotor2 = hwMap.get(DcMotor.class, "leftMotor2");
        rightMotor2 = hwMap.get(DcMotor.class, "rightMotor2");
        liftMotor1 = hwMap.get(DcMotor.class, "LanderArm1");
        // TODO: uncomment these when arm is added
        //LanderArm1 = hardwareMap.get(DcMotor.class, "LanderArm1");
        //LanderArm2 = hardwareMap.get(DcMotor.class, "LanderArm2");
        //landerClaw = hardwareMap.servo.get("LanderServo1");


        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        leftMotor1.setDirection(DcMotor.Direction.FORWARD);
        leftMotor2.setDirection(DcMotor.Direction.FORWARD);// Set to REVERSE if using AndyMark motors
        rightMotor1.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        liftMotor1.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //upMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set all motors to zero power
        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);


    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
