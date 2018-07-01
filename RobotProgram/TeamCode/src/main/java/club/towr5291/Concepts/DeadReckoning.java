package club.towr5291.Concepts;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import club.towr5291.functions.FileLogger;
import club.towr5291.robotconfig.HardwareDriveMotors;

/**
 * Created by Wyatt on 6/7/2018.
 */
@Autonomous(name = "DeadRec", group = "Wyatt")
@Disabled
public class DeadReckoning extends LinearOpMode {

    private HardwareDriveMotors robot = new HardwareDriveMotors();   // Use a Pushbot's hardware

    public DcMotor XMotor;
    public DcMotor YMotor;

    static final double     COUNTS_PER_MOTOR_REV    = 400 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4 ;     // For figuring circumference
    static final double     WHEEL_DIAMETER_MILLI    = 42 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     COUNTS_PER_MILLI        = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MILLI * 3.1415);
    static final double     COUNTS_PER_DEGREE       = (COUNTS_PER_MILLI * 7.58);
    static final double DRIVE_COEFF = 0.15;
    double steer, max;
    double ErrorDegree;
    double ErrorXCounts, ErrorYCounts;
    int adelanteError;
    double leftSpeed, rightSpeed;
    int XMotorStart, YMotorStart;
    int moveCounts;
    int targetPositionLeft1;
    int targetPositionLeft2;
    int targetPositionRight1;
    int targetPositionRight2;

    final String TAG = "DeadRecPos";
    private ElapsedTime runtime = new ElapsedTime();
    private FileLogger fileLogger;

    @Override
    public void runOpMode(){

        fileLogger = new FileLogger(runtime);
        fileLogger.open();
        fileLogger.write("Starting File Logger");
        fileLogger.writeEvent(TAG, "Log Started");

        XMotor = hardwareMap.get(DcMotor.class, "XMotor");
        YMotor = hardwareMap.get(DcMotor.class, "YMotor");

        waitForStart();

        forward(5, 1);
    }
    public void forward(int Inches, double speed) {
        restEncoder();

        XMotorStart = XMotor.getCurrentPosition();
        YMotorStart = YMotor.getCurrentPosition();

        fileLogger.writeEvent("XMotorStart = ", String.valueOf(XMotorStart));
        fileLogger.writeEvent("YMotorStart = ", String.valueOf(YMotorStart));

        moveCounts = (int) (Inches * COUNTS_PER_INCH);
        targetPositionLeft1 = robot.baseMotor1.getCurrentPosition() + moveCounts;
        targetPositionLeft2 = robot.baseMotor2.getCurrentPosition() + moveCounts;
        targetPositionRight1 = robot.baseMotor3.getCurrentPosition() + moveCounts;
        targetPositionRight2 = robot.baseMotor4.getCurrentPosition() + moveCounts;

        fileLogger.writeEvent("moveCounts = ", String.valueOf(moveCounts));
        fileLogger.writeEvent("targetPositionLeft1 = ", String.valueOf(targetPositionLeft1));
        fileLogger.writeEvent("targetPositionLeft2 = ", String.valueOf(targetPositionLeft2));
        fileLogger.writeEvent("targetPositionRight1 = ", String.valueOf(targetPositionRight1));
        fileLogger.writeEvent("targetPositionRight2 = ", String.valueOf(targetPositionRight2));

        robot.baseMotor1.setTargetPosition(targetPositionLeft1);
        robot.baseMotor2.setTargetPosition(targetPositionLeft2);
        robot.baseMotor3.setTargetPosition(targetPositionRight1);
        robot.baseMotor4.setTargetPosition(targetPositionRight2);

        robot.baseMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.baseMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.baseMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.baseMotor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.baseMotor1.setPower(speed);
        robot.baseMotor2.setPower(speed);
        robot.baseMotor3.setPower(speed);
        robot.baseMotor4.setPower(speed);

        while (robot.baseMotor1.isBusy() && robot.baseMotor2.isBusy() && robot.baseMotor3.isBusy() && robot.baseMotor4.isBusy() && opModeIsActive()) {

            ErrorXCounts = moveCounts - ((COUNTS_PER_MILLI * XMotor.getCurrentPosition()) * 2);
            ErrorYCounts = moveCounts - ((COUNTS_PER_MILLI * YMotor.getCurrentPosition()) * 2);
            ErrorDegree = ErrorYCounts - ErrorXCounts / COUNTS_PER_DEGREE;//fix
            steer = Range.clip(ErrorDegree * DRIVE_COEFF, -1, 1);
            speed = speed - steer;

            // if driving in reverse, the motor correction also needs to be reversed
            if (Inches < 0)
                steer *= -1.0;

            leftSpeed = speed - steer;
            rightSpeed = speed + steer;

            // Normalize speeds if either one exceeds +/- 1.0;
            max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0) {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            robot.baseMotor1.setPower(leftSpeed);
            robot.baseMotor2.setPower(leftSpeed);
            robot.baseMotor3.setPower(rightSpeed);
            robot.baseMotor4.setPower(rightSpeed);

            telemetry.addLine()
                    .addData("left speed is ", leftSpeed)
                    .addData("right speed is ", rightSpeed);

            fileLogger.writeEvent("left Speed is ", String.valueOf(leftSpeed));
            fileLogger.writeEvent("right Speed is ", String.valueOf(rightSpeed));
        }
        if ((XMotor.getCurrentPosition() - XMotorStart) != moveCounts){//fix
            adelanteError = moveCounts - ((XMotor.getCurrentPosition() - XMotorStart) * 2);

            robot.baseMotor1.setTargetPosition(robot.baseMotor1.getCurrentPosition() + adelanteError);
            robot.baseMotor2.setTargetPosition(robot.baseMotor2.getCurrentPosition() + adelanteError);
            robot.baseMotor3.setTargetPosition(robot.baseMotor3.getCurrentPosition() + adelanteError);
            robot.baseMotor4.setTargetPosition(robot.baseMotor4.getCurrentPosition() + adelanteError);

            robot.baseMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.baseMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.baseMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.baseMotor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.baseMotor1.setPower(speed);
            robot.baseMotor2.setPower(speed);
            robot.baseMotor3.setPower(speed);
            robot.baseMotor4.setPower(speed);
        }

        robot.baseMotor1.setPower(0);
        robot.baseMotor2.setPower(0);
        robot.baseMotor3.setPower(0);
        robot.baseMotor4.setPower(0);
    }
    public void restEncoder(){
        YMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        XMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.baseMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.baseMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.baseMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.baseMotor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}