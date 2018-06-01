package club.towr5291.opmodes;

/**
 * Created by kids on 9/30/2017 at 9:55 AM.
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import java.util.Locale;

//import club.towr5291.robotconfig.HardwareArmMotorsRR;

@TeleOp(name="RR Mecanum Test", group="RRTest")
//@Disabled
public class RRMecanumTest extends OpMode {

    //motors
    private DcMotor leftFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightBackMotor;

    //mode selection stuff
    public boolean activated = false;
    public int mode = 0;

    //general variables
    public float speed = 0;
    public float turn = 0;
    public float intendedTurn = 0;
    public float strafe = 0;

    //all modes variables
    public double leftFrontSpeed = 0;
    public double leftBackSpeed = 0;
    public double rightFrontSpeed = 0;
    public double rightBackSpeed = 0;


    //gyro assisted and field-centric driving variables
    public int quadrant = 1;
    public double radius = 0;
    public double heading = 0;
    public float ajustedHeading = 0;
    public float revHeading = 0;
    public float driftCorrectionHeadingStart = 0;
    public boolean enableDriftCorrection = false;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    @Override
    public void init() {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //drive base motors
        leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        leftBackMotor = hardwareMap.dcMotor.get("leftBackMotor");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
        rightBackMotor = hardwareMap.dcMotor.get("rightBackMotor");

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);   //left
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);    //right
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  //right
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);   //left



        /*
        lF,lB,rF,rB
        Forward: 1,1,1,1
        Back: -1,-1,-1,-1
        Right: -1,1,1,-1
        Left: 1,-1,-1,1
        TurnClockwise: 1,1,-1,-1
        TurnCounterClockwise: -1,-1,1,1
        DiagRightUp: 0,1,1,0
        DiagRightDown: 0,-1,-1,0
        DiagLeftUp: 1,0,0,1
        DiagLeftDown: -1,0,0,-1

        Gyro is port 0x28, bus 0



        */

//        telemetry.addData("Init?", "Initialized");
    }
//    @Override
//    public void init_loop() {
//    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
//    @Override
//    public void start() {
//    }


    @Override
    public void loop() {
        if (activated == false) {
            if (gamepad1.a) {
                mode = 0;
                activated = true;
            } else if (gamepad1.b) {
                mode = 1;
                activated = true;
            } else if (gamepad1.x) {
                mode = 2;
                activated = true;
            } else if (gamepad1.y) {
                mode = 3;
                activated = true;
            }


        } else {
            if (mode == 0) {
//                setMotorPowers(gamepad1.left_stick_y,gamepad1.right_stick_y,gamepad2.left_stick_y,gamepad2.right_stick_y);
//                leftFrontMotor.setPower(-gamepad1.left_stick_y);
//                leftBackMotor.setPower(gamepad2.left_stick_y);
//                rightFrontMotor.setPower(gamepad1.right_stick_y);
//                rightBackMotor.setPower(-gamepad2.right_stick_y);
                leftFrontSpeed = -gamepad1.left_stick_y;
                leftBackSpeed = gamepad2.left_stick_y;
                rightFrontSpeed = gamepad1.right_stick_y;
                rightBackSpeed = -gamepad2.right_stick_y;

                setMotorPowers(leftFrontSpeed,leftBackSpeed,rightFrontSpeed,rightBackSpeed);
            }

            if (mode == 1) {
                if ((gamepad1.dpad_up || gamepad1.dpad_down) && (!gamepad1.dpad_right && !gamepad1.dpad_left)) {
                    leftFrontMotor.setPower(-gamepad1.right_stick_y);
                    leftBackMotor.setPower(gamepad1.right_stick_y);
                    rightFrontMotor.setPower(gamepad1.right_stick_y);
                    rightBackMotor.setPower(-gamepad1.right_stick_y);
                    telemetry.addData("Forward/Back", 1);
                    telemetry.addData(String.valueOf(gamepad1.right_stick_y), "Power");
                } else if ((gamepad1.dpad_right || gamepad1.dpad_left) && (!gamepad1.dpad_up && !gamepad1.dpad_down)) {
                    leftFrontMotor.setPower(gamepad1.right_stick_x);
                    leftBackMotor.setPower(gamepad1.right_stick_x);
                    rightFrontMotor.setPower(gamepad1.right_stick_x);
                    rightBackMotor.setPower(gamepad1.right_stick_x);
                    telemetry.addData("Left/Right", 1);
                    telemetry.addData(String.valueOf(gamepad1.right_stick_x), "Power");
                } else if (!(gamepad1.dpad_right && gamepad1.dpad_left && gamepad1.dpad_up && gamepad1.dpad_down)) {
                    turn = -(gamepad1.right_trigger - gamepad1.left_trigger);
                    leftFrontMotor.setPower(-turn);
                    leftBackMotor.setPower(turn);
                    rightFrontMotor.setPower(-turn);
                    rightBackMotor.setPower(turn);
                    telemetry.addData("Turning", 1);
                    telemetry.addData(String.valueOf(turn), "Power");
                } else if ((gamepad1.dpad_up && gamepad1.dpad_right) || (gamepad1.dpad_down && gamepad1.dpad_right)) {
                    leftFrontMotor.setPower(0);
                    leftBackMotor.setPower((0.5 * gamepad1.right_stick_y) + (0.5 * gamepad1.right_stick_x));
                    rightFrontMotor.setPower((0.5 * gamepad1.right_stick_y) + (0.5 * gamepad1.right_stick_x));
                    rightBackMotor.setPower(0);
                    telemetry.addData("Diag UpRight", 1);
                    telemetry.addData(String.valueOf((0.5 * gamepad1.right_stick_x) + (0.5 * gamepad1.right_stick_y)), "Power");
                } else if ((gamepad1.dpad_up && gamepad1.dpad_left) || (gamepad1.dpad_down && gamepad1.dpad_right)) {
                    leftFrontMotor.setPower(-((0.5 * gamepad1.right_stick_y) + (-0.5 * gamepad1.right_stick_x)));
                    leftBackMotor.setPower(0);
                    rightFrontMotor.setPower(0);
                    rightBackMotor.setPower(-((0.5 * gamepad1.right_stick_y) + (-0.5 * gamepad1.right_stick_x)));
                    telemetry.addData("Diag UpLeft", 1);
                    telemetry.addData(String.valueOf((0.5 * gamepad1.right_stick_y) + (-0.5 * gamepad1.right_stick_x)), "Power");
                }
            } else if (mode == 2) {
                //LF = +Speed+Turn-Strafe   RF = +Speed-Turn+Strafe
                //LB = +Speed+Turn+Strafe   RB = +Speed-Turn-Strafe

                speed = gamepad1.left_stick_y;
                turn = -gamepad1.right_stick_x;
                strafe = gamepad1.left_stick_x;

                leftFrontSpeed = speed+turn-strafe;
                leftBackSpeed = speed+turn+strafe;
                rightFrontSpeed = speed-turn+strafe;
                rightBackSpeed = speed-turn-strafe;

                setMotorPowers(leftFrontSpeed, leftBackSpeed, rightFrontSpeed, rightBackSpeed);
            } else if (mode == 3) {

                // Start the logging of measured acceleration
                imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

                //NOTE:
                //Quadrants in this program are going clockwise, with quad 1 bein up right, quad 2 being down right, quad 3 being down left, quad 4 being up left


                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

                radius = (float) Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_x, 2));

                telemetry.addData(String.valueOf(gamepad1.left_stick_x), "X");
                telemetry.addData(String.valueOf(gamepad1.left_stick_y), "Y");

                if (gamepad1.left_stick_y >= 0 && gamepad1.left_stick_x >= 0) {
                    quadrant = 1;
                    heading = determineHeading(gamepad1.left_stick_x);
                } else if (gamepad1.left_stick_y < 0 && gamepad1.left_stick_x > 0) {
                    quadrant = 2;
                    heading = 90 + determineHeading(Math.abs(gamepad1.left_stick_y));
                } else if (gamepad1.left_stick_y < 0 && gamepad1.left_stick_x < 0) {
                    quadrant = 3;
                    heading = 180 + determineHeading(Math.abs(gamepad1.left_stick_x));
                } else if (gamepad1.left_stick_y > 0 && gamepad1.left_stick_x < 0) {
                    quadrant = 4;
                    heading = 270 + determineHeading(Math.abs(gamepad1.left_stick_y));
                }

                revHeading = angles.firstAngle;
                ajustedHeading = determineAjustedHeading();
                turn = -gamepad1.right_stick_x;
                speed = determineSpeed(ajustedHeading, (float) radius);
                strafe = determineSpeed(ajustedHeading, (float) radius);

                if (turn == 0 && speed == 0 && strafe == 0) {
                    driftCorrectionHeadingStart = revHeading;
                }

                if (turn == 0) {
                    enableDriftCorrection = true;
                    if (driftCorrectionHeadingStart < revHeading) {
                        turn += 0.1;
                    } else if (driftCorrectionHeadingStart > revHeading) {
                        turn -= 0.1;
                    }
                }

                leftFrontSpeed = speed+turn-strafe;
                leftBackSpeed = speed+turn+strafe;
                rightFrontSpeed = speed-turn+strafe;
                rightBackSpeed = speed-turn-strafe;

                setMotorPowers(leftFrontSpeed,leftBackSpeed,rightFrontSpeed,rightBackSpeed);

                telemetry.addData(String.valueOf(heading), "Heading");
                telemetry.addData(String.valueOf(radius), "Power");
                telemetry.addData(String.valueOf(intendedTurn), "Intended Turn");
                telemetry.addData(String.valueOf(revHeading), "Rev Heading");
                telemetry.addData(String.valueOf(ajustedHeading), "Ajusted Heading");
                telemetry.addData(String.valueOf(speed), "Speed");
                telemetry.addData(String.valueOf(strafe), "Strafe");
                telemetry.addData(String.valueOf(turn), "Turn");

                telemetry.addData(String.valueOf(enableDriftCorrection), "Drift Correction");

                telemetry.addData(" ", "Rev IMU");
                telemetry.addData(String.valueOf(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)), "Angles");


            }
        }
        }

    public void setMotorPowers(double leftFront, double leftBack, double rightFront, double rightBack) {
        leftFrontMotor.setPower(-leftFront);
        leftBackMotor.setPower(leftBack);
        rightFrontMotor.setPower(rightFront);
        rightBackMotor.setPower(-rightBack);
    }

    public double determineHeading(float x) {
        return Math.asin(x/radius);
    }

    public float determineAjustedHeading() {
        // ajustedHeading = heading from driver - robot heading relative to driver
        ajustedHeading = (float) (heading - revHeading);
        return ajustedHeading;
    };

    public float determineSpeed(float angle, float distance) {
        speed = (float) (distance/(-Math.sin(angle)));
        return speed;
    }

    public float determineStrafe(float angle, float distance) {
        strafe = (float) (distance/(-Math.sin(angle)));
        return strafe;
    }

    public boolean moveServo (Servo Servo, double Position, double RangeMin, double RangeMax ) {
        boolean OKToMove = true;

        //set right position
        if ((Range.scale(Position, 0, 180, 0, 1) < RangeMin ) || (Range.scale(Position, 0, 180, 0, 1) > RangeMax )) {
            return false;
        }

        Servo.setPosition(Range.scale(Position, 0, 180, 0, 1));

        return true;
    }


//    public void powerMotors(float heading, float power, float rotation) {
//
//    }

    /*
    * Code to run ONCE after the driver hits STOP
    */
//    @Override
//    public void stop() {
//    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}

