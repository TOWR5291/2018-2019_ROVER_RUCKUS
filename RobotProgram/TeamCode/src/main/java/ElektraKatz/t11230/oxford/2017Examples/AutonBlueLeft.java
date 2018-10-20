/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package ElektraKatz.t11230.oxford.LastYearExamples;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AutoBlueLeft", group="11230")
//@Disabled
public class AutonBlueLeft extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareCompBot robot   = new HardwareCompBot();   // Use a Pushbot's hardware
    DistanceSensor          sensorDistance;


    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV        = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     LIFT_COUNTS_PER_MOTOR_REV    = 1680 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION        = 1.0 ;    // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES       = 4.0 ;     // For figuring circumference
    static final double     STRAFE_DIST_PER_REV         = 3.0 ;     // Distance travelled per wheel revolution
    static final double     PULLEY_DIAMETER_INCHES      = 1.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH             = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     STRAFE_COUNTS_PER_INCH      = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (STRAFE_DIST_PER_REV);
    static final double     LIFT_COUNTS_PER_INCH        = (LIFT_COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (PULLEY_DIAMETER_INCHES * Math.PI);
    static final double     DRIVE_SPEED             = 0.2;
    static final double     TURN_SPEED              = 0.2;
    final static double     LEFT_CLAW_OPEN = 0.5;
    final static double     LEFT_CLAW_CLOSE = 0.8;
    final static double     LEFT_CLAW_PARK = 1;
    final static double     RIGHT_CLAW_OPEN = 0.6;
    final static double     RIGHT_CLAW_CLOSE = 0.3;
    final static double     RIGHT_CLAW_PARK = 0;

    double                  keyDistance=0;
    double                  jewelOffset=0;

    ColorSensor sensorColor;


    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;


    @Override
    public void runOpMode() {


        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        sensorColor = hardwareMap.get(ColorSensor.class, "color sensor");

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.upMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.rightMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.upMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftClaw.setPosition(LEFT_CLAW_PARK);//Initiate left claw to 0 position
        robot.rightClaw.setPosition(RIGHT_CLAW_PARK);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.leftMotor1.getCurrentPosition(),
                robot.leftMotor2.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);



        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        keyDistance = readPictograph();
        encoderLift(.7,6,5.0);
        openClaws();
        encoderLift(.7,-6,5.0);
        closeClaws();
        encoderLift(.4,2,2.0);
        jewelOffset = getJewelRed(sensorColor.red(), sensorColor.blue());
        encoderDrive(DRIVE_SPEED, 24-jewelOffset,  24-jewelOffset, 15.0);  // Forward 24 inches
        encoderDrive(TURN_SPEED, -22, 22, 10); // Turn around
        strafeRight(DRIVE_SPEED,  keyDistance, 9.0);  // Stafe right to align with key
        encoderDrive(DRIVE_SPEED, 8, 8, 4.0);  // Drive to Cryptobox
        openClaws();
        encoderDrive(DRIVE_SPEED, -2, -2, 4.0);  // Back away from glyph


        telemetry.addData("Path", "Complete");

        telemetry.update();
        // Set the panel back to the default color
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });

    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget1;
        int newLeftTarget2;
        int newRightTarget1;
        int newRightTarget2;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget1 = robot.leftMotor1.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newLeftTarget2 = robot.leftMotor2.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget1 = robot.rightMotor1.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRightTarget2 = robot.rightMotor2.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftMotor1.setTargetPosition(newLeftTarget1);
            robot.leftMotor2.setTargetPosition(newLeftTarget2);
            robot.rightMotor1.setTargetPosition(newRightTarget1);
            robot.rightMotor2.setTargetPosition(newRightTarget2);

            // Turn On RUN_TO_POSITION
            robot.rightMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor1.setPower(Math.abs(speed));
            robot.rightMotor1.setPower(Math.abs(speed));
            robot.leftMotor2.setPower(Math.abs(speed));
            robot.rightMotor2.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor1.isBusy() && robot.rightMotor1.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget1,  newRightTarget1);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftMotor1.getCurrentPosition(),
                        robot.rightMotor1.getCurrentPosition());
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftMotor2.getCurrentPosition(),
                        robot.rightMotor2.getCurrentPosition());
                telemetry.addData("power1",  "Running at %7d :%7d",
                        robot.leftMotor2.getCurrentPosition(),
                        robot.rightMotor2.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion
            robot.leftMotor1.setPower(0);
            robot.rightMotor1.setPower(0);
            robot.rightMotor2.setPower(0);
            robot.leftMotor2.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void strafeRight(double speed, double rightInches, double timeoutS) {
        int newTarget1, newTarget2, newTarget3, newTarget4;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget1 = robot.leftMotor1.getCurrentPosition() + (int)(rightInches * STRAFE_DIST_PER_REV);
            newTarget2 = robot.leftMotor2.getCurrentPosition() - (int)(rightInches * STRAFE_DIST_PER_REV);
            newTarget3 = robot.rightMotor1.getCurrentPosition() - (int)(rightInches * STRAFE_DIST_PER_REV);
            newTarget4 = robot.rightMotor2.getCurrentPosition() + (int)(rightInches * STRAFE_DIST_PER_REV);
            robot.leftMotor1.setTargetPosition(newTarget1);
            robot.leftMotor2.setTargetPosition(newTarget2);
            robot.rightMotor1.setTargetPosition(newTarget3);
            robot.rightMotor2.setTargetPosition(newTarget4);

            // Turn On RUN_TO_POSITION
            robot.rightMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor1.setPower(Math.abs(speed));
            robot.rightMotor1.setPower(Math.abs(speed));
            robot.leftMotor2.setPower(Math.abs(speed));
            robot.rightMotor2.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor1.isBusy() && robot.rightMotor1.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Target",  "Running to %7d", newTarget1);
                telemetry.addData("Path1",  "Running at %7d :%7d",
                        robot.leftMotor1.getCurrentPosition(),
                        robot.rightMotor1.getCurrentPosition());
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftMotor2.getCurrentPosition(),
                        robot.rightMotor2.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion
            robot.leftMotor1.setPower(0);
            robot.rightMotor1.setPower(0);
            robot.rightMotor2.setPower(0);
            robot.leftMotor2.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void encoderLift(double speed, double raiseInches, double timeoutS) {
        int newTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = robot.upMotor.getCurrentPosition() + (int)(raiseInches * LIFT_COUNTS_PER_INCH);
            robot.upMotor.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            robot.upMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.upMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.upMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Lift",  "Running to %7d", newTarget);
                telemetry.addData("Lift",  "Running at %7d",
                        robot.upMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion
            robot.upMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.upMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public double getJewelRed(int sensorRed, int sensorBlue) {

        double jewelDistance;
        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;


        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            jewelDistance=-3;
            telemetry.addData("Red = 7d", sensorRed);
            telemetry.addData("Blue = 7d", sensorBlue);
            telemetry.update();
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            robot.jewelArm.setPosition(0.7);
            sleep(1000);
            if (sensorRed > sensorBlue){
                jewelDistance=3;
            }else if(sensorBlue > sensorRed){
                jewelDistance=-3;
            }else{
                jewelDistance=0;
            }
            encoderDrive(DRIVE_SPEED, jewelDistance, jewelDistance, 3.0);
            robot.jewelArm.setPosition(0.2);
            sleep(1000);

            return(jewelDistance);
        }
        return(0);
    }

    public double readPictograph(){
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AZMqAHf/////AAAAGVd8Se7Hs0p1vryExhCkOP4ITaW6G1inYwiJd0WQdhRGOScTLAYvku1F9nvLLcpleZqETx7ZZJeVayZE0VmhRElmwvRaFB7tKzXw7rqvO9rK01S4UGXQIYvdxO4LYbBh3IUreMSOR8oJezIELz2r2epdEaLX5LssbxIWu9ofa0GnsMQaRIrb4Up36XKyCx3hnnNEfREwuzZCehbNZ1RPwfRK/NX7HdrlgYwIHxbu9KaWlChBibzXh1LrnkDNuXNTkhmxxH+yCO8yTX3uW0Lg/YG7Od5tlBpfsnHUESV8KudzY5T3EhMiLbRR4ZWWKWcK45e/a/rafRA4xZg0r4MxkG+JaPKsXgPs6XDZYf7zhnx1";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        /**
         * See if any of the instances of {@link relicTemplate} are currently visible.
         * {@link RelicRecoveryVuMark} is an enum which can have the following values:
         * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
         * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
         */
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

            /* Found an instance of the template. In the actual game, you will probably
             * loop until this condition occurs, then move on to act accordingly depending
             * on which VuMark was visible. */
            telemetry.addData("VuMark", "%s visible", vuMark);


            if (vuMark==RelicRecoveryVuMark.LEFT){
                return (19.08);
            }
            else if (vuMark == RelicRecoveryVuMark.CENTER){
                return(11.45);
            }else if(vuMark==RelicRecoveryVuMark.RIGHT){
                return (3.82);
            }
        }
        else {
            telemetry.addData("VuMark", "not visible");
            return (11.45);
        }

        return (11.45);

    }

    public void openClaws(){
        robot.leftClaw.setPosition(LEFT_CLAW_OPEN);
        robot.rightClaw.setPosition(RIGHT_CLAW_OPEN);
        sleep(1000);

    }

    public void closeClaws(){
        robot.leftClaw.setPosition(LEFT_CLAW_CLOSE);
        robot.rightClaw.setPosition(RIGHT_CLAW_CLOSE);
        sleep(1000);

    }


}
