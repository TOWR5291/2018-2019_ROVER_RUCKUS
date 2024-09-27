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

package ElektraKatz.t11230.oxford.RoverRuckus;

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
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


import java.util.ArrayList;
import java.util.List;

import ElektraKatz.t11230.oxford.LastYearExamples.HardwareCompBot;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

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

@Autonomous(name="RoverRuckusAuton", group="ElektraKatz")
//@Disabled
public class ElektraKatzRoverRuckusAuton extends LinearOpMode {

    /* Declare OpMode members. */
    RoverRuckusRobot robot   = new RoverRuckusRobot();   // Use a Pushbot's hardware
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
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AeFRhev/////AAABmZ0RDop0TkPSrFphLTLiFd8/W356D3elsFdSuPSWgfIPEy3A1fnmQHv5h0g74BgHFRzumh4gR1CTWxsxmrdU5wgMgwmXgpia1HYL7nTYaWxD4SlGJmghtJ/Nq4WX9RZouFV9lHrUl2MrU5gqDepNLY7yGt9VI3NYVnGVcmYdUNUZw3TnooAj8QgvSUolTGZTiZ2LCu1t1I0vf8jAbZ633DjdWD4GYGgU4FFG8Lk+4pIo+WWoan6UR47nh0XpRGQ16UaeUuRkVIsLp30AxX07hUB97Yd5Iu+R9HR5o0mlIc1iIFbzG6TdUDNlwf7gGm+QpcfDenZMBLKlmZAbaG8zq5i0KvK4Z7mH7JzQH3ymOZcI";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    //public static final String TAG = "Vuforia VuMark Sample";

    /**variable we will use to store our instance of the Vuforia localization engine.   */
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
        //robot.upMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.rightMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // robot.upMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // robot.leftClaw.setPosition(LEFT_CLAW_PARK);//Initiate left claw to 0 position
       // robot.rightClaw.setPosition(RIGHT_CLAW_PARK);

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY ;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * This Rover Ruckus sample places a specific target in the middle of each perimeter wall.
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        /**
         * To place the BlueRover target in the middle of the blue perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Then, we translate it along the Y axis to the blue perimeter wall.
         */
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        /**
         * To place the RedFootprint target in the middle of the red perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative Y axis to the red perimeter wall.
         */
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        /**
         * To place the FrontCraters target in the middle of the front perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative X axis to the front perimeter wall.
         */
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        /**
         * To place the BackSpace target in the middle of the back perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the X axis to the back perimeter wall.
         */
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        /**
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
         * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
         *
         * If using the rear (High Res) camera:
         * We need to rotate the camera around it's long axis to bring the rear camera forward.
         * This requires a negative 90 degree rotation on the Y axis
         *
         * If using the Front (Low Res) camera
         * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
         * This requires a Positive 90 degree rotation on the Y axis
         *
         * Next, translate the camera lens to where it is on the robot.
         * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
         */

        final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        /** Start tracking the data sets we care about. */
        targetsRoverRuckus.activate();

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



        // Step through each leg of the path,  (See Last year for examples AutonBlueLeft)
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Step 1:  Lower From Lander
        // Step 2:  Turn Slightly to the left so we are looking at Location Target
        // Step 3:  Determine which target we are seeing so we know what are starting position was
        //    SWITCH - If SPACE Then Start was Red-1
        //             IF Footprint then Start was Red-2
        //             IF Craters then Start was Blue-1
        //             IF Rover then Start was Blue-2
        // Step 4:  If start position was 1 (Red 1 or Blue 1)
        //            Then - Turn back right to face corner and drive to corner (Use Robot Position from Vuforia to navigate)
        //          Else - start position was 2 (Red 2 or Blue 2)
        //            Then - Drive forward to Navigation Target, Turn Left to face corner Drive to corner
        // Step 5:  Drop team Marker in corner
        // Step 6:  If start position was 1 (Red 1 or Blue 1)
        //            - Turn Left to face Front corner crater, Drive to crater and park
        //          Else - start position was 2 (Red 2 or Blue 2)
        //            - Drive straight backward to Back corner crater and park
        // Step 3B: - If we don't see any target, HOPE WE ARE POSITION 1, turn toward corner drive forward and drop team marker.


 /*         // Step through each leg of the path,  (See Last year for examples AutonBlueLeft)
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Step 1:  Lower From Lander
        // Step 2:  Turn Slightly to the left so we are looking at Location Target
        // Step 3:  Determine which target we are seeing so we know what are starting position was
        //    SWITCH - If SPACE Then Start was Red-1
        //             IF Footprint then Start was Red-2
        //             IF Craters then Start was Blue-1
        //             IF Rover then Start was Blue-2
        // Step 4:  If start position was 1 (Red 1 or Blue 1)
        //            Then - Turn back right to face corner and drive to corner (Use Robot Position from Vuforia to navigate)
        //          Else - start position was 2 (Red 2 or Blue 2)
        //            Then - Drive forward to Navigation Target, Turn Left to face corner Drive to corner
        // Step 5:  Drop team Marker in corner
        // Step 6:  If start position was 1 (Red 1 or Blue 1)
        //            - Turn Left to face Front corner crater, Drive to crater and park
        //          Else - start position was 2 (Red 2 or Blue 2)
        //            - Drive straight backward to Back corner crater and park
        // Step 3B: - If we don't see any target, HOPE WE ARE POSITION 1, turn toward corner drive forward and drop team marker.
      encoderDrive(DRIVE_SPEED, 24-jewelOffset,  24-jewelOffset, 15.0);  // Forward 24 inches
        encoderDrive(TURN_SPEED, -22, 22, 10); // Turn around
        strafeRight(DRIVE_SPEED,  keyDistance, 9.0);  // Stafe right to align with key
        encoderDrive(DRIVE_SPEED, 8, 8, 4.0);  // Drive to Cryptobox
        encoderDrive(DRIVE_SPEED, -2, -2, 4.0);  // Back away from glyph
*/

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

 /*   public void encoderLift(double speed, double raiseInches, double timeoutS) {
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
*/
    public VectorF navOffWall(VectorF trans, double robotAngle, VectorF offWall){
        return new VectorF((float) (trans.get(0) - offWall.get(0) * Math.sin(Math.toRadians(robotAngle)) - offWall.get(2) * Math.cos(Math.toRadians(robotAngle))), trans.get(1), (float) (trans.get(2) + offWall.get(0) * Math.cos(Math.toRadians(robotAngle)) - offWall.get(2) * Math.sin(Math.toRadians(robotAngle))));
    }

    public VectorF anglesFromTarget(VuforiaTrackableDefaultListener image){
        OpenGLMatrix CurrentLocation = image.getRobotLocation();
        float [] data = CurrentLocation.getData();
        float [] [] rotation = {{data[0], data[1]}, {data[4], data[5], data[6]}, {data[8], data[9], data[10]}};

        double thetaX = Math.atan2(rotation[2][1], rotation[2][2]);
        double thetaY = Math.atan2(-rotation[2][0], Math.sqrt(rotation[2][1] * rotation[2][1] + rotation[2][2] * rotation[2][2]));
        double thetaZ = Math.atan2(rotation[1][0], rotation[0][0]);
        return new VectorF((float)thetaX, (float)thetaY, (float)thetaZ);
    }


}
