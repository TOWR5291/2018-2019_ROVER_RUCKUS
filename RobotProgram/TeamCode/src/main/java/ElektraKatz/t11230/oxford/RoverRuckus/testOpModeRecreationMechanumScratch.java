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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import club.towr5291.functions.TOWR5291Toggle;
/*
This is a recreation of an original OpMode. Product of Elektrakatz Programming Inc. Do not recreate a recreation.
That's just logical.
*/


@TeleOp(name="POV Drive Mechanum v3.0", group="Training")
//@Disabled
public class testOpModeRecreationMechanumScratch extends LinearOpMode {

    private DcMotor leftMotor1 = null;
    private DcMotor rightMotor1 = null;
    private DcMotor leftMotor2 = null;
    private DcMotor rightMotor2 = null;
    private DcMotor liftMotor = null;
    private DcMotor grabberMotor = null;
    private ColorSensor TestColor = null;
    private DistanceSensor TestDistance = null;
    private TouchSensor TestTouch = null;
    private GyroSensor TestGyro = null;
    Servo markerServo;
    Servo grabberServo;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        TOWR5291Toggle toggleButton1 = new TOWR5291Toggle(gamepad1.y);
        TOWR5291Toggle toggleButton2 = new TOWR5291Toggle(gamepad1.dpad_down);

        //These motors are named according to the RevisedMechanum configuration
        leftMotor1  = hardwareMap.get(DcMotor.class, "leftMotor1");
        leftMotor2  = hardwareMap.get(DcMotor.class, "leftMotor2");
        rightMotor1  = hardwareMap.get(DcMotor.class, "rightMotor1");
        rightMotor2  = hardwareMap.get(DcMotor.class, "rightMotor2");
        liftMotor = hardwareMap.get(DcMotor.class,"liftMotor");
        grabberMotor = hardwareMap.get(DcMotor.class,"grabberMotor");
        markerServo = hardwareMap.servo.get("markerServo");
        grabberServo = hardwareMap.servo.get("grabberServo");

        //This is optional, it's just for debug
        telemetry.addData("LeftMotorPower","null");
        telemetry.addData("RightMotorPower","null");
        telemetry.addData("Direction","Forward");
        telemetry.update();

        //This is setting up all the variables
        double turn = 0;
        double velocity = 0;
        double leftDrive = 0;
        double rightDrive = 0;
        double slowDown = 1.0;
        boolean reverse = false;

        //These motors MUST be set to reverse
        rightMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Slow down commands for motors
            if (gamepad1.dpad_down){
                if (slowDown == 1.0) {
                    slowDown = 0.5;
                }
                else {
                    slowDown = 1.0;
                }
            }
            toggleButton2.setDebounce(500);

            //This makes the robot crab crawl
            if (gamepad1.left_trigger > 0){

                leftMotor1.setPower(-gamepad1.left_trigger);
                leftMotor2.setPower(gamepad1.left_trigger);
                rightMotor1.setPower(gamepad1.left_trigger);
                rightMotor2.setPower(-gamepad1.left_trigger);
            }
            else{
                if (gamepad1.right_trigger > 0){

                    leftMotor1.setPower(gamepad1.right_trigger);
                    leftMotor2.setPower(-gamepad1.right_trigger);
                    rightMotor1.setPower(-gamepad1.right_trigger);
                    rightMotor2.setPower(gamepad1.right_trigger);
                }
            }
            //This is the POV drive engine
            turn = -gamepad1.right_stick_x;
            velocity = gamepad1.left_stick_y;

            leftDrive = Range.clip(velocity - turn,-slowDown,slowDown);
            rightDrive = Range.clip(velocity + turn,-slowDown,slowDown);


            leftMotor1.setPower(leftDrive);
            leftMotor2.setPower(leftDrive);
            rightMotor1.setPower(rightDrive);
            rightMotor2.setPower(rightDrive);

            //This controls the marker servo and lift arm
            liftMotor.setPower(-gamepad2.left_stick_y);
            markerServo.setPosition(gamepad2.left_stick_x);

            //This controls the grabber
            grabberMotor.setPower(-gamepad2.right_stick_y);
            grabberServo.setPosition(gamepad2.right_stick_x);

            //This inverts the controls on gamepad 1
            if (gamepad1.b){
                if (reverse){

                    leftMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
                    leftMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
                    rightMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
                    rightMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
                    reverse = false;
                }
                else {

                    leftMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
                    leftMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
                    rightMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
                    rightMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
                    reverse = true;
                }
            }
            toggleButton1.setDebounce(500);

            //This updates the stats every loop
            telemetry.addData("LeftMotorPower",leftDrive);
            telemetry.addData("RightMotorPower",rightDrive);
            if (reverse){
                telemetry.addData("Direction","Reverse");
            }
            else{
                telemetry.addData("Direction","Forward");
            }
            telemetry.update();
        }

    }
}

