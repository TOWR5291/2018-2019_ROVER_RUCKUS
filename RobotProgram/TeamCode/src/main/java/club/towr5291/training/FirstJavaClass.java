package club.towr5291.training;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "My First Java Class", group = "Learning")
public class FirstJavaClass extends LinearOpMode {
    private DcMotor leftMotor1 = null;
    private DcMotor leftMotor2 = null;
    private DcMotor rightMotor1 = null;
    private DcMotor rightMotor2 = null;

    @Override
    public void runOpMode() throws InterruptedException {

        leftMotor1 = hardwareMap.dcMotor.get("leftMotor1");
        leftMotor2 = hardwareMap.dcMotor.get("leftMotor2");
        rightMotor1 = hardwareMap.dcMotor.get("rightMotor1");
        rightMotor2 = hardwareMap.dcMotor.get("rightMotor2");

        //Before Driver Hits Play
        waitForStart();
        //After Driver Hits Play

        while(opModeIsActive()){
            //This loop runs until the driver hits stop

            double gamePad1LeftJoyStickValY = -gamepad1.left_stick_y;
            leftMotor1.setPower(gamePad1LeftJoyStickValY);
            leftMotor2.setPower(gamePad1LeftJoyStickValY);
            rightMotor1.setPower(gamePad1LeftJoyStickValY);
            rightMotor2.setPower(gamePad1LeftJoyStickValY);
        }
    }
}
