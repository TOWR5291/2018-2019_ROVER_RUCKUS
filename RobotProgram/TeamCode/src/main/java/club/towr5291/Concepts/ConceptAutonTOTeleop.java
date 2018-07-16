package club.towr5291.Concepts;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

import club.towr5291.opmodes.OpModeMasterLinear;

/**
 * Created by ianhaden on 1/6/18.
 */
@Autonomous(name="Auton to Teleop", group="Concept")
@Disabled
public class ConceptAutonTOTeleop extends OpModeMasterLinear {

    private OpMode onStop = this;
    private OpModeManagerImpl opModeManager;
    private String TeleOpMode = "Concept Logging";

    @Override
    public void runOpMode() throws InterruptedException {

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //the main loop.  this is where the action happens
        while (opModeIsActive()) {


        }
        //switch opmode to teleop
        opModeManager = (OpModeManagerImpl) onStop.internalOpModeServices;
        opModeManager.initActiveOpMode(TeleOpMode);

    }


}
