package club.towr5291.Concepts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;

import club.towr5291.functions.TOWR5291TextToSpeech;

@TeleOp(name = "Concept Text To Speech", group = "")
public class ConceptAndroidTextToSpeech extends LinearOpMode {

    private TOWR5291TextToSpeech towr5291TextToSpeech = new TOWR5291TextToSpeech();

    @Override
    public void runOpMode() throws InterruptedException {
        this.towr5291TextToSpeech.setSpeakingPitch(.25f);
        waitForStart();
        while (opModeIsActive()) {
            this.towr5291TextToSpeech.Speak("Hello World");

            while (opModeIsActive() && this.towr5291TextToSpeech.isSpeaking()) {
             telemetry.addData("Speaking.......", "Speaking.......");
            }

            Thread.sleep(5000);
        }
    }
}