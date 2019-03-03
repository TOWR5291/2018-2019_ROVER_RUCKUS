package club.towr5291.libraries;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import club.towr5291.functions.FileLogger;

public class LibraryTensorFlowRoverRuckus {
    private VuforiaLocalizer vuforiaLocalizer;
    private FileLogger fileLogger;
    private HardwareMap hardwareMap;
    private TFObjectDetector tenFlow;
    private String TFOD_MODEL_ASSET;
    private String GOLD_LABEL;
    private String SILVER_LABEL;
    private String Position = "";

    public void initTensorFlow(VuforiaLocalizer vuforia, HardwareMap HWmap, FileLogger logger, String tenFlow_Asset, String GOLD_label, String SILVER_label, boolean ShowView) {
        this.vuforiaLocalizer = vuforia;
        this.fileLogger = logger;
        this.hardwareMap = HWmap;
        this.GOLD_LABEL = GOLD_label;
        this.SILVER_LABEL = SILVER_label;
        this.TFOD_MODEL_ASSET = tenFlow_Asset;

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());

            TFObjectDetector.Parameters tfodParameters;
            if (ShowView){
                tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            } else {
                tfodParameters = new TFObjectDetector.Parameters();
            }
            tenFlow = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tenFlow.loadModelFromAsset(TFOD_MODEL_ASSET, GOLD_LABEL, SILVER_LABEL);
        } else {
            fileLogger.writeEvent(3, "This Device is not compatible with Tensor Flow");
        }

        if (tenFlow != null){
            tenFlow.activate();
        }
    }

    public void tenFlowLoop (){
        if (tenFlow != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tenFlow.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                fileLogger.writeEvent(3, "# Object Detected", String.valueOf(updatedRecognitions.size()));

                if (updatedRecognitions.size() == 3) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(GOLD_LABEL)) {
                            goldMineralX = (int) recognition.getLeft();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }
                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                            fileLogger.writeEvent(3, "Gold Mineral Position = LEFT");
                            Position = "LEFT";
                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            fileLogger.writeEvent(3, "Gold Mineral Position = RIGHT");
                            Position = "RIGHT";
                        } else {
                            fileLogger.writeEvent(3, "Gold Mineral Position = CENTER");
                            Position = "CENTER";
                        }
                    }
                }
            }
        }
    }

    public String findGold() {
        return Position;
    }

    public void shutdown(){
        if (tenFlow != null){
            tenFlow.shutdown();
        }
    }
}