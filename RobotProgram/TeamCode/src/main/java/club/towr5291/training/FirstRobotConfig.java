package club.towr5291.training;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FirstRobotConfig {
    public DcMotor leftMotor1 = null;
    public DcMotor leftMotor2 = null;
    public DcMotor rightMotor1 = null;
    public DcMotor rightMotor2 = null;

    private HardwareMap hardwareMap;

    public FirstRobotConfig(HardwareMap hwMap){
        this.hardwareMap = hwMap;
    }
}