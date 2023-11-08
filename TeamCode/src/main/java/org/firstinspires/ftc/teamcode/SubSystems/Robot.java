package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.states.*;
public class Robot {
    public Mecanum drivetrain;
    public arm Arm;

    public Robot(HardwareMap hardwareMap){
        drivetrain = new Mecanum(hardwareMap);
        Arm = new arm(hardwareMap);
    }
}
