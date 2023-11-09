package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.states.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
public class Robot {
    public Mecanum drivetrain;
    public arm Arm;
    public Outtake outtake;
    public Robot(HardwareMap hardwareMap, Telemetry telemtry){
        drivetrain = new Mecanum(hardwareMap);
        Arm = new arm(hardwareMap);
        outtake = new Outtake(hardwareMap);
    }
}
