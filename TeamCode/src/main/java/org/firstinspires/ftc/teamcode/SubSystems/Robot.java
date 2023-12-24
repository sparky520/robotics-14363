package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.states.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
public class Robot {
    public Mecanum drivetrain;
    public arm Arm;
    public slides slide;
    public claw Claw;

    //public airplane Airplane;
    public Robot(HardwareMap hardwareMap, Telemetry telemtry){
        drivetrain = new Mecanum(hardwareMap);
        //Arm = new arm(hardwareMap, telemtry);
        //slide = new slides(hardwareMap);
        //Claw = new claw(hardwareMap, telemtry);
        //Airplane = new airplane(hardwareMap);
    }

}
