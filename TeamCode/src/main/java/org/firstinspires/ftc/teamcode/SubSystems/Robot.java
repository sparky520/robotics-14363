package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.states.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    public Mecanum drivetrain;
    public slides slide;
    public slidesV2 slidev2;
    public finger Claw;
    public airplane Airplane;
    public wrist wrist;
    public arm Arm;

    //public airplane Airplane;
    public Robot(HardwareMap hardwareMap, Telemetry telemtry){
        drivetrain = new Mecanum(hardwareMap);
        Arm = new arm(hardwareMap, telemtry);
        slide = new slides(hardwareMap);
        slidev2 = new slidesV2(hardwareMap);
        Claw = new finger(hardwareMap, telemtry);
        Airplane = new airplane(hardwareMap);
        wrist = new wrist(hardwareMap);
    }

    public void initialize(){
        Arm.setPosition(armState.low);
        Claw.setPosition(armState.outtaking);
    }

}
