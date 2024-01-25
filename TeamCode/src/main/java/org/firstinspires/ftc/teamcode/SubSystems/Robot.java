package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.states.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
public class Robot {
    public double worldX,worldY,worldHeading;
    public double movementX,movementY, movementTurn;
    public Mecanum drivetrain;
    public arm Arm;
    public slides slide;
    public claw Claw;
    public airplane Airplane;
    public wrist wrist;

    //public airplane Airplane;
    public Robot(HardwareMap hardwareMap, Telemetry telemtry){
        worldX = 0;
        worldY = 0;
        worldHeading = 0;
        movementX = 0;
        movementY = 0;
        drivetrain = new Mecanum(hardwareMap);
        Arm = new arm(hardwareMap, telemtry);
        slide = new slides(hardwareMap);
        Claw = new claw(hardwareMap, telemtry);
        Airplane = new airplane(hardwareMap);
        wrist = new wrist(hardwareMap);
    }

    public void initialize(){
        Arm.setPosition(armState.low);
        slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
        Claw.setPosition(armState.outtaking);
    }

}
