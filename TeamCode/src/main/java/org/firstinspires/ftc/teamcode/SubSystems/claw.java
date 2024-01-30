package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.states.armState;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
public class claw {
    private Servo leftClaw, rightClaw;
    Telemetry tel;
    public claw (HardwareMap hardwareMap, Telemetry telemetry)
    {
        tel = telemetry;
        leftClaw = hardwareMap.servo.get("leftClaw");
        rightClaw = hardwareMap.servo.get("rightClaw");
        rightClaw.setDirection(Servo.Direction.REVERSE);
    }

    public void setPosition(armState state){
        switch (state){
            case open:
                leftClaw.setPosition(.82);
                rightClaw.setPosition(.82);
                break;
            case close:
                leftClaw.setPosition(.935);
                //rightClaw.setPosition(.51                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       );
                rightClaw.setPosition(.92);
                break;
        }
    }

    //auto
    public void setTape(){
        rightClaw.setPosition(.8);
    }
    public void afterTape(){
        rightClaw.setPosition(1);
    }
    //auto
    public void dropBoard(){
        leftClaw.setPosition(.82);
    }
    public void maxClose(){
        leftClaw.setPosition(.98);
        rightClaw.setPosition(.98);
    }
    public void stack()
    {
        leftClaw.setPosition(1);
        rightClaw.setPosition(.65);
    }

}