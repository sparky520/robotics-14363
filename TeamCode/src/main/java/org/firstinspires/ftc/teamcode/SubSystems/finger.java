package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.states.armState;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;

public class finger {
    private Servo leftClaw, rightClaw;
    Telemetry tel;
    public enum state{
        opened,closed,
    }
    public state LClawState = state.opened;
    public state RClawState = state.opened;
    public finger(HardwareMap hardwareMap, Telemetry telemetry)
    {
        tel = telemetry;
        leftClaw = hardwareMap.servo.get("leftClaw");
        rightClaw = hardwareMap.servo.get("rightClaw");
        rightClaw.setDirection(Servo.Direction.REVERSE);
    }

    public void setPosition(armState state){
        switch (state){
            case open:
                leftClaw.setPosition(6);
                rightClaw.setPosition(.52);
                break;
            case close:
                leftClaw.setPosition(.82);
                rightClaw.setPosition(.83);
                break;
        }
    }

    public void test(double pos){
        rightClaw.setPosition(pos);
        //rightClaw.setPosition(pos);
    }
    public void closeLeft(){
        LClawState = finger.state.closed;
        leftClaw.setPosition(.82);
    }
    public void closeRight(){
        RClawState = finger.state.closed;
        rightClaw.setPosition(.83);
    }
    public void openLeft(){
        LClawState = finger.state.opened;
        leftClaw.setPosition(.6);
    }
    public void openRight(){
        RClawState = finger.state.opened;
        rightClaw.setPosition(.52);
    }

}