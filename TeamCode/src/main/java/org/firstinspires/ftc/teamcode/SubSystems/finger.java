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
                RClawState = finger.state.opened;
                LClawState = finger.state.opened;
                leftClaw.setPosition(.66);
                rightClaw.setPosition(.18);
                break;
            case close:
                RClawState = finger.state.closed;
                LClawState = finger.state.closed;
                leftClaw.setPosition(.99);
                rightClaw.setPosition(.51);
                break;
        }
    }

    public void test(double pos){
        leftClaw.setPosition(pos);
        rightClaw.setPosition(pos);
    }
    public void closeLeft(){
        LClawState = finger.state.closed;
        leftClaw.setPosition(.99);
    }
    public void closeRight(){
        RClawState = finger.state.closed;
        rightClaw.setPosition(.51);
    }
    public void openLeft(){
        LClawState = finger.state.opened;
        leftClaw.setPosition(.7);
    }
    public void openRight(){
        RClawState = finger.state.opened;
        rightClaw.setPosition(.18);
    }
    public void setTape(){
        rightClaw.setPosition(.51);
    }
    public void afterTape(){
        rightClaw.setPosition(.51);
    }
    //auto
    public void dropBoard(){
        leftClaw.setPosition(.72);
    }
    public void maxClose(){
        leftClaw.setPosition(1);
        rightClaw.setPosition(1);
    }
    public void stack()
    {
        double test = DriveConstants.MAX_ANG_VEL;
        double test2 = DriveConstants.MAX_ANG_ACCEL;
        leftClaw.setPosition(1);
        //.84
        //.97
        rightClaw.setPosition(.7);
    }

}