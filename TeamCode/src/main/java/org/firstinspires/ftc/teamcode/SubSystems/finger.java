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
                leftClaw.setPosition(.45);
                rightClaw.setPosition(.47);
                break;
            case close:
                leftClaw.setPosition(.12);
                rightClaw.setPosition(.16);
                break;
        }
    }

    public void test(double pos){
        leftClaw.setPosition(pos);
        rightClaw.setPosition(pos);
    }
    public void closeLeft(){
        LClawState = finger.state.closed;
        leftClaw.setPosition(.12);
    }
    public void closeRight(){
        RClawState = finger.state.closed;
        rightClaw.setPosition(.16);
    }
    public void openLeft(){
        LClawState = finger.state.opened;
        leftClaw.setPosition(.45);
    }
    public void openRight(){
        RClawState = finger.state.opened;
        rightClaw.setPosition(.47);
    }

}