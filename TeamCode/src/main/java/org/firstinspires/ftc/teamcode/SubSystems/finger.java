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
                leftClaw.setPosition(.15);
                rightClaw.setPosition(.15);
                break;
            case close:
                RClawState = finger.state.closed;
                LClawState = finger.state.closed;
                leftClaw.setPosition(.47);
                rightClaw.setPosition(.515);
                break;
        }
    }

    public void test(double pos){
        leftClaw.setPosition(pos);
        rightClaw.setPosition(pos);
    }
    public void closeLeft(){
        LClawState = finger.state.closed;
        leftClaw.setPosition(.47);
    }
    public void closeRight(){
        RClawState = finger.state.closed;
        rightClaw.setPosition(.515);
    }
    public void openLeft(){
        LClawState = finger.state.opened;
        leftClaw.setPosition(.15);
    }
    public void openRight(){
        RClawState = finger.state.opened;
        rightClaw.setPosition(.15);
    }

}