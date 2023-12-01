package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.states.armState;

public class claw {
    private Servo leftClaw, rightClaw;
    public claw (HardwareMap hardwareMap)
    {
        leftClaw = hardwareMap.servo.get("leftClaw");
        rightClaw = hardwareMap.servo.get("rightClaw");
    }

    public void setPosition(armState state){
        switch (state){
            case intaking:
                leftClaw.setPosition(1);
                rightClaw.setPosition(1);
                break;
            case outtaking:
                leftClaw.setPosition(.8);
                rightClaw.setPosition(.8);
                break;
        }
    }


}