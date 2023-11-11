package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.states.armState;

public class claw {
    private Servo leftClaw, rightClaw;
    public claw (HardwareMap hardwareMap)
    {
        leftClaw = hardwareMap.servo.get("leftClaw");
        //rightClaw = hardwareMap.servo.get("rightClaw");
    }

    public void setPosition(armState state){
        switch (state){
            case intaking:
                leftClaw.setPosition(.5);
                //rightClaw.setPosition(0.1);
                break;
            case outtaking:
                leftClaw.setPosition(.6);
                //rightClaw.setPosition(-0.1);
                break;
        }
    }


}