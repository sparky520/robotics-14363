package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.states.armState;

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
            case intakingCLAW:
                tel.addLine("bobross");
                tel.update();
                leftClaw.setPosition(.82);
                rightClaw.setPosition(.82);
                break;
            case outtaking:
                leftClaw.setPosition(.91);
                //rightClaw.setPosition(.51                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       );
                rightClaw.setPosition(.91);
                break;
        }
    }

    //auto
    public void setTape(){
        leftClaw.setPosition(.91);
    }
    //auto
    public void dropBoard(){
        rightClaw.setPosition(.91);
    }

}