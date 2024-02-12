package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.states.armState;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;

public class finger {
    private Servo leftClaw, rightClaw;
    Telemetry tel;
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
                leftClaw.setPosition(.76);
                rightClaw.setPosition(.76);
                break;
            case close:
                leftClaw.setPosition(.99);
                //rightClaw.setPosition(.51                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       );
                rightClaw.setPosition(.99);
                break;
        }
    }

    //auto
    public void setTape(){
        rightClaw.setPosition(.84);
    }
    public void afterTape(){
        rightClaw.setPosition(1);
    }
    //auto
    public void dropBoard(){
        leftClaw.setPosition(.82);
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
        rightClaw.setPosition(.8);
    }

}