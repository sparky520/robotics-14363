package org.firstinspires.ftc.teamcode.SubSystems;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.states.armState;

public class arm {
    private Servo leftArm, rightArm;
    public Telemetry telem;
    public arm (HardwareMap hardwareMap, Telemetry telemetryy)
    {
        leftArm = hardwareMap.servo.get("leftArm");
        rightArm = hardwareMap.servo.get("rightArm");
        leftArm.setDirection(Servo.Direction.REVERSE);
        rightArm.setDirection(Servo.Direction.REVERSE);
        telem = telemetryy;


    }

    public void setPosition(armState state){
        double rightOffSet = 0;
        double leftOffSet = 0.01;
        switch(state) {
            case low:
                leftArm.setPosition(.91 - leftOffSet);
                rightArm.setPosition(.91 - rightOffSet);
                break;
            case medium:
                leftArm.setPosition(.75- leftOffSet);
                rightArm.setPosition(.75 - rightOffSet);
                break;

            case high:
                //for auto
                leftArm.setPosition(-1- leftOffSet);
                rightArm.setPosition(-1 - rightOffSet);
                break;
            case outtaking:
                leftArm.setPosition(-.15- leftOffSet);
                rightArm.setPosition(-.15- rightOffSet);
                //leftArm.setPosition(.485);
                //rightArm.setPosition(.485);
                break;
            case outtaking2:
                leftArm.setPosition(.49 - leftOffSet);
                rightArm.setPosition(.49 - rightOffSet);
        }
    }

    public void longAuto(){
        leftArm.setPosition(.2);
        rightArm.setPosition(.2);
    }
    public void topStack(){
        leftArm.setPosition(.9);
        rightArm.setPosition(.9);
    }

}