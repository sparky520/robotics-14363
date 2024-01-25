package org.firstinspires.ftc.teamcode.SubSystems;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.states.armState;

public class arm {
    private Servo leftArm, rightArm;
    public Telemetry telem;
    motionProfile profile;
    public arm (HardwareMap hardwareMap, Telemetry telemetryy)
    {
        leftArm = hardwareMap.servo.get("leftArm");
        rightArm = hardwareMap.servo.get("rightArm");
        telem = telemetryy;


    }

    public void setPosition(armState state){
        switch(state) {
            case low:
                leftArm.setPosition(1);
                rightArm.setPosition(1);
                break;
            case medium:
                leftArm.setPosition(.93);
                rightArm.setPosition(.93);
                break;

            case high:
                //for auto
                leftArm.setPosition(.11);
                rightArm.setPosition(.11);
                break;
            case outtaking:
                leftArm.setPosition(.478);
                rightArm.setPosition(.478);
                //leftArm.setPosition(.485);
                //rightArm.setPosition(.485);
                break;
            case outtaking2:
                leftArm.setPosition(.49);
                rightArm.setPosition(.49);
        }
    }

    public void longAuto(){
        leftArm.setPosition(.2);
        rightArm.setPosition(.2);
    }
    public void topStack(){
        leftArm.setPosition(.98);
        rightArm.setPosition(.98);
    }


}