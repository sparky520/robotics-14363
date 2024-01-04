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
        telem = telemetryy;


    }

    public void setPosition(armState state){
        switch(state) {
            case low:
                leftArm.setPosition(.17);
                rightArm.setPosition(.17);
                break;
            case medium:
                leftArm.setPosition(.24);
                rightArm.setPosition(.24);
                break;
            case high:
                leftArm.setPosition(.33);
                rightArm.setPosition(.33);
                break;
            case outtaking:
                leftArm.setPosition(.685);
                rightArm.setPosition(.685);
                break;
        }
    }



}