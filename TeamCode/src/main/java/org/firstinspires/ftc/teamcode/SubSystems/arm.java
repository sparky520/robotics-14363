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
                leftArm.setPosition(1.05);
                rightArm.setPosition(1.05);
                break;
            case medium:
                leftArm.setPosition(.93);
                rightArm.setPosition(.93);
                break;
            case high:
                leftArm.setPosition(.11);
                rightArm.setPosition(.11);
                break;
            case outtaking:
                leftArm.setPosition(.48);
                rightArm.setPosition(.48);
                break;
        }
    }



}