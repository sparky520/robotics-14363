package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.states.armState;

public class arm {
    private Servo leftArm, rightArm;
    public arm (HardwareMap hardwareMap)
    {
        leftArm = hardwareMap.servo.get("leftArm");
        rightArm = hardwareMap.servo.get("rightArm");



    }

    public void setPosition(armState state){
        switch (state){
            case intaking:
                leftArm.setPosition(-1);
                rightArm.setPosition(1);
                break;
            case outtaking:
                leftArm.setPosition(1);
                rightArm.setPosition(-1);
                break;
        }
    }


}