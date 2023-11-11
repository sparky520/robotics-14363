package org.firstinspires.ftc.teamcode.SubSystems;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.states.armState;

public class arm {
    private Servo leftArm, rightArm;
    public arm (HardwareMap hardwareMap, Telemetry telemetry)
    {
        Telemetry tel = telemetry;
        leftArm = hardwareMap.servo.get("leftArm");
        rightArm = hardwareMap.servo.get("rightArm");



    }

    public void setPosition(armState state){

        switch (state){
            case intaking:
                leftArm.setPosition(-0.5);
                rightArm.setPosition(0.5);
                break;
            case outtaking:
                leftArm.setPosition(0.8);
                rightArm.setPosition(-0.8);
                break;
        }
    }


}