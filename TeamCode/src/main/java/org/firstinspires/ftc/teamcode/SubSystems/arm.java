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
        telem.addLine("#1");
        telem.update();
        switch (state){

            case low:
                leftArm.setPosition(.974);
                rightArm.setPosition(.955);
                break;
            case medium:
                leftArm.setPosition(869);
                rightArm.setPosition(.85);
                break;
            case outtaking:
                leftArm.setPosition(.419);
                rightArm.setPosition(.4);
                break;
        }
    }


}