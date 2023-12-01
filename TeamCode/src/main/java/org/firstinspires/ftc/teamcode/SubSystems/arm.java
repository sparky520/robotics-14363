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

            case intaking:
                telem.addLine("#3");
                telem.update();
                leftArm.setPosition(-.8);
                rightArm.setPosition(.8);
                break;
            case outtaking:
                //outtaking
                leftArm.setPosition(.8);
                rightArm.setPosition(-.8);
                break;
        }
    }


}