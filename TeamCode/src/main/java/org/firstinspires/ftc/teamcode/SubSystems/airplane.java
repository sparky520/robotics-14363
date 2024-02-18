package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.states.armState;

public class airplane {
    Servo airplane;
    public airplane (HardwareMap hardwareMap){

        airplane = hardwareMap.servo.get("airplane");
    }

    public void setPosition(armState state){
        switch (state){
            case airplaneInit:
                airplane.setPosition(.22);
                break;
            case airplaneLaunch:
                airplane.setPosition(.4);
                break;
        }
    }
}
