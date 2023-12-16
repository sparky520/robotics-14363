package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class airplane {
    Servo airplane;
    public airplane (HardwareMap hardwareMap){
        airplane = hardwareMap.servo.get("airplane");
    }

    public void launchAirplane()
    {
        airplane.setPosition(1);
    }
}
