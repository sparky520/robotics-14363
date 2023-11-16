package org.firstinspires.ftc.teamcode.SubSystems;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.states.armState;

public class arm {

    private CRServo leftArm,rightArm;
    Telemetry tel;
    public arm (HardwareMap hardwareMap, Telemetry telemetry)
    {
        tel = telemetry;
        leftArm = hardwareMap.crservo.get("leftArm");
        rightArm = hardwareMap.crservo.get("rightArm");



    }

    public void setPosition(armState state){
        tel.addLine("#1");
        tel.update();
        switch (state){
            case intaking:
                tel.addLine("#2");
                tel.update();
                leftArm.setPower(.65);
                rightArm.setPower(.45);
                //Thread.sleep(5000);

                break;
            case outtaking:
                leftArm.setPower(.8);
                rightArm.setPower(-.80);
                break;
        }
    }


}