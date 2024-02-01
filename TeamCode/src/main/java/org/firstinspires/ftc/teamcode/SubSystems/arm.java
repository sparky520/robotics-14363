package org.firstinspires.ftc.teamcode.SubSystems;


import org.firstinspires.ftc.teamcode.drive.DriveConstants;
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
        leftArm.setDirection(Servo.Direction.REVERSE);
        rightArm.setDirection(Servo.Direction.REVERSE);
        telem = telemetryy;


    }

    public void setPosition(armState state){
        double rightOffSet = 0;
        double leftOffSet = 0.01;
        switch(state) {
            case low:
                intake();
                break;
            case medium:
                medium();
                break;
            case outtaking:
                outtake();
                break;
        }
    }


    public void outtake(){
        double ofset = .011;

        leftArm.setPosition(.25 + ofset);
        rightArm.setPosition(.25);
    }
    public void intake(){
        double ofset;
        ofset = .011;
        leftArm.setPosition(.955 + ofset);
        rightArm.setPosition(.955);
    }
    public void medium(){
        double ofset;
        ofset = .014;

        leftArm.setPosition(.845 + ofset);
        rightArm.setPosition(.845);
    }
    public void topStack(){
        double ofset;
        ofset = .014;
        double test = DriveConstants.MAX_ANG_VEL;
        leftArm.setPosition(.92 + ofset);
        rightArm.setPosition(.92);
    }

}