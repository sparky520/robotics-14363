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
        double ofset = -.012;

        leftArm.setPosition(.15 + ofset);
        rightArm.setPosition(.15);
    }
    public void test(double pos, double offset){
        leftArm.setPosition(pos + offset);
        rightArm.setPosition(pos);

    }
    public void intake(){
        double ofset;
        ofset = -.011;
        leftArm.setPosition(.837 + ofset);
        rightArm.setPosition(.837);
    }
    public void medium(){
        double ofset;
        ofset = -.011;

        leftArm.setPosition(.75 + ofset);
        rightArm.setPosition(.75);
    }
    public void highOuttake(){
        double ofset;
        ofset = -.012;

        leftArm.setPosition(.22 + ofset);
        rightArm.setPosition(.22);
    }
    public void half(){
        double ofset;
        ofset = -.011;

        leftArm.setPosition(.7 + ofset);
        rightArm.setPosition(.7);
    }
    public void stackDrop(){
        double offset = -.023;
        leftArm.setPosition(.24 + offset);
        rightArm.setPosition(.24);
    }
    public void topStack(){
        double ofset;
        ofset = -.014;
        leftArm.setPosition(.815 + ofset);
        rightArm.setPosition(.815);
    }

    public void autoOuttake(){
        double offset = -.012;
        leftArm.setPosition(.2 + offset);
        rightArm.setPosition(.2);
    }

}