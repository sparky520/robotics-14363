package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.states.armState;

public class wrist {
    Servo wrist;
    public wrist (HardwareMap hardwareMap){

        wrist = hardwareMap.servo.get("wrist");
    }

    public void setPosition(armState state){
        switch (state){
            case intakingCLAW:
                wrist.setPosition(.525);
                break;
            case outtaking:
                wrist.setPosition(.76);
                break;
        }
    }
    public void auto(){
        double test = DriveConstants.MAX_ANG_ACCEL;
        wrist.setPosition(.52);
    }

    public void test(double pos){
        wrist.setPosition(pos);
    }
    public void topStack(){
        wrist.setPosition(.505);
    }
    public void autoOuttake(){
        wrist.setPosition(.63);
    }


}
