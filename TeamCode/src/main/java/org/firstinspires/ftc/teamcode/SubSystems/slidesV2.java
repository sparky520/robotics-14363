package org.firstinspires.ftc.teamcode.SubSystems;

import org.firstinspires.ftc.teamcode.states.outtakeStates;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
public class slidesV2 {
    public DcMotorEx leftSlide, rightSlide;
    double power = .3;
    public int driftOffset = 0;
    public boolean goingUp;
    public outtakeStates currentSlideState = outtakeStates.TELEOPSTATION;

    public slidesV2(HardwareMap hardwareMap) {
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftOuttakeSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightOuttakeSlide");

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    public boolean setSlidePos(){
        switch (currentSlideState){
            case LOWIN:
                if (goingUp){
                    if (leftSlide.getCurrentPosition() > 100){
                        setSlidePower(.001);
                        return true;
                    }
                    setSlidePower(power);
                }else{
                    if (leftSlide.getCurrentPosition() < 100){
                        setSlidePower(.001);
                        return true;
                    }
                    setSlidePower(-power);
                }
                return false;
            case MEDIUMIN:
                if (goingUp){
                    if (leftSlide.getCurrentPosition() > 1100){
                        setSlidePower(.001);
                        return true;
                    }
                    setSlidePower(power);
                }else{
                    if (leftSlide.getCurrentPosition() < 100){
                        setSlidePower(.001);
                        return true;
                    }
                    setSlidePower(-power);
                }
                return false;
        }
        return false;
    }
    public void isGoingUp(int targetPos){
        if(leftSlide.getCurrentPosition() < targetPos){
            goingUp = true;
        }
    }
    public void setSlidePower(double pow){
        leftSlide.setPower(pow);
        rightSlide.setPower(pow);
    }
}