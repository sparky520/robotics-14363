package org.firstinspires.ftc.teamcode.SubSystems;

import org.firstinspires.ftc.teamcode.states.outtakeStates;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class slides {
    private PID leftPID, rightPID;
    DcMotorEx leftSlide, rightSlide;
    double power = 0.65;
    public slides(HardwareMap hardwareMap){
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftOuttakeSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightOuttakeSlide");

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftSlide.setTargetPosition(0);
        rightSlide.setTargetPosition(0);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftPID = new PID(0.1,0,0,0,515);
        rightPID = new PID(0.1,0,0,0,515);


        leftSlide.setPower(0);
        rightSlide.setPower(0);

        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public void getOuttakeState(){
        return
    }
    public void powerSlides(double voltage, double override){

        int rightCurrent = rightSlide.getCurrentPosition();
        double power = rightPID.getCorrectionPosition(rightCurrent,voltage,state);
        if(override != 0){
            setTarget(rightCurrent);
            power = -override;
        }
        leftSlide.setPower(power);
        rightSlide.setPower(power);
    }
    public void setTarget(int position){
        leftPID.clearError();
        rightPID.clearError();

        leftPID.setTarget(position);
        rightPID.setTarget(position);

    }

    public void setOuttakeSlidePosition(outtakeStates extensionState, outtakeStates outtakeSlidesState) {
        switch (extensionState) {
            case retracted:
                break;
                //positive goes up for left slide
            case etxending: {
                switch (outtakeSlidesState) {
                    case AUTO_LONG_HIGH:
                        leftSlide.setTargetPosition(1200);
                        rightSlide.setTargetPosition(1200);

                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        leftSlide.setPower(power);
                        rightSlide.setPower(power);

                        extensionState = extensionState.extended;
                        break;
                    case AUTO_HIGH:
                        setTarget(1300);

                        extensionState = extensionState.extended;
                        break;
                    case HIGHIN:
                        leftSlide.setTargetPosition(1200);
                        rightSlide.setTargetPosition(-1200);

                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        leftSlide.setPower(power);
                        rightSlide.setPower(power);

                        extensionState = extensionState.extended;
                        break;
                    case MEDIUMIN:
                        setTarget(600);
                        extensionState = extensionState.extended;

                        break;
                    case LOWIN:
                        setTarget(300);


                        extensionState = extensionState.extended;


                        break;
                    case STATION:
                        leftSlide.setTargetPosition(-10);
                        rightSlide.setTargetPosition(0);

                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        extensionState = extensionState.retracted;

                        leftSlide.setPower(power);
                        rightSlide.setPower(power);
                        break;
                }
            }
            case extended:
                break;
        }
    }

}

