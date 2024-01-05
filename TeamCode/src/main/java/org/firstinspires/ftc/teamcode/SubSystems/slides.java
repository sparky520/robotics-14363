package org.firstinspires.ftc.teamcode.SubSystems;

import org.firstinspires.ftc.teamcode.robotConstants.slideConstants;
import org.firstinspires.ftc.teamcode.states.outtakeStates;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
public class slides {
    DcMotorEx leftSlide, rightSlide;
    double power = 0.65;
    public slides(HardwareMap hardwareMap){
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftOuttakeSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightOuttakeSlide");

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftSlide.setTargetPosition(0);
        rightSlide.setTargetPosition(0);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide.setPower(0.6);
        rightSlide.setPower(0.6);


    }

    public void setOuttakeSlidePosition(outtakeStates extensionState, outtakeStates outtakeSlidesState) {
        switch (extensionState) {
            case retracted:
                break;
                //positive goes up for left slide
            case etxending: {
                switch (outtakeSlidesState) {
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
                        leftSlide.setTargetPosition(600);
                        rightSlide.setTargetPosition(-600);

                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        extensionState = extensionState.extended;

                        leftSlide.setPower(power);
                        rightSlide.setPower(power);
                        break;
                    case LOWIN:
                        leftSlide.setTargetPosition(300);
                        rightSlide.setTargetPosition(-300);

                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        extensionState = extensionState.extended;

                        leftSlide.setPower(power);
                        rightSlide.setPower(power);
                        break;
                    case STATION:
                        leftSlide.setTargetPosition(0);
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