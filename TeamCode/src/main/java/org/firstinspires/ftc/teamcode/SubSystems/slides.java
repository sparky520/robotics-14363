package org.firstinspires.ftc.teamcode.SubSystems;

import org.firstinspires.ftc.teamcode.robotConstants.slideConstants;
import org.firstinspires.ftc.teamcode.states.outtakeStates;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
public class slides {
    DcMotorEx leftSlide, rightSlide;
    double power = 0.9;
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

        leftSlide.setPower(0.3);
        rightSlide.setPower(0.3);


    }

    public void setOuttakeSlidePosition(outtakeStates extensionState, outtakeStates outtakeSlidesState) {
        switch (extensionState) {
            case retracted:
                break;
            case etxending: {
                switch (outtakeSlidesState) {
                    case HIGHIN:
                        leftSlide.setTargetPosition(1500);
                        rightSlide.setTargetPosition(1500);

                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        leftSlide.setPower(power);
                        rightSlide.setPower(power);

                        extensionState = extensionState.extended;
                        break;
                    case MEDIUMIN:
                        leftSlide.setTargetPosition(800);
                        rightSlide.setTargetPosition(800);

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
