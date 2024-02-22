package org.firstinspires.ftc.teamcode.SubSystems;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.states.outtakeStates;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
public class slides {
    public DcMotorEx leftSlide, rightSlide;
    double power = .75;
    public int driftOffset = 0;
    public outtakeStates currentSlideState = outtakeStates.STATION;
    public boolean goingUp = false;
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
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    public void setOuttakeSlidePosition(outtakeStates extensionState, outtakeStates outtakeSlidesState) {
        switch (extensionState) {
            case retracted:
                break;
            case etxending: {
                switch (outtakeSlidesState) {

                    case SHORT_AUTO:
                        leftSlide.setTargetPosition(200);
                        rightSlide.setTargetPosition(200);
                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftSlide.setPower(power);
                        rightSlide.setPower(power);
                        extensionState = extensionState.extended;
                        break;
                    case INBETWEEN_SHORT_AND_LONG_AUTO:
                        leftSlide.setTargetPosition(350);
                        rightSlide.setTargetPosition(350);
                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftSlide.setPower(power);
                        rightSlide.setPower(power);
                        extensionState = extensionState.extended;
                        break;
                    case HIGH:
                        leftSlide.setTargetPosition(1300);
                        rightSlide.setTargetPosition(1300);
                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftSlide.setPower(power);
                        rightSlide.setPower(power);
                        currentSlideState = outtakeStates.HIGH;
                        extensionState = extensionState.extended;
                        break;
                    case MEDIUM:
                        leftSlide.setTargetPosition(800);
                        rightSlide.setTargetPosition(800);
                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftSlide.setPower(power);
                        rightSlide.setPower(power);
                        currentSlideState = outtakeStates.MEDIUM;
                        extensionState = extensionState.extended;
                        break;
                    case TWO_PLUS_TWO_OUTTAKE:
                        leftSlide.setTargetPosition(600);
                        rightSlide.setTargetPosition(600);
                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftSlide.setPower(power);
                        rightSlide.setPower(power);
                        currentSlideState = outtakeStates.MEDIUM;
                        extensionState = extensionState.extended;
                        break;
                    case LONG_AUTO:
                        leftSlide.setTargetPosition(500);
                        rightSlide.setTargetPosition(500);
                        extensionState = extensionState.extended;
                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftSlide.setPower(power);
                        rightSlide.setPower(power);
                        break;
                    case TOPSTACK:
                        leftSlide.setTargetPosition(15);
                        rightSlide.setTargetPosition(15);
                        extensionState = extensionState.extended;
                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftSlide.setPower(power);
                        rightSlide.setPower(power);
                        break;
                    case LOW:
                        leftSlide.setTargetPosition(500);
                        rightSlide.setTargetPosition(500);
                        extensionState = extensionState.extended;
                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftSlide.setPower(power);
                        rightSlide.setPower(power);
                        break;
                    case STATION:
                        leftSlide.setTargetPosition(-10);
                        rightSlide.setTargetPosition(-10);
                        extensionState = extensionState.retracted;
                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        currentSlideState = outtakeStates.STATION;
                        leftSlide.setPower(power);
                        rightSlide.setPower(power);
                        break;
                    case ZERO:
                        leftSlide.setTargetPosition(1);
                        rightSlide.setTargetPosition(1);
                        extensionState = extensionState.retracted;
                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftSlide.setPower(power);
                        rightSlide.setPower(power);
                        currentSlideState = outtakeStates.ZERO;
                        break;
                }
            }
            case extended:
                break;

        }
    }
     public void resetEncoders(){
         leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     }
}
