package org.firstinspires.ftc.teamcode.SubSystems;

import org.firstinspires.ftc.teamcode.states.outtakeStates;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
public class slides {
    public DcMotorEx leftSlide, rightSlide;
    double power = .75;
    public int driftOffset = 0;
    public outtakeStates currentSlideState = outtakeStates.TELEOPSTATION;
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
                        leftSlide.setTargetPosition(200-driftOffset);
                        rightSlide.setTargetPosition(200-driftOffset);
                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftSlide.setPower(power);
                        rightSlide.setPower(power);
                        currentSlideState = outtakeStates.SHORT_AUTO;
                        if (leftSlide.getCurrentPosition() < 200){
                            goingUp = true;
                        }
                        else{
                            goingUp = false;
                        }
                        extensionState = extensionState.extended;
                        break;
                    case MEDIUM_AUTO:
                        leftSlide.setTargetPosition(350-driftOffset);
                        rightSlide.setTargetPosition(350-driftOffset);
                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftSlide.setPower(power);
                        rightSlide.setPower(power);
                        currentSlideState = outtakeStates.SHORT_AUTO;
                        if (leftSlide.getCurrentPosition() < 200){
                            goingUp = true;
                        }
                        else{
                            goingUp = false;
                        }
                        extensionState = extensionState.extended;
                        break;
                    case HIGH_AUTO:
                        leftSlide.setTargetPosition(1200-driftOffset);
                        rightSlide.setTargetPosition(1200-driftOffset);
                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftSlide.setPower(power);
                        rightSlide.setPower(power);
                        currentSlideState = outtakeStates.HIGH_AUTO;
                        if (leftSlide.getCurrentPosition() < 200){
                            goingUp = true;
                        }
                        else{
                            goingUp = false;
                        }
                        extensionState = extensionState.extended;
                        break;
                    case HIGHIN:
                        leftSlide.setTargetPosition(1300-driftOffset);
                        rightSlide.setTargetPosition(1300-driftOffset);
                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftSlide.setPower(power);
                        rightSlide.setPower(power);
                        currentSlideState = outtakeStates.HIGHIN;
                        if (leftSlide.getCurrentPosition() < 200){
                            goingUp = true;
                        }
                        else{
                            goingUp = false;
                        }
                        extensionState = extensionState.extended;
                        break;
                    case MEDIUMIN:
                        leftSlide.setTargetPosition(950-driftOffset);
                        rightSlide.setTargetPosition(950-driftOffset);
                        extensionState = extensionState.extended;
                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftSlide.setPower(power);
                        rightSlide.setPower(power);
                        currentSlideState = outtakeStates.MEDIUMIN;
                        if (leftSlide.getCurrentPosition() < 200){
                            goingUp = true;
                        }
                        else{
                            goingUp = false;
                        }
                        break;
                    case AUTO1:
                        leftSlide.setTargetPosition(500-driftOffset);
                        rightSlide.setTargetPosition(500-driftOffset);
                        extensionState = extensionState.extended;
                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftSlide.setPower(power);
                        rightSlide.setPower(power);
                        currentSlideState = outtakeStates.AUTO1;
                        break;
                    case LOW_AUTO:
                        leftSlide.setTargetPosition(250-driftOffset);
                        rightSlide.setTargetPosition(250-driftOffset);
                        extensionState = extensionState.extended;
                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftSlide.setPower(power);
                        rightSlide.setPower(power);
                        currentSlideState = outtakeStates.LOW_AUTO;
                        break;
                    case TOPSTACK:
                        leftSlide.setTargetPosition(45-driftOffset);
                        rightSlide.setTargetPosition(45-driftOffset);
                        extensionState = extensionState.extended;
                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftSlide.setPower(power);
                        rightSlide.setPower(power);
                        currentSlideState = outtakeStates.TOPSTACK;
                        break;
                    case LOWIN:
                        leftSlide.setTargetPosition(400-driftOffset);
                        rightSlide.setTargetPosition(400-driftOffset);
                        extensionState = extensionState.extended;
                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftSlide.setPower(power);
                        rightSlide.setPower(power);
                        currentSlideState = outtakeStates.LOWIN;
                        if (leftSlide.getCurrentPosition() < 200){
                            goingUp = true;
                        }
                        else{
                            goingUp = false;
                        }
                        break;
                    case TELEOPSTATION:
                        leftSlide.setTargetPosition(15-driftOffset);
                        rightSlide.setTargetPosition(15-driftOffset);
                        extensionState = extensionState.retracted;
                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        currentSlideState = outtakeStates.TELEOPSTATION;
                        leftSlide.setPower(power);
                        rightSlide.setPower(power);
                        if (leftSlide.getCurrentPosition() < 200){
                            goingUp = true;
                        }
                        else{
                            goingUp = false;
                        }
                        break;
                    case RESET:
                        leftSlide.setTargetPosition(1-driftOffset);
                        rightSlide.setTargetPosition(1-driftOffset);
                        extensionState = extensionState.retracted;
                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftSlide.setPower(power);
                        rightSlide.setPower(power);
                        currentSlideState = outtakeStates.RESET;
                        break;
                }
            }
            case extended:
                break;

        }
    }

    public boolean checkSlidePos(){
        switch (currentSlideState){
            case TELEOPSTATION:
                if (goingUp && leftSlide.getCurrentPosition() > 100){
                    leftSlide.setPower(.001);
                    rightSlide.setPower(.001);
                    return true;
                }
                break;
            case MEDIUMIN:
                if (goingUp && leftSlide.getCurrentPosition() > 950){
                    leftSlide.setPower(.001);
                    rightSlide.setPower(.001);
                    return true;
                }
                break;
        }
        return false;
    }

}
