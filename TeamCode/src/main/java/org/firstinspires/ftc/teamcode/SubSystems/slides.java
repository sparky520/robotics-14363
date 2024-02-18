package org.firstinspires.ftc.teamcode.SubSystems;

import org.firstinspires.ftc.teamcode.states.outtakeStates;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
public class slides {
    DcMotorEx leftSlide, rightSlide;
    double power = 1;
    public int driftOffset = 0;
    public slides(HardwareMap hardwareMap){
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftOuttakeSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightOuttakeSlide");

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
                    case SHORT_AUTO:
                        leftSlide.setTargetPosition(200-driftOffset);
                        rightSlide.setTargetPosition(200+driftOffset);

                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        leftSlide.setPower(power);
                        rightSlide.setPower(power);

                        extensionState = extensionState.extended;
                        break;
                    case HIGH_AUTO:
                        leftSlide.setTargetPosition(1200-driftOffset);
                        rightSlide.setTargetPosition(1200+driftOffset);

                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        leftSlide.setPower(power);
                        rightSlide.setPower(power);

                        extensionState = extensionState.extended;
                        break;
                    case HIGHIN:
                        leftSlide.setTargetPosition(1340-driftOffset);
                        rightSlide.setTargetPosition(1300+driftOffset);

                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        //leftSlide.setPower(power);
                        rightSlide.setPower(power);

                        extensionState = extensionState.extended;
                        break;
                    case MEDIUMIN:
                        leftSlide.setTargetPosition(990-driftOffset);
                        rightSlide.setTargetPosition(950+driftOffset);

                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        extensionState = extensionState.extended;

                        //leftSlide.setPower(power);
                        rightSlide.setPower(power);
                        break;
                    case AUTO1:
                        leftSlide.setTargetPosition(350-driftOffset);
                        rightSlide.setTargetPosition(350+driftOffset);

                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        extensionState = extensionState.extended;

                        leftSlide.setPower(power);
                        rightSlide.setPower(power);
                        break;
                    case TOPSTACK:
                        leftSlide.setTargetPosition(45-driftOffset);
                        rightSlide.setTargetPosition(45+driftOffset);

                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        extensionState = extensionState.extended;

                        leftSlide.setPower(power);
                        rightSlide.setPower(power);
                        break;
                    case LOWIN:
                        leftSlide.setTargetPosition(440-driftOffset);
                        rightSlide.setTargetPosition(400+driftOffset);

                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        extensionState = extensionState.extended;

                        leftSlide.setPower(power);
                        rightSlide.setPower(power);
                        break;
                    case STATION:
                        leftSlide.setTargetPosition(30-driftOffset);
                        rightSlide.setTargetPosition(30+driftOffset);

                        //leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        extensionState = extensionState.retracted;

                        leftSlide.setPower(power);
                        rightSlide.setPower(power);
                        break;
                    case TELEOPSTATION:
                        leftSlide.setTargetPosition(15-driftOffset);
                        rightSlide.setTargetPosition(15+driftOffset);

                       // leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        extensionState = extensionState.retracted;

                        //leftSlide.setPower(power);
                        rightSlide.setPower(power);
                        break;
                    case RESET:
                        leftSlide.setTargetPosition(0-driftOffset);
                        rightSlide.setTargetPosition(0+driftOffset);

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
