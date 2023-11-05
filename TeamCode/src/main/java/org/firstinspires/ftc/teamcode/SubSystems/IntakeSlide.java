package org.firstinspires.ftc.teamcode.SubSystems;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Commands.intakeSlidesStates;
import org.firstinspires.ftc.teamcode.Utilities.RobotConstants;

public class IntakeSlide {
    DcMotorEx intakeMotor;
    public IntakeSlide(HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotorEx.class,"intakeMotor");


        //Stop and reset encoders doesn't work?
        //intakeSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void on_off(double power){

            intakeMotor.setPower(power);



    }

}
