package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.*;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.states.*;
import org.firstinspires.ftc.teamcode.SubSystems.*;
import org.firstinspires.ftc.teamcode.SubSystems.arm;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode
{
    private GamepadEx driver, operator;
    private Robot robot;
    private Mecanum driveTrain;

    private Servo leftArm, rightArm;
    Mecanum wheels;
    @Override
    public void init()
    {
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        robot = new Robot(hardwareMap);


        leftArm = hardwareMap.servo.get("leftArm");
        rightArm = hardwareMap.servo.get("rightArm");
    }
    @Override
    public void loop() {
        driver.readButtons();
        operator.readButtons();

        robot.drivetrain.fieldCentric(driver);

        if(gamepad1.y) {
            wheels.resetIMU();
        }
/*
        if (gamepad1.dpad_up){
            //robot.Arm.setPosition(armState.intaking);
            leftArm.setPosition(-1);
            rightArm.setPosition(1);
        }
        if (gamepad1.dpad_down){
            //robot.Arm.setPosition(armState.outtaking);
            leftArm.setPosition(1);
            rightArm.setPosition(-1);
        }
        */

        Outtake outtakeSlide = new Outtake(hardwareMap);
        if (gamepad1.dpad_up){robot.outtake.setSlidePosition(outtakeStates.high,outtakeStates.etxending);}
        if (gamepad1.dpad_left){outtakeSlide.setSlidePosition(outtakeStates.medium,outtakeStates.etxending);}
        if (gamepad1.dpad_down){outtakeSlide.setSlidePosition(outtakeStates.low,outtakeStates.etxending);}



    }
}