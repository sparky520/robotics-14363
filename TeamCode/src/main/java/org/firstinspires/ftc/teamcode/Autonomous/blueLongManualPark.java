package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.*;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.states.*;
import org.firstinspires.ftc.teamcode.SubSystems.*;
import org.firstinspires.ftc.teamcode.SubSystems.arm;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public class blueLongManualPark extends LinearOpMode {
    double mult = 1.0;
    double batteryVoltage;
    String voltageCategory;
    private DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    HardwareMap hardwareMap;

    public void runOpMode(){

        frontLeftMotor = hardwareMap.get(DcMotorEx.class,"frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class,"backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class,"frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class,"backRightMotor");

        frontRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorEx.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();



    }


}
