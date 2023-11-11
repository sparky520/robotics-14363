package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Mecanum
{
    private DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private double offset = 1;
    BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    private double x, y, rx, rotX, rotY, denominator, frontLeftPower, backLeftPower, frontRightPower, backRightPower;



    public Mecanum(HardwareMap hardwareMap)
    {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class,"frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class,"backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class,"frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class,"backRightMotor");

        frontRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // Retrieve the IMU from the hardware map


        imu = hardwareMap.get(BNO055IMU.class, "cIMU");
        // this is making a new object called 'parameters' that we use to hold the angle the imu is at
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);



    }

    public void resetIMU()
    {

        imu.initialize(parameters);

    }

    public void fieldCentric(GamepadEx driver){
        y = driver.getLeftY();
        x = driver.getLeftX();
        rx = -driver.getRightX();

        double botHeading = -imu.getAngularOrientation().firstAngle;

        rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
        denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        frontLeftPower = 1 * (rotY + rotX + rx) / denominator;
        backLeftPower = 1 * (rotY - rotX + rx) / denominator;
        frontRightPower = 1 * (rotY - rotX - rx) / denominator;
        backRightPower = 1 * (rotY + rotX - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower * offset);
        backLeftMotor.setPower(backLeftPower * offset);
        frontRightMotor.setPower(frontRightPower * offset);
        backRightMotor.setPower(backRightPower * offset);
    }




}