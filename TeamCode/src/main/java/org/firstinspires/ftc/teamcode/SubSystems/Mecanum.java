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
    HardwareMap hardwareMap;
    IMU imu;    private double botHeading, x, y, rx, rotX, rotY, denominator, frontLeftPower, backLeftPower, frontRightPower, backRightPower;


    public Mecanum(HardwareMap hardwareMap)
    {
        this.hardwareMap = hardwareMap;
        frontLeftMotor = hardwareMap.get(DcMotorEx.class,"frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class,"backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class,"frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class,"backRightMotor");

        frontRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // Retrieve the IMU from the hardware map

        imu = hardwareMap.get(IMU.class, "cIMU");
        // Makes a new object titled 'parameters' usd to hold the angle of the IMU
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();


    }

    public void resetIMU()
    {

        imu.resetYaw();

    }

    public void fieldCentric(GamepadEx gamepad1){
        y = gamepad1.getLeftY();
        x = gamepad1.getLeftX();
        rx = -gamepad1.getRightX();

        botHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
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
    public void rotation(){
        if(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) != 0){
            resetIMU();
        }
    }



}