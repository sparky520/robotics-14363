package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
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

public class Mecanum
{
    IMU imu;
    DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    HardwareMap hardwareMap;

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
        IMU imu = hardwareMap.get(IMU.class, "cIMU");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
    }

    public void resetIMU(){
        imu.resetYaw();
    }

    public void fieldCentric(GamepadEx gamepad1){
        double y = gamepad1.getLeftY();
        double x = -gamepad1.getLeftX();
        double rx = -gamepad1.getRightX();
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        double frontLeftPower = 1 * (rotY + rotX + rx) / denominator;
        double backLeftPower = 1 * (rotY - rotX + rx) / denominator;
        double frontRightPower = 1 * (rotY - rotX - rx) / denominator;
        double backRightPower = 1 * (rotY + rotX - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

}