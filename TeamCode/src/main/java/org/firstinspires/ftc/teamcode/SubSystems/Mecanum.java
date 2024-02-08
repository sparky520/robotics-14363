package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.drive.Drive;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

public class Mecanum
{
    private DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private double offset = 1;
    BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    private double x, y, rx, rotX, rotY, denominator, frontLeftPower, backLeftPower, frontRightPower, backRightPower;
    public double slow_mode, botHeading ;
    private double lastError = 0;
    ElapsedTime timer = new ElapsedTime();
    double integralSum = 0;
    //.0005
    double kp = DriveConstants.MAX_ANG_VEL;
    double ki = DriveConstants.MAX_VEL;
    //.75
    double kd = DriveConstants.MAX_ANG_ACCEL;
    double output,  error;

    public Mecanum(HardwareMap hardwareMap)
    {
        slow_mode = 1;
        backRightMotor = hardwareMap.get(DcMotorEx.class,"frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class,"backLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class,"frontRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotorEx.class,"backRightMotor");

        frontRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorEx.Direction.REVERSE);



        // Retrieve the IMU from the hardware map


        imu = hardwareMap.get(BNO055IMU.class, "cIMU");
        // this is making a new object called 'parameters' that we use to hold the angle the imu is at
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);



    }
    public void driveAngleLock(mecanumState mecanumState, GamepadEx gamepad1){
        kp = DriveConstants.MAX_ANG_VEL;
        ki = DriveConstants.MAX_VEL;
        //.75
        kd = DriveConstants.MAX_ANG_ACCEL;
        switch (mecanumState){
            case NORMAL:
                y = gamepad1.getLeftY();
                x = gamepad1.getLeftX();
                rx = -gamepad1.getRightX();

                botHeading = -imu.getAngularOrientation().firstAngle;

                rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
                rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

                denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                frontLeftPower = (rotY + rotX + rx) / denominator;
                backLeftPower = (rotY - rotX + rx) / denominator;
                frontRightPower = (rotY - rotX - rx) / denominator;
                backRightPower = (rotY + rotX - rx) / denominator;

                frontLeftMotor.setPower(frontLeftPower*.995*slow_mode);
                backLeftMotor.setPower(backLeftPower*slow_mode);
                frontRightMotor.setPower(frontRightPower *slow_mode);
                backRightMotor.setPower(backRightPower*.885*slow_mode);                break;
            case TOREDBACKBOARD:
                y = gamepad1.getLeftY();
                x = gamepad1.getLeftX();
                // double error = angleWrap(Math.toRadians(90) - imu.getAngularOrientation().firstAngle);
                // rx = .1*(Math.toRadians(90)-imu.getAngularOrientation().firstAngle);
                // rx = gamepad1.getRightX(); // 0.01 * (des_angle - curr_angle)
                error = smallestAngleDifference(90, imu.getAngularOrientation().firstAngle * (180/Math.PI));


                timer.reset();
                output = (error * -kp) + (imu.getAngularVelocity().zRotationRate * kd) + (integralSum * ki);
                rx = output;

                botHeading = -imu.getAngularOrientation().firstAngle;



                rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
                rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

                denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                frontLeftPower = (rotY + rotX + rx) / denominator;
                backLeftPower = (rotY - rotX + rx) / denominator;
                frontRightPower = (rotY - rotX - rx) / denominator;
                backRightPower = (rotY + rotX - rx) / denominator;

                frontLeftMotor.setPower(frontLeftPower*.995*slow_mode);
                backLeftMotor.setPower(backLeftPower*slow_mode);
                frontRightMotor.setPower(frontRightPower *slow_mode);
                backRightMotor.setPower(backRightPower*.885*slow_mode);
                break;
            case TOBLUEBACKBOARD:
                y = gamepad1.getLeftY();
                x = gamepad1.getLeftX();
                // double error = angleWrap(Math.toRadians(90) - imu.getAngularOrientation().firstAngle);
                // rx = .1*(Math.toRadians(90)-imu.getAngularOrientation().firstAngle);
                 rx = gamepad1.getRightX(); // 0.01 * (des_angle - curr_angle)
                error = smallestAngleDifference(270, imu.getAngularOrientation().firstAngle * (180/Math.PI));


                timer.reset();
                output = (error * -kp) + (imu.getAngularVelocity().zRotationRate * kd) + (integralSum * ki);
                rx = output;

                botHeading = -imu.getAngularOrientation().firstAngle;


                rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
                rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

                denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                frontLeftPower = (rotY + rotX + rx) / denominator;
                backLeftPower = (rotY - rotX + rx) / denominator;
                frontRightPower = (rotY - rotX - rx) / denominator;
                backRightPower = (rotY + rotX - rx) / denominator;

                frontLeftMotor.setPower(frontLeftPower*.995*slow_mode);
                backLeftMotor.setPower(backLeftPower*slow_mode);
                frontRightMotor.setPower(frontRightPower *slow_mode);
                backRightMotor.setPower(backRightPower*.885*slow_mode);
                break;

        }
    }



    public void resetIMU()
    {

        imu.initialize(parameters);

    }

    public void fieldCentric(GamepadEx driver){
        y = driver.getLeftY();
        x = driver.getLeftX();
        rx = -driver.getRightX();
        botHeading = -imu.getAngularOrientation().firstAngle;

        rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
        denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        frontLeftPower = 1 * (rotY + rotX + rx) / denominator;
        backLeftPower = 1 * (rotY - rotX + rx) / denominator;
        frontRightPower = 1 * (rotY - rotX - rx) / denominator;
        backRightPower = 1 * (rotY + rotX - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower*.995*slow_mode);
        backLeftMotor.setPower(backLeftPower*slow_mode);
        frontRightMotor.setPower(frontRightPower *slow_mode);
        backRightMotor.setPower(backRightPower*.885*slow_mode);
    }
    public double angleWrap360(double angle){
        angle = angle % 360;
        if(angle < 0.0){
            angle += 360;
        }
        return angle;

    }

    public double angleWrap180(double angle){
        angle = angleWrap360(angle);
        if(angle > 180.0){
            angle-=360;
        } else if(angle < -180.0) {
            angle+=360;
        }
        return angle;
    }
    public double smallestAngleDifference(double current, double desired){
        current = angleWrap360(current);
        desired = angleWrap360(desired);

        double difference = current - desired;
        if(difference > 180.0){
            difference = -(360.0 - difference);
        }
        else if(difference < -180.0){
            difference = 360.0 + difference;
        }
        return difference;
    }





}