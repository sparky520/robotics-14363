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
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public double slow_mode, botHeading ;
    private double lastError = 0;
    ElapsedTime timer = new ElapsedTime();
    double integralSum = 0;
    double kp = 0.02;
    double ki = 0;
    double kd = 1;
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
        switch (mecanumState){
            case NORMAL:
                y = gamepad1.getLeftY();
                x = gamepad1.getLeftX();
                rx = gamepad1.getRightX();

                botHeading = -imu.getAngularOrientation().firstAngle;

                rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
                rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

                denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                frontLeftPower = (rotY + rotX + rx) / denominator;
                backLeftPower = (rotY - rotX + rx) / denominator;
                frontRightPower = (rotY - rotX - rx) / denominator;
                backRightPower = (rotY + rotX - rx) / denominator;
                break;
            case TOREDBACKBOARD:
                y = gamepad1.getLeftY();
                x = gamepad1.getLeftX();
                // double error = angleWrap(Math.toRadians(90) - imu.getAngularOrientation().firstAngle);
                // rx = .1*(Math.toRadians(90)-imu.getAngularOrientation().firstAngle);
                // rx = gamepad1.getRightX(); // 0.01 * (des_angle - curr_angle)
                error = small(90, imu.getAngularOrientation().firstAngle * (180/Math.PI));


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
                break;
            case TOBLUEBACKBOARD:
                y = gamepad1.getLeftY();
                x = gamepad1.getLeftX();
                // double error = angleWrap(Math.toRadians(90) - imu.getAngularOrientation().firstAngle);
                // rx = .1*(Math.toRadians(90)-imu.getAngularOrientation().firstAngle);
                // rx = gamepad1.getRightX(); // 0.01 * (des_angle - curr_angle)
                error = small(270, imu.getAngularOrientation().firstAngle * (180/Math.PI));


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
    public double wrap(double angle){
        if (angle > 180){
            angle -= 360;
        }
        else if (angle < -180){
            angle += 360;
        }
        return angle;
    }
    public double small(double target, double current){
        current = wrap(current);
        target = wrap(target);
        double dif = target - current;
        if (dif < -180){
            dif += 360;
        }
        else if (dif > 180){
            dif = -(360-dif);
        }
        return dif;
    }
    public void angleLock(GamepadEx  driver, double angle){
        y = driver.getLeftY();
        x = driver.getLeftX();
        double error = small(angle ,imu.getAngularOrientation().firstAngle * (180/Math.PI));

        integralSum += error * timer.seconds();
        double derivative = -(wrap(error - lastError)) / (timer.seconds());
        lastError = error;


        timer.reset();
        double output = (error * -kp) + (imu.getAngularVelocity().zRotationRate * kd) + (integralSum * ki);
        double rx = output;

        double botHeading = -imu.getAngularOrientation().firstAngle;



        rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        frontLeftPower = (rotY + rotX + rx) / denominator;
        backLeftPower = (rotY - rotX + rx) / denominator;
        frontRightPower = (rotY - rotX - rx) / denominator;
        backRightPower = (rotY + rotX - rx) / denominator;
    }




}