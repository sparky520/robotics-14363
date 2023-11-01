package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.*;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.SubSystems.Mecanum;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {
    public void resetIMU()
    {
        imu.resetYaw();
    }
    private ElapsedTime runtime = new ElapsedTime();
    private GamepadEx driver, operator;
    private Mecanum driveTrain;
    IMU imu = hardwareMap.get(IMU.class, "imu");

    @Override
    public void init() {
        runtime = new ElapsedTime();
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        driveTrain = new Mecanum(hardwareMap);
    }
    @Override
    public void start() {
        runtime.reset();
    }
    @Override
    public void loop() {
        driver.readButtons();
        operator.readButtons();
        driveTrain.vroom(driver);
        if(driver.wasJustPressed(Button.RIGHT_BUMPER)) {
            // can be any button
            driveTrain.slowdown();
        }
    }
    @Override
    public void stop() {
        //how tf do you stop
        telemetry.addLine("Total runtime:" + getRuntime() + "(s)");
    }
}