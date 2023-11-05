package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.*;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SubSystems.IntakeSlide;
import org.firstinspires.ftc.teamcode.SubSystems.Mecanum;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode
{
    private GamepadEx driver, operator;

    private Mecanum driveTrain;
    private IntakeSlide intake;
    private double intakeSpeed = 0.35;
    Mecanum wheels;
    @Override
    public void init()
    {
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        driveTrain = new Mecanum(hardwareMap);
        intake = new IntakeSlide(hardwareMap);

    }
    @Override
    public void loop() {
        driver.readButtons();
        operator.readButtons();

        driveTrain.fieldCentric(driver);
        driveTrain.rotation();

        if(driver.wasJustPressed(Button.DPAD_UP)) {
            wheels.resetIMU();
        }
        if (driver.wasJustPressed(Button.DPAD_LEFT)){
            intake.on_off(intakeSpeed);
            }

        if(driver.wasJustPressed(Button.DPAD_RIGHT)){
            intake.on_off(0);
        }

    }

}