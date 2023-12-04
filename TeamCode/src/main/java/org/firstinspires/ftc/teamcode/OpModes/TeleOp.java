package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.*;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.SubSystems.IntakeSlide;
import org.firstinspires.ftc.teamcode.SubSystems.Mecanum;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode
{
    private GamepadEx driver, operator;

    private Mecanum driveTrain;

    @Override
    public void init()
    {

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        driveTrain = new Mecanum(hardwareMap);

    }
    @Override
    public void loop() {
        driver.readButtons();
        operator.readButtons();

        driveTrain.fieldCentric(driver);

        if(driver.wasJustPressed(Button.Y)) {
            driveTrain.resetIMU();
        }

        if(driver.wasJustPressed(Button.DPAD_DOWN)){
            driveTrain.slowDown();
        }
        if(driver.wasJustPressed(Button.DPAD_UP)){
            driveTrain.speedUp();
        }

    }

}