package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.*;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import org.firstinspires.ftc.teamcode.Commands.outtakeSlidesState;
import org.firstinspires.ftc.teamcode.SubSystems.Mecanum;
import org.firstinspires.ftc.teamcode.SubSystems.Outtake;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode
{
    private GamepadEx driver, operator;

    private Mecanum driveTrain;
    Mecanum wheels;
    @Override
    public void init()
    {
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        driveTrain = new Mecanum(hardwareMap);
        Outtake outtakeSlide = new Outtake(hardwareMap);
    }
    @Override
    public void loop() {
        driver.readButtons();
        operator.readButtons();

        driveTrain.fieldCentric(driver);

        if(gamepad1.y) {
            wheels.resetIMU();
        }


        Outtake outtakeSlide = new Outtake(hardwareMap);
        if (gamepad1.dpad_up){outtakeSlide.setSlidePosition("high","extending");}
        if (gamepad1.dpad_left){outtakeSlide.setSlidePosition("medium","extending");}
        if (gamepad1.dpad_down){outtakeSlide.setSlidePosition("station","extending");}

    }
}