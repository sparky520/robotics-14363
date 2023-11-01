package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
public class Slides {

    private DcMotorEx controlSlideMotor, expansionSlideMotor;
    HardwareMap hardwareMap;
    private GamepadEx driver, operator;
    private double controlSlidePower, expansionSlidePower;



    public Slides (HardwareMap hardwareMap)
    {
        controlSlideMotor = hardwareMap.get(DcMotorEx.class, "ControlSlideMotor");
        expansionSlideMotor = hardwareMap.get(DcMotorEx.class,"ExpansionSlideMotor");

        controlSlideMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }
    public void extend (GamepadEx gamepad1) {
        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            controlSlideMotor.setPower(1.0);
            expansionSlideMotor.setPower(1.0);
        }
        //if(driver.wasJustPressed(GamepadKeys.Button.A)){
        //controlSlideMotor.setTargetPosition(2);
        //expansionSlideMotor.setTargetPosition(2);
        //}
        //if(driver.wasJustPressed(GamepadKeys.Button.A)){
        // controlSlideMotor.setTargetPosition(3);
        //  expansionSlideMotor.setTargetPosition(3);
        // }
    }
        public void retract(GamepadEx gamepad1) {
        if (driver.wasJustPressed(GamepadKeys.Button.Y)) {
            controlSlideMotor.setPower(-1.0);
            expansionSlideMotor.setPower(-1.0);
        }
    }
}
