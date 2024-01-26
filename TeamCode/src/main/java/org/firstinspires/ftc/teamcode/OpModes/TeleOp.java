package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SubSystems.Robot;
import org.firstinspires.ftc.teamcode.states.armState;
import org.firstinspires.ftc.teamcode.states.outtakeStates;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode
{
    private GamepadEx driver, operator;
    private Robot robot;
    enum state{
        high,high2,high3,high4, low,low2,low3,low4,IDLE
    }
    ElapsedTime timer = new ElapsedTime();
    state armPos = state.IDLE;

    @Override
    public void init()
    {
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        robot = new Robot(hardwareMap, telemetry);
        robot.Arm.setPosition(armState.high);
    }
    @Override
    public void loop() {

        driver.readButtons();
        operator.readButtons();

        robot.drivetrain.fieldCentric(driver);


        if (gamepad1.dpad_down){
            robot.Arm.setPosition(armState.low);
        }if (gamepad1.dpad_left){
            robot.Arm.setPosition(armState.medium);
        }if (gamepad1.dpad_up){
            robot.Arm.setPosition(armState.high);
        }if (gamepad1.dpad_right){
            robot.Arm.setPosition(armState.outtaking);
        }




        if (gamepad1.square){
            robot.wrist.setPosition(armState.intakingCLAW);
        }
        if (gamepad1.triangle){
            robot.wrist.setPosition(armState.outtaking);
        }
        telemetry.update();


    }
}