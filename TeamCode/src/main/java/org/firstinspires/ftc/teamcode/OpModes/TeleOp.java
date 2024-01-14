package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.SubSystems.Robot;
import org.firstinspires.ftc.teamcode.states.armState;
import org.firstinspires.ftc.teamcode.states.outtakeStates;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode
{
    private GamepadEx driver, operator;
    private Robot robot;
    @Override
    public void init()
    {
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        robot = new Robot(hardwareMap, telemetry);
    }
    @Override
    public void loop() {

        driver.readButtons();
        operator.readButtons();

        robot.drivetrain.fieldCentric(driver);
        if (gamepad1.dpad_up){
            robot.drivetrain.resetIMU();
        }
        if (gamepad1.triangle){
            robot.Airplane.setPosition(armState.airplaneInit);
        }
        if (gamepad1.circle){
            robot.Airplane.setPosition(armState.airplaneLaunch);
        }
        if (gamepad1.square){
            robot.Arm.setPosition(armState.outtaking2);
        }
        if (gamepad1.right_trigger > 0){
            robot.drivetrain.slow_mode = .2;
        }
        else if (gamepad1.left_trigger > 0){
            robot.drivetrain.slow_mode=.1;
        }
        else{
            robot.drivetrain.slow_mode = 1;
        }
        if (gamepad2.circle){
            robot.Arm.setPosition(armState.outtaking);
        }
        if (gamepad2.square){
            robot.Arm.setPosition(armState.low);
            //robot.Arm.topStack();
        }
        if (gamepad2.left_bumper){
            robot.Arm.topStack();
        }
        if (gamepad2.triangle){
            robot.Arm.setPosition(armState.medium);

        }
        if (gamepad1.right_bumper){
            robot.Claw.setPosition(armState.intakingCLAW);
        }
        if (gamepad1.left_bumper){
            robot.Claw.setPosition(armState.outtaking);
        }
        if (gamepad2.dpad_down){
            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
        }
        if (gamepad2.dpad_left){
            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.LOWIN);
        }
        if (gamepad2.dpad_up){
            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.MEDIUMIN);
        }
        if (gamepad2.dpad_right){
            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.AUTO_HIGH);
        }









    }
}