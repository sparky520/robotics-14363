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
        outtake,outtake2,outtake3, low,low2,low3,low4,IDLE
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

        switch (armPos){
            case outtake:
                timer.reset();
                robot.Arm.setPosition(armState.outtaking);
                armPos = state.outtake2;
                break;
            case outtake2:
                if (timer.seconds() > .3){
                    robot.wrist.setPosition(armState.outtaking);
                    armPos = state.IDLE;
                }
                break;
            case low:
                robot.Arm.setPosition(armState.medium);
                robot.wrist.setPosition(armState.intakingCLAW);
                armPos = state.low2;
                timer.reset();
                break;
            case low2:
                if (timer.seconds() > .4){
                    robot.Arm.setPosition(armState.low);
                    armPos = state.IDLE;
                }
                break;
        }
        if (gamepad1.left_bumper){
            armPos = state.low;
        }if (gamepad1.dpad_left){
            armPos = state.outtake;
        }

        if (gamepad1.left_bumper){
            robot.Claw.setPosition(armState.outtaking);
        }
        if (gamepad1.right_bumper){
            robot.Claw.setPosition(armState.intakingCLAW);
        }

        if (gamepad2.dpad_down){
            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
        }
        if (gamepad2.dpad_left){
            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.LOWIN);
        }
        if (gamepad2.dpad_up){
            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.MEDIUMIN);
        }
        if (gamepad2.dpad_right){
            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.HIGHIN);
        }
        telemetry.update();


    }
}