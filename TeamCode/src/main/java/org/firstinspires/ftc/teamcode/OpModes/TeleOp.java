package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SubSystems.Robot;
import org.firstinspires.ftc.teamcode.states.armState;
import org.firstinspires.ftc.teamcode.states.outtakeStates;
import org.firstinspires.ftc.teamcode.SubSystems.finger.*;

import java.util.ArrayList;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode
{
    private GamepadEx driver, operator;
    private Robot robot;
    public TouchSensor touchSensor;
    ColorRangeSensor claw1,claw2,color1;
    double sensor1Val, sensor2Val, sensor3Val;
    enum clawState{
        open,close,
    }
    boolean outtaking = false;
    clawState leftClawState = clawState.open;
    clawState rightClawState = clawState.open;
    DistanceSensor distanceSensor3;
    boolean autoOuttake = false;
    @Override
    public void init()
    {
        distanceSensor3 = hardwareMap.get(DistanceSensor.class, "distanceSensor3");
        telemetry.setMsTransmissionInterval(50);
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        robot = new Robot(hardwareMap, telemetry);
        robot.wrist.setPosition(armState.intakingCLAW);
        robot.Arm.setPosition(armState.low);
        robot.Claw.setPosition(armState.open);
        color1 = hardwareMap.get(ColorRangeSensor.class, "colorBoard");
        claw1 = hardwareMap.get(ColorRangeSensor.class, "claw1");
        claw2 = hardwareMap.get(ColorRangeSensor.class, "claw2");
        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.TELEOPSTATION);
    }
    @Override
    public void loop() {
        driver.readButtons();
        operator.readButtons();
        robot.drivetrain.fieldCentric(driver);
        sensor1Val = claw1.blue()+claw1.red()+claw1.green();
        sensor2Val = claw2.blue()+claw2.red()+claw2.green();
        sensor3Val = color1.blue()+color1.red()+color1.green();
        telemetry.addData("claw1",  sensor1Val);
        telemetry.addData("claw2", sensor2Val);
        telemetry.addData("colorBoard", sensor3Val);
        telemetry.addData("d back", distanceSensor3.getDistance(DistanceUnit.INCH));
        if (sensor1Val > 200 && !outtaking){
            robot.Claw.closeRight();
        }
        if (sensor2Val > 200 && !outtaking){
            robot.Claw.closeLeft();
        }
        /*
        if (autoOuttake){
            if (sensor3Val > 700 && outtaking){
                robot.Claw.setPosition(armState.open);
            }
        }*/
        if (gamepad1.triangle){
            robot.drivetrain.resetIMU();
        }
        if (gamepad1.right_trigger > 0){
            robot.drivetrain.slow_mode = .15;
        }else if (gamepad1.left_trigger > 0){
            robot.drivetrain.slow_mode = .3;
        }
        else{
            robot.drivetrain.slow_mode = 1;
        }

        if (gamepad2.circle){
            outtaking = true;
            robot.Arm.setPosition(armState.outtaking);
            robot.wrist.setPosition(armState.outtaking);
        }
        if (gamepad2.triangle){
            outtaking = false;
            robot.Arm.setPosition(armState.medium);
            robot.wrist.setPosition(armState.intakingCLAW);
        }
        if (gamepad2.square){
            outtaking = false;
            robot.Arm.setPosition(armState.low);
            robot.wrist.setPosition(armState.intakingCLAW);
        }
        if (gamepad2.dpad_up){

            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.MEDIUMIN);
        }
        if (gamepad2.dpad_left){
            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.LOWIN);
        }
        if (gamepad2.dpad_right){
            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.HIGHIN);
        }

        if (gamepad2.dpad_down){
            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.TELEOPSTATION);
        }
        if (gamepad2.left_bumper){
            robot.slide.driftOffset += 10;
        }
        if (gamepad2.right_bumper){
            robot.slide.driftOffset += 10;
        }
        if (gamepad1.square){
            robot.Airplane.setPosition(armState.airplaneLaunch);
        }

        if (gamepad1.left_bumper){
            robot.Claw.openLeft();
        }
        if (gamepad1.right_bumper){
            robot.Claw.openRight();
        }
        telemetry.update();



    }
}