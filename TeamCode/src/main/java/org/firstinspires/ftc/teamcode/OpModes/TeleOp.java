package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SubSystems.Robot;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.states.armState;
import org.firstinspires.ftc.teamcode.states.outtakeStates;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode
{
    private GamepadEx driver, operator;
    private Robot robot;
    public TouchSensor touchSensor;
    public ElapsedTime timer = new ElapsedTime();
    ColorRangeSensor claw1,claw2,color1;
    double sensor1Val, sensor2Val, sensor3Val;
    enum state{
        outtaking,intaking
    }
    enum arm{
        low,medium,high
    }
    arm armPos = arm.low;
    DistanceSensor distanceSensor3,distanceSensor2, distanceSensor,distanceSensor4;
    double distanceFront, distanceLeftSide, distanceBack, distanceRightSide;
    state currentState = state.intaking;
    @Override
    public void init()
    {
        telemetry.setMsTransmissionInterval(50);
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        robot = new Robot(hardwareMap, telemetry);
        robot.wrist.setPosition(armState.intakingCLAW);
        robot.Arm.setPosition(armState.low);
        robot.Claw.setPosition(armState.open);
        robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.STATION);
        color1 = hardwareMap.get(ColorRangeSensor.class, "colorBoard");
        claw1 = hardwareMap.get(ColorRangeSensor.class, "claw1");
        claw2 = hardwareMap.get(ColorRangeSensor.class, "claw2");
        distanceSensor3 = hardwareMap.get(DistanceSensor.class, "distanceSensor3");
        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");
    }
    @Override
    public void loop() {
        driver.readButtons();
        operator.readButtons();
        robot.drivetrain.fieldCentric(driver);

        switch (currentState){
            case intaking:
                sensor1Val = claw1.blue()+claw1.red()+claw1.green();
                sensor2Val = claw2.blue()+claw2.red()+claw2.green();
                if (sensor2Val > 175){
                    robot.Claw.closeRight();
                }else{
                    robot.Claw.openRight();
                }
                if (sensor1Val > 175){
                    robot.Claw.closeLeft();
                }else{
                    robot.Claw.openLeft();
                }
                break;
            case outtaking:
                distanceBack = distanceSensor3.getDistance(DistanceUnit.INCH);
                sensor3Val = color1.blue()+color1.red()+color1.green();
                if (gamepad2.right_trigger > 0 && sensor3Val > 200){
                    robot.Claw.setPosition(armState.open);
                }
                break;
        }


        if (touchSensor.isPressed()){
            robot.slide.resetEncoders();
        }
        if (gamepad2.circle){
            currentState = state.outtaking;
            robot.Arm.setPosition(armState.outtaking);
            robot.wrist.setPosition(armState.outtaking);
            armPos = arm.high;
        }
        if (gamepad2.triangle){
            currentState = state.outtaking;
            robot.Arm.setPosition(armState.medium);
            robot.wrist.setPosition(armState.intakingCLAW);
            armPos = arm.medium;
        }
        if (gamepad2.square){
            currentState = state.intaking;
            robot.Arm.setPosition(armState.low);
            robot.wrist.setPosition(armState.intakingCLAW);
            robot.Claw.setPosition(armState.open);
            armPos = arm.low;
        }
        if (gamepad2.dpad_up && armPos != arm.medium){
            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.MEDIUM);
        }
        if (gamepad2.dpad_left && armPos != arm.medium){
            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.LOW);
        }
        if (gamepad2.dpad_right && armPos != arm.medium){
            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.HIGH);
            currentState = state.outtaking;
            robot.Arm.highOuttake();
            robot.wrist.highOuttake();
            armPos = arm.high;
        }

        if (gamepad2.dpad_down){
            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending,outtakeStates.STATION);
        }
        if (gamepad1.square){
            robot.Airplane.setPosition(armState.airplaneLaunch);
        }
        if (gamepad1.triangle){
            robot.drivetrain.resetIMU();
        }
        if (gamepad1.right_trigger > 0){
            robot.drivetrain.slow_mode = .25;
        }else if (gamepad1.left_trigger > 0){
            robot.drivetrain.slow_mode = .5;
        }
        else{
            robot.drivetrain.slow_mode = 1;
        }
        if (gamepad1.left_bumper){
            robot.Claw.openLeft();
        }
        if (gamepad1.right_bumper){
            robot.Claw.openRight();
        }

        telemetry.update();
    }
    public void initDistance(){
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "distanceSensor2");
        distanceSensor3 = hardwareMap.get(DistanceSensor.class, "distanceSensor3");
        distanceSensor4 = hardwareMap.get(DistanceSensor.class, "distanceSensor4");
    }
    public void distanceTelem(){
        distanceFront = distanceSensor.getDistance(DistanceUnit.INCH);
        distanceRightSide = distanceSensor2.getDistance(DistanceUnit.INCH);
        distanceBack = distanceSensor3.getDistance(DistanceUnit.INCH);
        distanceLeftSide = distanceSensor4.getDistance(DistanceUnit.INCH);
        telemetry.addData("right distance", distanceLeftSide);
        telemetry.addData("left distance", distanceRightSide);
        telemetry.addData("front distance", distanceFront);
        telemetry.addData("back distance", distanceBack);
    }
}