package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
        low,medium,high, IDLE
    }
    ElapsedTime timer = new ElapsedTime();
    state armPos = state.low;
    DistanceSensor distanceSensor;

    @Override
    public void init()
    {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distancesensor");
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
        telemetry.addLine("" + distanceSensor.getDistance(DistanceUnit.INCH));
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

        switch(armPos){
            case high:
                //Wrist.setpos outtaking
                robot.Arm.setPosition(armState.outtaking);
                break;
            case medium:
                robot.Arm.setPosition(armState.medium);
                timer.reset();
                armPos = state.low;
            case low:
                if (timer.seconds() > .3) robot.Arm.setPosition(armState.low);
                if (timer.seconds() > .5){
                    //Wrist.setpos intaking
                    armPos = state.IDLE;
                }
            case IDLE:
                
        }

    }
}