package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.*;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.states.*;
import org.firstinspires.ftc.teamcode.SubSystems.*;
import org.firstinspires.ftc.teamcode.SubSystems.arm;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode
{
    private GamepadEx driver, operator;
    private Robot robot;
    private Mecanum driveTrain;


    private Servo leftArm, rightArm;
    Mecanum wheels;
    @Override
    public void init()
    {
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        robot = new Robot(hardwareMap, telemetry);

//        leftArm.setPosition(0);
//        rightArm.setPosition(0);

    }
    @Override
    public void loop() {

        driver.readButtons();
        operator.readButtons();

        robot.drivetrain.fieldCentric(driver);

       if (gamepad2.dpad_up){
            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.HIGHIN);
       }
      if (gamepad2.dpad_right){
           robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.MEDIUMIN);
      }
      if (gamepad2.dpad_down){
           robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.STATION);
      }
        if (gamepad2.dpad_left){
            robot.slide.setOuttakeSlidePosition(outtakeStates.etxending, outtakeStates.LOWIN);
        }
      if (gamepad2.right_bumper){
          robot.Arm.setPosition(armState.outtaking);
      }
      if (gamepad2.left_bumper){

          robot.Arm.setPosition(armState.intakingHOVERUP);

      }
      if (gamepad2.square){

          robot.Arm.setPosition(armState.intakingHOVERBOTTOM);

      }
      if (gamepad2.circle){
          robot.Arm.setPosition(armState.intakingBOTTOM);
      }
      if (gamepad1.left_bumper){
          robot.Claw.setPosition(armState.intakingCLAW);
      }
      if (gamepad1.right_bumper){
          robot.Claw.setPosition(armState.outtaking);
      }
      if (gamepad2.triangle)
      {
          //robot.Airplane.launchAirplane();
      }







    }
}